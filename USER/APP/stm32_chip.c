#include "stm32f10x.h" 
#include "lcd\\gl_ui.h"
#include "flash.h"
#include "md5.h"
// #include "stm32_chip.h"


#define LOG_CACHE (4*6)

#define DATE_PER_MONTH (31)
struct pro_licence
{
	char key[4];					// 公钥
	unsigned long licence[4];		// 注册码
	unsigned long rand_hw;			// 机器码随机值
	unsigned long licence_times; 	// 注册次数
	unsigned long date;			 	// 使用期限，最大2年
	char log[LOG_CACHE];		 	// 使用次数记录
	unsigned long unuse0;			// 防止数据访问溢出，无效数据
};

static int UseTick(int bwrite);
static void calc_licence(unsigned long *lic, unsigned long month);

static int WriteProLicence(struct pro_licence *puselog);

int g_licence_timeout = 0;				///< 许可证是否超时
// struct pro_licence 必须4字节对齐，方便Flash读写操作


// // 芯片机器码
// struct chiip_id
// {
// 	unsigned long id0;
// 	unsigned long id1;
// 	unsigned long id2;
// 	unsigned long id_rand;   ///<随机值，来源于第一次开机存放于struct pro_licence的rand_hw
// };
// 采用AD值做随机值
extern volatile uint16_t ADCConvertedValue[2000];
unsigned long Rand()
{
	return 	(ADCConvertedValue[1] << 2) + 
			(ADCConvertedValue[3] << 4) + 
			(ADCConvertedValue[5] << 8) + 
			(ADCConvertedValue[7]);
}

// *****************************************************************************
// 对外接口
// id 长度为4Byte
void lc_GetChipID(unsigned long *id)
{
	struct pro_licence uselog;
	char strout[256];
	
	*(id+2) = *(__IO u32 *)(0X1FFFF7E8);  // 低字节
	*(id+1) = *(__IO u32 *)(0X1FFFF7EC); // 
	*(id+0) = *(__IO u32 *)(0X1FFFF7F0); // 高字节
	ReadProLicence(&uselog);
	*(id+3) = uselog.rand_hw;
}
void lc_GetChipMonth(unsigned long *month)
{
	struct pro_licence uselog;
	ReadProLicence(&uselog);
	*month = uselog.date / DATE_PER_MONTH;

	if (*month == 0) {
		*month = 1;
	}
	else if (*month > 24) {
		*month = 24;
	}
}

void lc_GetChipleave(unsigned long *leave)
{
	*leave = UseTick(0);
}


#define MONTH_FOREVERY 71
#define MONTH_RESET		72
#define MOUNT_LIMIT		24

#define INFINITUDE  (DATE_PER_MONTH * MONTH_FOREVERY)
#define LIMIT_DATE  (DATE_PER_MONTH * MOUNT_LIMIT)
void lc_CheckMonth(unsigned long *month)
{
	int i;
	struct pro_licence uselog;

	switch (*month) {
	case MONTH_FOREVERY:
		return ;
	case MONTH_RESET:
		// TODO reset page FLASH_PAGE_LICENCE
		// WriteFlash
		uselog.key[0] = -1;
		uselog.key[1] = -1;
		uselog.key[2] = -1;
		uselog.key[3] = -1;
		uselog.licence[0] = -1;
		uselog.licence[1] = -1;
		uselog.licence[2] = -1;
		uselog.licence[3] = -1;
		uselog.rand_hw = -1;
		uselog.licence_times = -1;
		uselog.date = -1;
		for (i = 0; i < LOG_CACHE; i++) {
			uselog.log[i] = -1;	
		}
		
		uselog.unuse0 = -1;
		WriteProLicence(&uselog);
		return ;
	}
	if (*month > MOUNT_LIMIT) {
		*month = MOUNT_LIMIT;
	}
}

/**
 * @brief	输入注册码
 * @param	licence 注册码 4 word
 * @param	month 注册时间
 * @retval	1 注册成功
 * @retval	1 注册失败
 * @remarks	
 * @see	
 */

int lc_InputLicence(unsigned long *licence, unsigned long *month)
{
	struct pro_licence uselog;
	unsigned long licence_true[4];
	char strout[111];
	calc_licence(licence_true, *month);

	// sprintf(strout,"%8.8u %8.8u %8.8u %8.8u \n", 
	// 	licence_true[0],licence_true[1],licence_true[2],licence_true[3]);
	// gl_text(0,100,strout, -1);
	if (licence_true[3] == licence[0]) {
		uselog.licence[0] = licence_true[0];
		uselog.licence[1] = licence_true[1];
		uselog.licence[2] = licence_true[2];
		uselog.licence[3] = licence_true[3];
		uselog.date = *month * DATE_PER_MONTH;
		// WriteProLicence(&uselog);
		return 1;
	}
	else {
		return 0;
	}
}
// End 对外接口
// *****************************************************************************




// 1. 写入硬件序列号
// 2. 写入授权Licence
static int WriteProLicence(struct pro_licence *puselog)
{

	WriteFlash(FLASH_PAGE_LICENCE,
		(uint32_t*)(puselog),
		sizeof(struct pro_licence));
}


/**
 * @brief	仅写入使用次数记录
 */

int WriteTick(struct pro_licence *puselog)
{

	WriteFlash(FLASH_PAGE_LICENCE,
		(uint32_t*)(puselog),
		sizeof(struct pro_licence));
}

/**
 * @brief	读取Licence，第一次启动后无Licence，并生成硬件序列号
 */

int ReadProLicence(struct pro_licence *puselog)
{
	int i;
	char *plog;

	ReadFlash(FLASH_PAGE_LICENCE,
		(uint32_t*)(puselog),
		sizeof(struct pro_licence));

	// 检验是否存在硬件序列号，没有则生成
	if (puselog->rand_hw == (unsigned long)(-1)) {
		puselog->rand_hw = Rand();
		// 强制将使用日志复位，防止 rand_hw 是 0xff，但日志并不是全0xff
		plog = &puselog->log[0];
		// for (i = 0; i < LOG_CACHE; i++) {
		// 	*plog = 0xff;
		// 	plog++;
		// }
		// 复位licence页面
		WriteProLicence(puselog);
	}
	// 授权期限最大2年，这是个虚值，开始时并未写入Flash，只有获取有效Licence后才写入具体值
	if (puselog->date > LIMIT_DATE) {
		puselog->date = LIMIT_DATE;
	}
	return 0;
}



/**
 * @brief	使用计数
 * @param	bwrite 是否将新使用次数写入Flash\n
 			1：写入，并剩余使用次数
 			0: 仅剩余使用次数
 * @retval	剩余使用次数

 * @remarks	
 * @see	
 */

int UseTick(int bwrite)
{
	int i;
	// char *plog;
	unsigned long *plog;
	struct pro_licence uselog;
	int istimeout = 0;
	char strout[256];
	unsigned long start, child;
	int use = 0;

	
	ReadProLicence(&uselog);

	if (INFINITUDE == uselog.date) {
		return INFINITUDE;
	}

	for (i = 0; i < 3 ;i++) {
		sprintf(strout, "k%2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x",
			uselog.log[i*16 + 0],uselog.log[i*16 + 1],uselog.log[i*16 + 2],uselog.log[i*16 + 3],
			uselog.log[i*16 + 4],uselog.log[i*16 + 5],uselog.log[i*16 + 6],uselog.log[i*16 + 7],
			uselog.log[i*16 + 8],uselog.log[i*16 + 9],uselog.log[i*16 +10],uselog.log[i*16 +11],
			uselog.log[i*16 +12],uselog.log[i*16 +13],uselog.log[i*16 +14],uselog.log[i*16 +15]
			);
		gl_text(0, 90 + i*10,strout, -1);
	}
	plog = (unsigned long*)&uselog.log[0];
	istimeout = 1;
	for (i = 0; i < LOG_CACHE/4; i++) {
		// if ( *plog == *(plog + 1) && *plog != 0) {
		// 	*plog = 0;
		// 	istimeout = 0;
		// 	break;
		// }
		switch(*plog) {
		case 0xFFFFFFFF:
			*plog = 0xffffff00;
			use += 0;
			goto _Write;
			break;
		case 0xffffff00:
			*plog = 0xffff0000;
			use += 1;
			goto _Write;
			break;
		case 0xffff0000:
			*plog = 0xff000000;
			use += 2;
			goto _Write;
			break;
		case 0xff000000:
			*plog = 0x00000000;
			use += 3;
			goto _Write;
			break;
		default:
			*plog = 0x00000000;
			use += 4;
			break;
		}
		plog++;
	}
_Write:;
	start = (unsigned long)&uselog;
	child = (unsigned long)plog;
	// child = (unsigned long)&uselog.log[0];
	if (bwrite == 1) {
		// WriteTick(&uselog);
		WriteFlash0(FLASH_PAGE_LICENCE ,
			 (child - start),
			(uint32_t*)plog,
			4);
	}
	
	return LOG_CACHE - use;
}

// 显示使用计数
int ShowTick()
{
	return UseTick(0);
}

// 显示使用期限，ShowTick与此值相等时Licence失效
int ShowTotal()
{

}




// static void calc_licence1(unsigned long *lic, unsigned long month)
// {
// 	unsigned long id[4];
// 	unsigned long licence[4];
// 	unsigned long val;

// 	lc_GetChipID(&id[0]);

// 	id[0] = 0x43077332;
// 	id[1] = 0x30364734;
// 	id[2] = 0x05d5ff33;
// 	id[3] = 0xc1d2;

// 	val = id[0] ^ id[1] ^ id[2] ^ id[3] ^ month;
// 	licence[0] = val ^ id[0];
// 	val += licence[0];

// 	licence[1] = val ^ id[1];
// 	val += licence[1];

// 	licence[2] = val ^ id[2];
// 	val += licence[2];

// 	licence[3] = val ^ id[3];
// 	val += licence[3];

// 	val += licence[0] + licence[1] + licence[2] + licence[3];
// 	*(lic + 0) = licence[0];
// 	*(lic + 1) = licence[1];
// 	*(lic + 2) = licence[2];
// 	*(lic + 3) = licence[3];
// }

/**
 * @brief	计算正确的序列号
 * @param	lic 输出序列号值，lic是 unsigned long [4]
 * @see	
 */
static void calc_licence(unsigned long *lic, unsigned long month)
{
	int i;
	unsigned long id[4];
	unsigned long licence[4];
	unsigned long val;
	unsigned char encrypt[64] ="admin";//21232f29 7a57a5a7 43894a0e 4a801fc3
	unsigned char decrypt[16];
	char strout[120];
	MD5_CTX md5;

	lc_GetChipID(&id[0]);
	sprintf(encrypt, "%8.8x-%8.8x-%8.8x-%8.8x-%2.2d",
		id[0],id[1],id[2],id[3],month);
	gl_text(0,180, encrypt,-1);

	MD5Init(&md5);         		
	MD5Update(&md5,encrypt,strlen((char *)encrypt));
	MD5Final(&md5,decrypt);        
	// printf("加密前:%s\n加密后:",encrypt);
	// for(i=0;i<16;i++)
	// {
	// 	// printf("%02x",decrypt[i]);
	// }
	sprintf(strout, "%02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x",
		decrypt[0],decrypt[1],decrypt[2],decrypt[3],decrypt[4],decrypt[5],
		decrypt[6],decrypt[7],decrypt[8],decrypt[9],decrypt[10],decrypt[11],
		decrypt[12],decrypt[13],decrypt[14],decrypt[15]);

	*(lic + 0) = decrypt[3] + (decrypt[2] << 8) + (decrypt[1] << 16) + (decrypt[0] << 24);
	*(lic + 1) = decrypt[7] + (decrypt[6] << 8) + (decrypt[5] << 16) + (decrypt[4] << 24);
	*(lic + 2) = decrypt[11] + (decrypt[10] << 8) + (decrypt[9] << 16) + (decrypt[8] << 24);
	*(lic + 3) = decrypt[15] + (decrypt[14] << 8) + (decrypt[13] << 16) + (decrypt[12] << 24);

	gl_text(0,200, strout,-1);
	
	// getchar();
	
	return ;
}

// return 0 非法
// return 1 正确
/**
 * @brief	检验许可证是许可
 * @param	输入序列号
 * @retval	0 无序列号
 * @retval	1 授权正确
 * @retval	2 授权期限到
 * @remarks	licence是一个数值，在这里可以制造一个陷阱，采用内存溢出的方式访问
 * @see	
 */

int lc_CheckLicence(unsigned long licence[4])
{
	int i;
	char *plog;
	struct pro_licence uselog;
	unsigned long id[4];
	int istimeout = 0;
	char strout[256];
	unsigned long val, licence_true[4];

	
	
	// 读取RandHW
	// 读取Licence
	ReadProLicence(&uselog);
	// 比较Licence
	lc_GetChipID(&id[0]);
	sprintf(strout, "%8.8x %8.8x %8.8x %8.8x", *(id+0), *(id+1), *(id+2), *(id+3));
	gl_text(0,20,strout,-1);


	// 无licence则失败
	val = !uselog.licence[0] + !uselog.licence[1] + !uselog.licence[2] + !uselog.licence[3];
	val = 1;
	if ( val == 0) {
		return 0;
	}
	// 有licence则首先计算正确的licence
	else {
		calc_licence(licence_true, uselog.date / DATE_PER_MONTH );	
	}
	sprintf(strout, "%8.8u %8.8u",licence_true[3], -1);
	gl_text(0,30,strout,-1);
	val = UseTick(0);
	if (licence_true[3] == uselog.licence[3] && val > 0) {
		sprintf(strout, "ok %d", val);
		gl_text(0,50,strout,-1);
		g_licence_timeout = 0;
		UseTick(1);
		return 1;
	}
	else if (licence_true[3] == uselog.licence[3] && val == 0) {
		gl_text(0,50,"timeout",-1);
		g_licence_timeout = 1;
		return 2;
	}
	else {
		gl_text(0,50,"noooooooooo",-1);
		g_licence_timeout = 1;
		return 0;
	}


	
	sprintf(strout, "%8.8x %8.8x %8.8x %8.8x", 
			uselog.licence[0], 
			uselog.licence[1], 
			uselog.licence[2], 
			uselog.licence[3]);
	gl_text(0,40,strout,-1);

	
	sprintf(strout, "limit %d",i);
	gl_text(0,50,strout,-1);

	return 0;
	if (licence_true[3] == uselog.licence[3]) {
		gl_text(0,50,"okokokook",-1);
		return 1;
	}

	gl_text(0,50,"noooooooooo",-1);
	return 0;
}

