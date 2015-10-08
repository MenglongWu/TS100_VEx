#include "stm32f10x.h" 
#include "lcd\\gl_ui.h"
#include "flash.h"


// 必须4字节对齐，方便Flash读写操作
#define LOG_CACHE (4*4)
#define LIMIT_DATE (24*30)
struct use_log
{
	char key[4];					// 公钥
	unsigned long licence[4];		// 注册码
	unsigned long rand_hw;			// 机器码随机值
	unsigned long licence_times; 	// 注册次数
	unsigned long date;			 	// 使用期限，最大2年
	char log[LOG_CACHE];		 	// 使用次数记录
	unsigned long unuse0;			// 防止数据访问溢出，无效数据
};

// 芯片机器码
struct chiip_id
{
	unsigned long id0;
	unsigned long id1;
	unsigned long id2;
	unsigned long id_rand;   ///<随机值，来源于第一次开机存放于struct use_log的rand_hw
};
// 采用AD值做随机值
extern volatile uint16_t ADCConvertedValue[2000];
unsigned long Rand()
{
	return 	(ADCConvertedValue[1] << 2) + 
			(ADCConvertedValue[3] << 4) + 
			(ADCConvertedValue[5] << 8) + 
			(ADCConvertedValue[7]);
}

// id 长度为
void Get_ChipID(unsigned long *id)
{
	struct use_log uselog;
	char strout[256];
	
	*(id+2) = *(__IO u32*)(0X1FFFF7E8);  // 低字节
	*(id+1) = *(__IO u32 *)(0X1FFFF7EC); // 
	*(id+0) = *(__IO u32 *)(0X1FFFF7F0); // 高字节
	ReadUseLog(&uselog);
	*(id+3) = uselog.rand_hw;
}



int WriteUseLog(struct use_log *puselog)
{

	WriteFlash(FLASH_PAGE_LICENCE,
		(uint32_t*)(puselog),
		sizeof(struct use_log));
}

int ReadUseLog(struct use_log *puselog)
{

	ReadFlash(FLASH_PAGE_LICENCE,
		(uint32_t*)(puselog),
		sizeof(struct use_log));


	if (puselog->rand_hw == (unsigned long)(-1)) {
		puselog->rand_hw = Rand();

		WriteUseLog(puselog);
	}
	if (puselog->date > LIMIT_DATE) {
		puselog->date = LIMIT_DATE;
	}
	return 0;
}



/**
 * @brief	使用计数
 * @retval	剩余使用次数
 * @remarks	
 * @see	
 */

int UseTick()
{
	int i;
	char *plog;
	struct use_log uselog;
	int istimeout = 0;
	char strout[256];

	ReadUseLog(&uselog);

	plog = &uselog.log[0];
	istimeout = 1;
	for (i = 0; i < LOG_CACHE; i++) {
		if ( *plog == *(plog + 1) && *plog != 0) {
			*plog = 0;
			istimeout = 0;
			break;
		}
		plog++;
	}

	WriteUseLog(&uselog);
	return LOG_CACHE - i;
}

// 显示使用计数
void ShowTick()
{

}

// 显示使用期限，ShowTick与此值相等时Licence失效
void ShowTotal()
{

}

int CalcLicence(unsigned long *licence)
{
	unsigned long id[4];

	Get_ChipID(&id[0]);


}


void calc_licence(unsigned long *lic)
{
	unsigned long id[4];
	unsigned long licence[4];
	unsigned long val;

	Get_ChipID(&id[0]);
	val = id[0] ^ id[1] ^ id[3] ^ id[4];
	licence[0] = val ^ id[0];
	val += licence[0];

	licence[1] = val ^ id[1];
	val += licence[1];

	licence[2] = val ^ id[2];
	val += licence[2];

	licence[3] = val ^ id[3];
	val += licence[3];

	val += licence[0] + licence[1] + licence[2] + licence[3];
	*(lic + 0) = licence[0];
	*(lic + 1) = licence[1];
	*(lic + 2) = licence[2];
	*(lic + 3) = licence[3];
}

// return 0 非法
// return 1 正确
int CheckLicence(unsigned long licence[4])
{
	int i;
	char *plog;
	struct use_log uselog;
	unsigned long id[4];
	int istimeout = 0;
	char strout[256];
	unsigned long val, licence_true[4];


	ReadUseLog(&uselog);
	Get_ChipID(&id[0]);
	sprintf(strout, "%8.8x %8.8x %8.8x %8.8x", *(id+0), *(id+1), *(id+2), *(id+3));
	

	// sprintf(stro2ut, "%8.8x ", rand);
	gl_text(0,20,strout,-1);
	

	val = !uselog.licence[0] + !uselog.licence[1] + !uselog.licence[2] + !uselog.licence[3];
	// if ( val == 0) {
	// 	return 0;
	// }
	calc_licence(licence_true);

	sprintf(strout, "%8.8u %8.8x",licence_true[3], FLASH_PAGE_LICENCE);
	gl_text(0,30,strout,-1);
	
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

