#include "stm32f10x.h" 
#include "lcd\\gl_ui.h"
#include "flash.h"
// id 长度为
void Get_ChipID(unsigned long *id)
{
	char strout[256];
	
	*(id+2) = *(__IO u32*)(0X1FFFF7E8);  // 低字节
	*(id+1) = *(__IO u32 *)(0X1FFFF7EC); // 
	*(id+0) = *(__IO u32 *)(0X1FFFF7F0); // 高字节
	
	sprintf(strout, "%8.8x %8.8x %8.8x", *(id+0), *(id+1), *(id+2));
	
	gl_text(0,10,strout,-1);

}

// 必须4字节对齐，方便Flash读写操作
#define LOG_CACHE (3*4)
#define LIMIT_DATE (24*30)
struct use_log
{
	char key[4];		// 公钥
	unsigned long licence[4];	// 注册码
	unsigned long licence_times; // 注册次数
	unsigned long date;			// 使用期限，最大2年
	char log[LOG_CACHE];// 使用次数记录
	unsigned long unuse0;		// 防止数据访问溢出，无效数据
};

int ReadUseLog(struct use_log *puselog)
{

	ReadFlash(FLASH_PAGE_LICENCE,
		(uint32_t*)(puselog),
		sizeof(struct use_log));

	if (puselog->date > LIMIT_DATE) {
		puselog->date = LIMIT_DATE;
	}
	return 0;
}

int WriteUseLog(struct use_log *puselog)
{

	WriteFlash(FLASH_PAGE_LICENCE,
		(uint32_t*)(puselog),
		sizeof(struct use_log));
}
// 使用计数
int UseTick()
{
	int i;
	char *plog;
	struct use_log uselog;
	int istimeout = 0;
	char strout[256];

	ReadUseLog(&uselog);
	// ReadFlash(FLASH_PAGE_LICENCE,
	// 	(uint32_t*)&(uselog),
	// 	sizeof(struct use_log));

	// plog = &uselog.log[0];
	// for (i = 0; i < 10; i++) {
	// 	sprintf(strout, "%2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x %2.2x", 
	// 		*(plog+0), *(plog+1), *(plog+2),*(plog+3), 
	// 		*(plog+4), *(plog+5),*(plog+6), *(plog+7),
	// 		*(plog+8), *(plog+9),*(plog+10), *(plog+11),
	// 		*(plog+12), *(plog+13),*(plog+14), *(plog+15));
	// 	gl_text(0,10 + i * 10,strout,-1);
	// 	plog += 16;
	// }



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
	unsigned long val;

	ReadUseLog(&uselog);
	Get_ChipID(&id[0]);
	sprintf(strout, "%8.8x %8.8x %8.8x", *(id+0), *(id+1), *(id+2));

	sprintf(strout, "%d", *(id+0));
	gl_text(0,20,strout,-1);
	val = !uselog.licence[0] + !uselog.licence[1] + !uselog.licence[2] ;
	if ( val == 0) {
		return 0;
	}
	uselog.licence[0] += uselog.date;
	uselog.licence[1] += uselog.date;
	uselog.licence[2] += uselog.date;
	uselog.licence[3] += uselog.date;
	
	if (uselog.licence[0] == *(id+0)) {
		return 1;
	}
	
	return 0;
}