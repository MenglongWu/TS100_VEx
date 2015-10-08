#ifndef _STM32_CHIP_H_
#define _STM32_CHIP_H_

extern int g_licence_timeout;				///< 许可证是否超时



unsigned long Rand();
void Get_ChipID(unsigned long *id);
int WriteProLicence(struct pro_licence *puselog);
int WriteTick(struct pro_licence *puselog);
int ReadProLicence(struct pro_licence *puselog);
int UseTick(int bwrite);
int ShowTick();
int ShowTotal();
int CheckLicence(unsigned long licence[4]);
#endif