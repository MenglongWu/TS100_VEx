#ifndef _STM32_CHIP_H_
#define _STM32_CHIP_H_

extern int g_licence_timeout;				///< 许可证是否超时



void lc_GetChipID(unsigned long *id);
void lc_GetChipMonth(unsigned long *month);
void lc_GetChipleave(unsigned long *leave);
void lc_CheckMonth(unsigned long *month);
int lc_InputLicence(unsigned long *licence, unsigned long *month);
int lc_CheckLicence(unsigned long licence[4]);
#endif