#ifndef _STM32_FLASH_H_
#define _STM32_FLASH_H_


/*
将整个Flash划分成3个分区
---------------------------
产品代码   0 ~    Flash - 2 Page
---------------------------
使用期限   1 Page
---------------------------
产品配置   1 Page   为了和以前老版本兼容
---------------------------
*/
// 产品配置去
#if defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || (STM32F10X_CL) || defined (STM32F10X_XL)

#define FLASH_PAGE_SIZE    ((uint16_t)0x800)
//#define FLASH_PAGE_PRODUCT ((u32)0x08020000)
#define FLASH_PAGE_PRODUCT ((u32)0x0803F800)
#define FLASH_PAGE_LICENCE ((u32) (FLASH_PAGE_PRODUCT - FLASH_PAGE_SIZE))
#else
#define FLASH_PAGE_SIZE    ((uint16_t)0x400)
//#define FLASH_PAGE_PRODUCT ((u32)0x08020000)
#define FLASH_PAGE_PRODUCT ((u32)0x0803F800)
#endif


#define FLASH_PAGE_LICENCE ((u32) (FLASH_PAGE_PRODUCT - FLASH_PAGE_SIZE))

uint32_t WriteFlash(uint32_t addr,uint32_t *data,uint32_t len);
uint32_t ReadFlash(uint32_t addr,uint32_t *data,uint32_t len);
// uint32_t WriteFlash0(uint32_t addr, uint32_t off,uint32_t *data,uint32_t len);
uint32_t CoverFlash(uint32_t addr,uint32_t addr2,uint32_t *data,uint32_t len);
void UnProtectFlash();
void ProtectFlash();
uint32_t WriteFlash0(uint32_t addr, uint32_t off,uint32_t *data,uint32_t len);
#endif



