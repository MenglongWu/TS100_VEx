#ifndef _STM32_FLASH_H_
#define _STM32_FLASH_H_


/*
������Flash���ֳ�3������
---------------------------
��Ʒ����   0 ~    Flash - 2 Page
---------------------------
ʹ������   1 Page
---------------------------
��Ʒ����   1 Page   Ϊ�˺���ǰ�ϰ汾����
---------------------------
*/
// ��Ʒ����ȥ
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



