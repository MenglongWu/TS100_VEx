#include "stm32f10x_flash.h"
#include "flash.h"

#include "stm32f10x.h" 
#include "lcd\\gl_ui.h"
#include "flash.h"

/*
#if defined (STM32F10X_HD) || defined (STM32F10X_HD_VL) || (STM32F10X_CL) || defined (STM32F10X_XL)
	#define FLASH_PAGE_SIZE    ((uint16_t)0x800)
	#define FLASH_PAGE_PRODUCT ((u32)0x08002000)
#else
	#define FLASH_PAGE_SIZE    ((uint16_t)0x400)
	#define FLASH_PAGE_PRODUCT ((u32)0x08002000)
#endif
*/

/**
 * @brief	以4Byte宽度向Flash写入数据
 * @param	addr Flash 页面地址
 * @param	data 数据内容
 * @param	len 数据长度，
 * @retval	null
 * @remarks	内部调用FLASH_ProgramWord，支持STM32F10x设备，要求\n
 			addr = PageAddr + 4*N ，页面地址偏移量以页面地址 + 4倍单位\n
 			len  = 4 *N ，数据长度为4的倍数\n
 * @see	WriteFlash0
 */

uint32_t WriteFlash(uint32_t addr,uint32_t *data,uint32_t len)
{
	uint32_t numPage,EraseCounter;
	volatile FLASH_Status FLASHStatus;
	uint32_t startAddr,endAddr;
	
	FLASH_Unlock();								//Unlock the Flash Program Erase controller
	FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);  // Clear All pending flags	
	
	FLASHStatus = FLASH_COMPLETE;
	numPage = (len / FLASH_PAGE_SIZE) + (len % FLASH_PAGE_SIZE) + 1;
	numPage = 1;

	for(EraseCounter = 0; (EraseCounter < numPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
	{
		FLASHStatus = FLASH_ErasePage(addr + (FLASH_PAGE_SIZE * EraseCounter));
	}
	
	startAddr = addr;
	endAddr = startAddr + len;
	FLASHStatus = FLASH_COMPLETE;
	while((startAddr < endAddr) )//&& (FLASHStatus == FLASH_COMPLETE))
	{
		FLASHStatus = FLASH_ProgramWord(startAddr, *data++);
		startAddr = startAddr + 4;
	}
	FLASH_Lock();
}


uint32_t ReadFlash(uint32_t addr,uint32_t *data,uint32_t len)
{
	uint32_t numPage,EraseCounter;
	volatile FLASH_Status FLASHStatus;
	uint32_t startAddr,endAddr;
	uint32_t *p;
	
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);  // Clear All pending flags	
	FLASHStatus = FLASH_COMPLETE;
	startAddr = addr;
	endAddr = startAddr + len;
	
	while(startAddr < endAddr) {
		p = (uint32_t*)startAddr;
		*data++ = *p;
		startAddr += 4;
	}
	FLASH_Lock();
}

/**
 * @brief	以4Byte宽度向Flash写入数据，数据写入前不对原始Flash内容清除，Flash只能
 			从1写成0，
 			原始Flash数据    	\n
 			ff 3f 0f ff 		\n
 			写入				\n
 			55 55 55 55 		\n
 			写入后Flash数据		\n
 			ff 1f 05 05			\n
 * @param	addr Flash 页面地址
 * @param	data 数据内容
 * @param	len 数据长度，
 * @retval	null
 * @remarks	内部调用FLASH_ProgramWord，支持STM32F10x设备，要求\n
 			addr = PageAddr + 4*N ，页面地址偏移量以页面地址 + 4倍单位\n
 			len  = 4 *N ，数据长度为4的倍数\n
 * @see	WriteFlash
 */
uint32_t WriteFlash0(uint32_t addr, uint32_t off,uint32_t *data,uint32_t len)
{
	uint32_t numPage,EraseCounter;
	volatile FLASH_Status FLASHStatus;
	uint32_t startAddr,endAddr;
	uint32_t val;
	
	FLASH_Unlock();								//Unlock the Flash Program Erase controller
	FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);  // Clear All pending flags	
	
	FLASHStatus = FLASH_COMPLETE;
	numPage = (len / FLASH_PAGE_SIZE) + (len % FLASH_PAGE_SIZE) + 1;
	numPage = 1;

	// for(EraseCounter = 0; (EraseCounter < numPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
	// {
	// 	FLASHStatus = FLASH_ErasePage(addr + (FLASH_PAGE_SIZE * EraseCounter));
	// }
	
	startAddr = addr + off;
	endAddr = startAddr + len;
	FLASHStatus = FLASH_COMPLETE;
	while((startAddr < endAddr) )//&& (FLASHStatus == FLASH_COMPLETE))
	{
		val = *data;
		FLASHStatus = FLASH_ProgramWord(startAddr, val);
		startAddr = startAddr + 4;
		data += 1;
	}
	FLASH_Lock();
}

/**
 * @brief	以保留方式覆盖原始Flash数据，写入新数据\n
 向地址addr2 写入data[2] = {0x31,0x44}
 写入前page \n
 ---------------------------  		\n
 addr   00 12 15 33 ....			\n
 addr2  ff ff ff ff ....			\n
 ---------------------------		\n
 写入后page \n
 ---------------------------  		\n
 addr   00 12 15 33 ....			\n
 addr2  31 44 ff ff ....			\n
 ---------------------------		\n

 * @param	addr  Flash页面地址，
 * @param	addr2 Flash页面地址，
 * @retval	null
 * @remarks	
 * @see	
 */

uint8_t pagetmp[FLASH_PAGE_SIZE];
uint32_t CoverFlash(uint32_t addr,uint32_t addr2,uint32_t *data,uint32_t len)
{
	uint32_t start,i;
	uint32_t *pstart32;
	uint8_t *pstart8;
	
	start = addr2 - addr;
	pstart32 = (uint32_t*)start;
	pstart8 = (uint8_t*)data;
	ReadFlash(addr,(uint32_t*)pagetmp,FLASH_PAGE_SIZE);
	for(i = 0;i < len;++i) {
		pagetmp[start+i] = *(pstart8+i);
	}
	WriteFlash(addr,(uint32_t*)pagetmp,FLASH_PAGE_SIZE);
	
}

// flash ²úÆ··À¿½±´£¬
// ×¢Òâ£¡£¡£¡£¡£¡£¡£¡£¡£¡£¡£¡£¡£¡£¡£¡£¡£¡
// µ÷ÓÃºóFLASH½«²»ÄÜÔÙ´ÎÉÕÂ¼£¬±ØÐëÓÃ UnProtectFlash½âËø

// ÏÂÃæ½éÉÜÁ½ÖÖÇé¿ö£¬Ö»ÓÐ´æÔÚJ-Link ÉÕÐ´Ê§°ÜµÄÇé¿öÏÂ²Åµ¼ÖÂFlash½âËøºóÈ«²¿±»×Ô¶¯²Á³ý
// 1. ProtectFlash   --- J-LinkÉÕÂ¼  --- ÉÕÂ¼Ê§°Ü --- UnProtectFlash --- Flash ±»²Á³ý
// 2. ProtectFlash   --- UnProtectFlash --- Flash ¿ÉÒÔ¶ÁÐ´²¢Î´±»²Á³ý£¬¿ÉÒÔÖØ¸´µ÷ÓÃÉÏÃæÁ½¸öº¯Êý

void ProtectFlash()
{
#ifdef _USE_PROTECT_FLASH_
	FLASH_Unlock();
	FLASH_ReadOutProtection(ENABLE);
	FLASH_Lock();
#endif
}


void UnProtectFlash()
{
	FLASH_Unlock();
	FLASH_ReadOutProtection(DISABLE);
	FLASH_Lock();
}