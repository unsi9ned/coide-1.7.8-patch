/**
 *******************************************************************************
 * @file       FlashAlgoFncs.c
 * @version    V0.1
 * @date       2010.12.02
 * @brief      Flash  Algorithm For SAM3U with 128KB Flash Rom	
 *******************************************************************************
 * @copy
 *
 * INTERNAL FILE,DON'T PUBLIC.
 * 
 * <h2><center>&copy; COPYRIGHT 2009 CooCox </center></h2>
 *******************************************************************************
 */

#include <FlashAlgorithm.h>

#include "samd21g18a.h"

/** Frequency of the board main oscillator */
#define VARIANT_MAINOSC                   (25000ul)

/** Master clock frequency */
#define VARIANT_MCK                       (48000000UL)

#define FLASH_PAGE_SIZE_BYTE               64
#define FLASH_ROW_SIZE                     4

static const unsigned long pageSizes[] = { 8, 16, 32, 64, 128, 256, 512, 1024 };

// Base Address
unsigned long base_adr;
unsigned long pageSize;
unsigned long pageNum;
int flashErased = 0;

extern struct FlashAlgorithm const FlashDriver;

/**
 *******************************************************************************
 * @brief      Initialize before Flash Programming/Erase Functions 
 * @param[in]  baseAddr     Flash device base address.
 * @param[in]  clk     			Flash program clock.
 * @param[in]  operateFuc   Init for what operation
 (FLASH_OPT_ERASECHIP/FLASH_OPT_ERASESECTORS/FLASH_OPT_PROGRAMPAGE).
 * @param[out] None  
 * @retval     0   					All is OK.
 * @retval     others       Some error occurs.		 
 *
 * @par Description
 * @details    This function is called before flash programming/erase. 
 * @note 
 *******************************************************************************
 */
int FlashInit(unsigned long baseAddr,
              unsigned long clk,
              unsigned long operateFuc)
{
	SYSCTRL->OSC8M.bit.PRESC = SYSCTRL_OSC8M_PRESC_0_Val ;  //CMSIS 4.5 changed the prescaler defines
	SYSCTRL->OSC8M.bit.ONDEMAND = 0 ;

	NVMCTRL->CTRLB.bit.MANW = 1;
	
	base_adr = baseAddr;
	pageSize = pageSizes[NVMCTRL->PARAM.bit.PSZ];
	pageNum = NVMCTRL->PARAM.bit.NVMP;
	flashErased = 0;

	return (0);
}

/**
 *******************************************************************************
 * @brief      Un-Init after Flash Programming/Erase Functions  
 * @param[in]  operateFuc   Init for what operation
 (FLASH_OPT_ERASECHIP/FLASH_OPT_ERASESECTORS/FLASH_OPT_PROGRAMPAGE).
 * @param[out] None  
 * @retval     0   					All is OK.
 * @retval     others       Some error occurs.		 
 *
 * @par Description
 * @details    This function is called after flash programming/erase. 
 * @note 
 *******************************************************************************
 */
int FlashUnInit(unsigned long operateFuc)
{
	flashErased = 0;
	return (0);
}

/**
 *******************************************************************************
 * @brief      Erase the full chip.  
 * @param[in]  None.
 * @param[out] None  
 * @retval     0   					All is OK.
 * @retval     others       Some error occurs.		 
 *
 * @par Description
 * @details     
 * @note 
 *******************************************************************************
 */
int FlashEraseChip(void)
{
#if 0
	unsigned long i, addr;
	unsigned long sectorSize = FlashDriver.deviceInfo.sectors[0].size;
	unsigned long sectorsNum = pageNum * pageSize/ sectorSize;

	for(i = 0, addr = 0; i < sectorsNum; i++, addr += sectorSize)
		FlashEraseSector(addr);
#else
	unsigned long i, addr;
	unsigned long sectorsNum = pageNum / FLASH_ROW_SIZE;
	unsigned long sectorSize = pageSize * FLASH_ROW_SIZE;

	for(i = 0, addr = 0; i < sectorsNum; i++, addr += sectorSize)
	{
		NVMCTRL->ADDR.reg = addr / 2;
		NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_ER;
		while (!NVMCTRL->INTFLAG.bit.READY)
		{
		}
	}
#endif

	return (0);
}

/**
 *******************************************************************************
 * @brief      Erase the select Sector. 
 * @param[in]  sectorAddr   Sector's start address.
 * @param[out] None  
 * @retval     0   					All is OK.
 * @retval     others       Some error occurs.		 
 *
 * @par Description
 * @details     
 * @note 
 *******************************************************************************
 */
#if 0
int FlashEraseSector(unsigned long sectorAddr)
{
	unsigned long i;
	unsigned long realSectorSize = pageSize * FLASH_ROW_SIZE;
	unsigned long realSectorsNum = FlashDriver.deviceInfo.sectors[0].size / realSectorSize;

	for(i = 0; i < realSectorsNum; i++, sectorAddr += realSectorSize)
	{
		NVMCTRL->ADDR.reg = sectorAddr / 2;
		NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_ER;
		while (!NVMCTRL->INTFLAG.bit.READY)
		{
		}
	}
	return (0);
}
#else
// Из-за некорректной работы программы coflash стираем всю память каждый раз
int FlashEraseSector(unsigned long sectorAddr)
{
	if(!flashErased)
	{
		FlashEraseChip();
		flashErased = 1;
	}
	return 0;
}
#endif

/**
 *******************************************************************************
 * @brief      Proram a page. 
 * @param[in]  pageAddr   Page's start address.
 * @param[in]  size			  Page size
 * @param[in]  buf   			source point.
 * @param[out] None  
 * @retval     0   					All is OK.
 * @retval     others       Some error occurs.		 
 *
 * @par Description
 * @details     
 * @note 
 *******************************************************************************
 */
int FlashProgramPage(unsigned long pageAddr,
                     unsigned long size,
                     unsigned char *buf)
{
	unsigned long i;
	unsigned long page;
	unsigned long * Flash;

	Flash = (unsigned long *) pageAddr;

	// Do writes in pages
	while (size)
	{
		// Execute "PBC" Page Buffer Clear
		NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_PBC;
		while (NVMCTRL->INTFLAG.bit.READY == 0)
		{
		}

		// Fill page buffer
		for (i = 0, size = (size + 3) & ~3;
			 i < FLASH_PAGE_SIZE_BYTE && size;
			 i += 4, buf += 4)
		{
			*Flash++ = *((unsigned long *)buf);

			if(size >= 4)
				size -= 4;
			else
				size = 0;
		}

		// Execute "WP" Write Page
		NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_WP;
		while (NVMCTRL->INTFLAG.bit.READY == 0)
		{
		}
	}
	return (0);
}

/**
 *******************************************************************************
 * @brief      Page Verify Function. 
 * @param[in]  verifyAddr   Verify Start Address(Usually page start address).
 * @param[in]  size			Verify size
 * @param[in]  buf   		Source buf point.
 * @param[out] None  
 * @retval     0   			Verify pass.
 * @retval     others       Some error occurs or verify failed..		 
 *
 * @par Description
 * @details   Optional function. When this function is absence, 
 *            the link will read flash memory directly to do verify.  
 * @note 
 *******************************************************************************
 */
int FlashVerify(unsigned long verifyAddr,
                unsigned long size,
                unsigned char *buf)
{
	// TODO: your code for the page verify
	unsigned long i;
	unsigned char* pageBuf = (unsigned char*)verifyAddr;

	for(i = 0; i < size; i++)
	{
		if(pageBuf[i] != buf[i])
		{
			return (1);
		}
	}

	return (0);
}

