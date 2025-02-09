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

#include <stdint.h>
#include <FlashAlgorithm.h>
#include "nrf52_bitfields.h"

/* ================================================================================ */
/* ================                      NVMC                      ================ */
/* ================================================================================ */


/**
  * @brief Non Volatile Memory Controller (NVMC)
  */

typedef struct {                                    /*!< NVMC Structure                                                        */
  volatile const  uint32_t  RESERVED0[256];
  volatile const  uint32_t  READY;                  /*!< Ready flag                                                            */
  volatile const  uint32_t  RESERVED1[64];
  volatile uint32_t         CONFIG;                 /*!< Configuration register                                                */

  union {
	  volatile uint32_t     ERASEPCR1;              /*!< Deprecated register - Register for erasing a page in Code area.
                                                         Equivalent to ERASEPAGE.                                              */
    volatile uint32_t       ERASEPAGE;              /*!< Register for erasing a page in Code area                              */
  };
  volatile uint32_t         ERASEALL;               /*!< Register for erasing all non-volatile user memory                     */
  volatile uint32_t         ERASEPCR0;              /*!< Deprecated register - Register for erasing a page in Code area.
                                                         Equivalent to ERASEPAGE.                                              */
  volatile uint32_t         ERASEUICR;              /*!< Register for erasing User Information Configuration Registers         */
  volatile const  uint32_t  RESERVED2[10];
  volatile uint32_t         ICACHECNF;              /*!< I-Code cache configuration register.                                  */
  volatile const  uint32_t  RESERVED3;
  volatile uint32_t         IHIT;                   /*!< I-Code cache hit counter.                                             */
  volatile uint32_t         IMISS;                  /*!< I-Code cache miss counter.                                            */
} NRF_NVMC_Type;

#define NRF_NVMC            ((NRF_NVMC_Type           *) 0x4001E000UL)

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
	// Включаем режим чтения памяти
	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
	while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
	{
	}

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
	// Включаем режим чтения памяти
	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
	while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
	{
	}

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
	// Enable erase.
	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een;
	while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
	{
	}

	// Erase the page
	NRF_NVMC->ERASEALL = NVMC_ERASEALL_ERASEALL_Erase;
	while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
	{
	}

	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
	while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
	{
	}

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
int FlashEraseSector(unsigned long sectorAddr)
{
	sectorAddr = (sectorAddr + 3) & ~0x3UL;

    // Enable erase.
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een;
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
    }

    // Erase the page
    NRF_NVMC->ERASEPAGE = sectorAddr;
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
    }

    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
    }

	return 0;
}

void nrf_nvmc_write_byte(uint32_t address, uint8_t value)
{
	uint32_t byte_shift = address & (uint32_t) 0x03;
	uint32_t address32 = address & ~byte_shift; // Address to the word this byte is in.
	uint32_t value32 = (*(uint32_t*) address32 & ~((uint32_t) 0xFF << (byte_shift << (uint32_t) 3)));

	value32 = value32 + ((uint32_t) value << (byte_shift << 3));

	// Enable write.
	NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos);
	while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
	{
	}

	*(uint32_t*) address32 = value32;
	while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
	{
	}

	NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos);
	{
	}
}

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

	for (i = 0; i < size; i++)
	{
		nrf_nvmc_write_byte(pageAddr + i, buf[i]);
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

