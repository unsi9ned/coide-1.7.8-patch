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

#ifdef LOG_ENABLE
#include "uart.h"
#endif

#ifdef __cplusplus
  #define   __I     volatile             /*!< Defines 'read only' permissions */
#else
  #define   __I     volatile const       /*!< Defines 'read only' permissions */
#endif
#define     __O     volatile             /*!< Defines 'write only' permissions */
#define     __IO    volatile             /*!< Defines 'read / write' permissions */

/* following defines should be used for structure members */
#define     __IM     volatile const      /*! Defines 'read only' structure member permissions */
#define     __OM     volatile            /*! Defines 'write only' structure member permissions */
#define     __IOM    volatile            /*! Defines 'read / write' structure member permissions */

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

/* ================================================================================ */
/* ================                      FICR                      ================ */
/* ================================================================================ */

typedef struct {
  __I  uint32_t  PART;                              /*!< Part code                                                             */
  __I  uint32_t  VARIANT;                           /*!< Part Variant, Hardware version and Production configuration           */
  __I  uint32_t  PACKAGE;                           /*!< Package option                                                        */
  __I  uint32_t  RAM;                               /*!< RAM variant                                                           */
  __I  uint32_t  FLASH;                             /*!< Flash variant                                                         */
  __IO uint32_t  UNUSED0[3];                        /*!< Description collection[0]: Unspecified                                */
} FICR_INFO_Type;


/**
  * @brief Factory Information Configuration Registers (FICR)
  */

typedef struct {                                    /*!< FICR Structure                                                        */
  __I  uint32_t  RESERVED0[4];
  __I  uint32_t  CODEPAGESIZE;                      /*!< Code memory page size                                                 */
  __I  uint32_t  CODESIZE;                          /*!< Code memory size                                                      */
  __I  uint32_t  RESERVED1[18];
  __I  uint32_t  DEVICEID[2];                       /*!< Description collection[0]: Device identifier                          */
  __I  uint32_t  RESERVED2[6];
  __I  uint32_t  ER[4];                             /*!< Description collection[0]: Encryption Root, word 0                    */
  __I  uint32_t  IR[4];                             /*!< Description collection[0]: Identity Root, word 0                      */
  __I  uint32_t  DEVICEADDRTYPE;                    /*!< Device address type                                                   */
  __I  uint32_t  DEVICEADDR[2];                     /*!< Description collection[0]: Device address 0                           */
  __I  uint32_t  RESERVED3[21];

  FICR_INFO_Type INFO;                              /*!< Device info                                                           */
  __I  uint32_t  RESERVED4[185];
#if 0
  FICR_TEMP_Type TEMP;                              /*!< Registers storing factory TEMP module linearization coefficients      */
  __I  uint32_t  RESERVED5[2];
  FICR_NFC_Type NFC;                                /*!< Unspecified                                                           */
#endif
} NRF_FICR_Type;

#define NRF_NVMC            ((NRF_NVMC_Type           *) 0x4001E000UL)
#define NRF_FICR            ((NRF_FICR_Type           *) 0x10000000UL)

#ifndef PROTECTED_REGION
#define PROTECTED_REGION    (0x44000)
#endif

#ifdef LOG_ENABLE
static int busy;
#endif

extern struct FlashAlgorithm const FlashDriver __attribute__ ((section(".driver_info")));

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

#if defined(LOG_ENABLE)
	uartInit();
	busy = 0;
	
#if 0
	uartPrintIntParameter("Flash total size", NRF_FICR->INFO.FLASH << 10, 0);
	uartPrintIntParameter("Page size", NRF_FICR->CODEPAGESIZE, 0);
	uartSendStr("--------------------------------\n");
#endif
#endif
	
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

#ifdef LOG_ENABLE
	uartUninit();
	busy = 0;
#endif
	
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

#if 0
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
#else
	for(unsigned long sectorAddr = PROTECTED_REGION; 
                      sectorAddr < (NRF_FICR->INFO.FLASH << 10);
					  sectorAddr += NRF_FICR->CODEPAGESIZE)
	{
		FlashEraseSector(sectorAddr);
	}
#endif

#ifdef LOG_ENABLE
	uartSendStr("Chip ERASE  ");
	uartSendStr("[DONE]\n");
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
int FlashEraseSector(unsigned long sectorAddr)
{
	sectorAddr = (sectorAddr + 3) & ~0x3UL;

#ifdef LOG_ENABLE
	uartSendStr("Sector ERASE, address = ");
	uartPrintHex(sectorAddr, 8);
	uartSendStr("  ");
		
	if(busy)
	{
		uartSendStr("[BUSY]\n");
		return 1;
	}
	busy = 1;
#endif

	if(sectorAddr >= PROTECTED_REGION)
	{
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

#ifdef LOG_ENABLE
		uartSendStr("[DONE]\n");
		busy = 0;
#endif
	}
	else
	{
#ifdef LOG_ENABLE
		uartSendStr("[PROTECTED]\n");
		busy = 0;
#endif
	}

	return 0;
}

void nrf_nvmc_write_byte(uint32_t address, uint8_t value)
{
	uint32_t byte_shift = address & (uint32_t) 0x03;
	uint32_t address32 = address & ~byte_shift; // Address to the word this byte is in.
	uint32_t value32 = (*(uint32_t*) address32 & ~((uint32_t) 0xFF << (byte_shift << (uint32_t) 3)));

	value32 = value32 + ((uint32_t) value << (byte_shift << 3));

#if 0
	uartSendStr("\tWrite ");
	uartPrintHex(value32, 8);
	uartSendStr(" to ");
	uartPrintHex(address, 8);
	uartSendStr("\n");
#endif

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
	while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
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
#ifdef LOG_ENABLE
	uartSendStr("Page PROGRAM, addr = ");
	uartPrintHex(pageAddr, 8);
	uartSendStr(" , size = ");
	uartPrintLongint32(size, 0);
	uartSendStr("  ");
	
	if(busy)
	{
		uartSendStr("[BUSY]\n");
		return 1;
	}
	busy = 1;
#endif

	if(pageAddr >= PROTECTED_REGION)
	{
#if 0
		for (unsigned long i = 0; i < size; i++)
		{
			nrf_nvmc_write_byte(pageAddr + i, buf[i]);
		}
#else
		// CoFlash на самом деле не контролирует статус завершения
		// операции записи в память. Он вызывает функцию FlashProgramPage
		// по таймеру. Потому пишем в память максимально быстро, чтобы
		// успевать до следующего вызова функции
		unsigned long * flash = (unsigned long*)pageAddr;
		unsigned long * ram = (unsigned long*)buf;
		size >>= 2;
		
		while(size)
		{
			// Enable write.
			NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos);
			while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
			
			*flash++ = *ram++;
			
			while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
			
			NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos);
			while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

			--size;
		}
#endif
	}

#ifdef LOG_ENABLE
	uartSendStr(pageAddr >= PROTECTED_REGION ? "[DONE]\n" : "[PROTECTED]\n");
	busy = 0;
#endif

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
	unsigned long i;
	unsigned char* pageBuf = (unsigned char*)verifyAddr;
	
#ifdef LOG_ENABLE
	uartSendStr("Data VERIFY, addr = ");
	uartPrintHex(verifyAddr, 8);
	uartSendStr(" , size = ");
	uartPrintLongint32(size, 0);
	uartSendStr("  ");
#endif

	for(i = 0; i < size; i++)
	{
		if(((unsigned long)&pageBuf[i]) < PROTECTED_REGION)
		{
			continue;
		}
		
		if(pageBuf[i] != buf[i])
		{
#ifdef LOG_ENABLE
			uartSendStr("[FAILED]\n");
#endif
			return (1);
		}
	}
	
#ifdef LOG_ENABLE
	uartSendStr(verifyAddr >= PROTECTED_REGION ? "[OK]\n" : "[PROTECTED]\n");
#endif

	return (0);
}

