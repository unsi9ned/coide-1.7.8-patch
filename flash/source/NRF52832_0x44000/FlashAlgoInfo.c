/**
 *******************************************************************************
 * @file       FlashAlgoInfo.c
 * @version    V0.01    Initial version
 * @date       2010.12.02
 * @brief      Flash  Algorithm and Flash Device Description.	
 *******************************************************************************
 * @copy
 *
 * INTERNAL FILE,DON'T PUBLIC.
 * 
 * <h2><center>&copy; COPYRIGHT 2009 CooCox </center></h2>
 *******************************************************************************
 */
#include <FlashAlgorithm.h>

#define FLASH_ALGORITHM_VERSION 0x0030

#ifndef PROTECTED_REGION
#define PROTECTED_REGION    (0x44000)
#endif

#define caption(text) #text
#define region(addr)  #addr
#define make_description(cap, addr) caption(cap)region(addr)

#if defined ( __GNUC__   )
__attribute__ ((section(".driver_info")))
#endif
struct FlashAlgorithm const FlashDriver = {
	((FLASH_FRAMEWORK_VERSION << 16) | FLASH_ALGORITHM_VERSION),
	make_description(nRF52832 protected 0x0-, PROTECTED_REGION),
	{
		FLASH_TYPE_ONCHIP,
		0x00000000,                        // Device Start Address
		0x00080000,                        // Device Size in Bytes (512kB)
		4096,                              // Programming Page Size
		0xFF,                              // Initial Content of Erased Memory
		{
			{ 4096, 0x00000000 },
			{ FLASH_SECTOR_END   },
		},
	},
	5000,												// Time Out of FlashInit/FlashUnInit/FlashVerify/FlashBlankCheck function (in msec)
	5000,    											// Time Out of FlashProgramage  Function (in msec)
	5000,  												// Time Out of FlashEraseSector Function (in msec)
	5000, 												// Time Out of FlashEraseChip   Function (in msec)

};

