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

/////////////////////////////////////////////////////////////////////////////

#if defined ( SAM3U_B1_128 )
#if defined ( __GNUC__   )
__attribute__ ((section(".driver_info")))
#endif
struct FlashAlgorithm const FlashDriver = {
	((FLASH_FRAMEWORK_VERSION << 16) | FLASH_ALGORITHM_VERSION),
	"ATSAM3U Bank1 128KB Flash",
	{
		FLASH_TYPE_ONCHIP,
		0x00100000,                 // Device Start Address
		0x00020000,                 // Device Size in Bytes (256kB)
		256,                        // Programming Page Size
		0xFF,                       // Initial Content of Erased Memory
		{
			{ 0x0100, 0x00000000 },
			{ FLASH_SECTOR_END   },
		},
	},
	5000,												// Time Out of FlashInit/FlashUnInit/FlashVerify/FlashBlankCheck function (in msec)
	5000,    											// Time Out of FlashProgramage  Function (in msec)
	5000,  												// Time Out of FlashEraseSector Function (in msec)
	5000, 												// Time Out of FlashEraseChip   Function (in msec)
};
#endif

/////////////////////////////////////////////////////////////////////////////

#if defined ( SAM3U_B0_128 )
#if defined ( __GNUC__   )
__attribute__ ((section(".driver_info")))
#endif
struct FlashAlgorithm const FlashDriver = {
	((FLASH_FRAMEWORK_VERSION << 16) | FLASH_ALGORITHM_VERSION),
	"ATSAM3U Bank0 128KB Flash",
	{
		FLASH_TYPE_ONCHIP,
		0x00080000,                 // Device Start Address
		0x00020000,                 // Device Size in Bytes (128kB)
		256,                        // Programming Page Size
		0xFF,                       // Initial Content of Erased Memory
		{
			{ 0x0100, 0x00000000 },
			{ FLASH_SECTOR_END   },
		},
	},
	5000,												// Time Out of FlashInit/FlashUnInit/FlashVerify/FlashBlankCheck function (in msec)
	5000,    											// Time Out of FlashProgramage  Function (in msec)
	5000,  												// Time Out of FlashEraseSector Function (in msec)
	5000, 												// Time Out of FlashEraseChip   Function (in msec)
};
#endif

/////////////////////////////////////////////////////////////////////////////

#if defined ( SAM3U_B0_64 )
#if defined ( __GNUC__   )
__attribute__ ((section(".driver_info")))
#endif
struct FlashAlgorithm const FlashDriver = {
	((FLASH_FRAMEWORK_VERSION << 16) | FLASH_ALGORITHM_VERSION),
	"ATSAM3U Bank0 64KB Flash",
	{
		FLASH_TYPE_ONCHIP,
		0x00080000,                 // Device Start Address
		0x00010000,                 // Device Size in Bytes (64kB)
		256,                        // Programming Page Size
		0xFF,                       // Initial Content of Erased Memory
		{
			{ 0x0100, 0x00000000 },
			{ FLASH_SECTOR_END   },
		},
	},
	5000,												// Time Out of FlashInit/FlashUnInit/FlashVerify/FlashBlankCheck function (in msec)
	5000,    											// Time Out of FlashProgramage  Function (in msec)
	5000,  												// Time Out of FlashEraseSector Function (in msec)
	5000, 												// Time Out of FlashEraseChip   Function (in msec)

};
#endif

/////////////////////////////////////////////////////////////////////////////
#if defined ( __GNUC__   )
__attribute__ ((section(".driver_info")))
#endif
struct FlashAlgorithm const FlashDriver = {
	((FLASH_FRAMEWORK_VERSION << 16) | FLASH_ALGORITHM_VERSION),
	"ATSAM21G18 Bank0 256KB Flash",
	{
		FLASH_TYPE_ONCHIP,
		0x00000000,                        // Device Start Address
		0x00040000,                        // Device Size in Bytes (256kB)
		64,                                // Programming Page Size TODO уточнить
		0xFF,                              // Initial Content of Erased Memory
		{
			{ 0x0100, 0x00000000 },
			{ FLASH_SECTOR_END   },
		},
	},
	5000,												// Time Out of FlashInit/FlashUnInit/FlashVerify/FlashBlankCheck function (in msec)
	5000,    											// Time Out of FlashProgramage  Function (in msec)
	5000,  												// Time Out of FlashEraseSector Function (in msec)
	5000, 												// Time Out of FlashEraseChip   Function (in msec)

};

