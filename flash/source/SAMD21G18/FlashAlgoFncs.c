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
#if 0
	/* Set 1 Flash Wait State for 48MHz, cf tables 20.9 and 35.27 in SAMD21 Datasheet */
	NVMCTRL->CTRLB.bit.RWS = NVMCTRL_CTRLB_RWS_HALF_Val;

	/* Turn on the digital interface clock */
	PM->APBAMASK.reg |= PM_APBAMASK_GCLK;

	/* Software reset the module to ensure it is re-initialized correctly */
	/* Note: Due to synchronization, there is a delay from writing CTRL.SWRST until the reset is complete.
	 * CTRL.SWRST and STATUS.SYNCBUSY will both be cleared when the reset is complete, as described in chapter 13.8.1
	 */
	GCLK->CTRL.reg = GCLK_CTRL_SWRST ;

	while ( (GCLK->CTRL.reg & GCLK_CTRL_SWRST) && (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY) )
	{
		/* Wait for reset to complete */
	}

	/* ----------------------------------------------------------------------------------------------
	 * 2) Put OSC8m/8 as source of Generic Clock Generator 1 and divide by 40 (25 kHz)
	 */
	GCLK->GENDIV.reg = GCLK_GENDIV_ID( 1 ) | // Generic Clock Generator 1
					   GCLK_GENDIV_DIV(40);

	while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
	{
		/* Wait for synchronization */
	}

	/* Write Generic Clock Generator 1 configuration */
	GCLK->GENCTRL.reg = GCLK_GENCTRL_ID( 1 ) | // Generic Clock Generator 1
						GCLK_GENCTRL_SRC_OSC8M | // Selected source is External 32KHz Oscillator
						GCLK_GENCTRL_GENEN ;

	while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
	{
		/* Wait for synchronization */
	}

	/* ----------------------------------------------------------------------------------------------
	 * 3) Put Generic Clock Generator 1 as source for Generic Clock Multiplexer 0 (DFLL48M reference)
	 */
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID( 0 ) | // Generic Clock Multiplexer 0
						GCLK_CLKCTRL_GEN_GCLK1 | // Generic Clock Generator 1 is source
						GCLK_CLKCTRL_CLKEN ;

	while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
	{
		/* Wait for synchronization */
	}


	/* ----------------------------------------------------------------------------------------------
	 * 4) Enable DFLL48M clock
	 */

	/* DFLL Configuration in Closed Loop mode, cf product datasheet chapter 15.6.7.1 - Closed-Loop Operation */

	/* Remove the OnDemand mode, Bug http://avr32.icgroup.norway.atmel.com/bugzilla/show_bug.cgi?id=9905 */
	SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE;

	while ( (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY) == 0 )
	{
		/* Wait for synchronization */
	}

	SYSCTRL->DFLLMUL.reg = SYSCTRL_DFLLMUL_CSTEP( 31 ) | // Coarse step is 31, half of the max value
						   SYSCTRL_DFLLMUL_FSTEP( 511 ) | // Fine step is 511, half of the max value
						   SYSCTRL_DFLLMUL_MUL( (VARIANT_MCK + VARIANT_MAINOSC/2) / VARIANT_MAINOSC ) ; // External 32KHz is the reference

	while ( (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY) == 0 )
	{
		/* Wait for synchronization */
	}

	/* Write full configuration to DFLL control register */
	SYSCTRL->DFLLCTRL.reg |= SYSCTRL_DFLLCTRL_MODE | /* Enable the closed loop mode */
						   SYSCTRL_DFLLCTRL_WAITLOCK |
						   SYSCTRL_DFLLCTRL_QLDIS ; /* Disable Quick lock */

	while ( (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY) == 0 )
	{
		/* Wait for synchronization */
	}

	/* Enable the DFLL */
	SYSCTRL->DFLLCTRL.reg |= SYSCTRL_DFLLCTRL_ENABLE ;

	while ( (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLLCKC) == 0 ||
		  (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLLCKF) == 0 )
	{
		/* Wait for locks flags */
	}

	while ( (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY) == 0 )
	{
		/* Wait for synchronization */
	}


	/* ----------------------------------------------------------------------------------------------
	 * 5) Switch Generic Clock Generator 0 to DFLL48M. CPU will run at 48MHz.
	 */
	GCLK->GENDIV.reg = GCLK_GENDIV_ID( 0 ) ; // Generic Clock Generator 0

	while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
	{
		/* Wait for synchronization */
	}

	/* Write Generic Clock Generator 0 configuration */
	GCLK->GENCTRL.reg = GCLK_GENCTRL_ID( 0 ) | // Generic Clock Generator 0
						GCLK_GENCTRL_SRC_DFLL48M | // Selected source is DFLL 48MHz
						GCLK_GENCTRL_IDC | // Set 50/50 duty cycle
						GCLK_GENCTRL_GENEN ;

	while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
	{
		/* Wait for synchronization */
	}

	/* ----------------------------------------------------------------------------------------------
	* 6) Modify PRESCaler value of OSC8M to have 8MHz
	*/
	SYSCTRL->OSC8M.bit.PRESC = SYSCTRL_OSC8M_PRESC_0_Val ;  //CMSIS 4.5 changed the prescaler defines
	SYSCTRL->OSC8M.bit.ONDEMAND = 0 ;

	/* ----------------------------------------------------------------------------------------------
	* 7) Put OSC8M as source for Generic Clock Generator 3
	*/
	GCLK->GENDIV.reg = GCLK_GENDIV_ID( 3 ) ; // Generic Clock Generator 3

	/* Write Generic Clock Generator 3 configuration */
	GCLK->GENCTRL.reg = GCLK_GENCTRL_ID( 3 ) | // Generic Clock Generator 3
						GCLK_GENCTRL_SRC_OSC8M | // Selected source is RC OSC 8MHz (already enabled at reset)
						GCLK_GENCTRL_GENEN ;

	while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
	{
		/* Wait for synchronization */
	}

	/*
	 * Now that all system clocks are configured, we can set CPU and APBx BUS clocks.
	 * There values are normally the one present after Reset.
	 */
	PM->CPUSEL.reg  = PM_CPUSEL_CPUDIV_DIV1 ;
	PM->APBASEL.reg = PM_APBASEL_APBADIV_DIV1_Val ;
	PM->APBBSEL.reg = PM_APBBSEL_APBBDIV_DIV1_Val ;
	PM->APBCSEL.reg = PM_APBCSEL_APBCDIV_DIV1_Val ;


	/*
	 * 9) Disable automatic NVM write operations
	 */
	NVMCTRL->CTRLB.bit.MANW = 1;	
#else
	/* ----------------------------------------------------------------------------------------------
	* 6) Modify PRESCaler value of OSC8M to have 8MHz
	*/
	SYSCTRL->OSC8M.bit.PRESC = SYSCTRL_OSC8M_PRESC_0_Val ;  //CMSIS 4.5 changed the prescaler defines
	SYSCTRL->OSC8M.bit.ONDEMAND = 0 ;

	//9) Disable automatic NVM write operations
	NVMCTRL->CTRLB.bit.MANW = 1;
#endif
	
	base_adr = baseAddr;
	pageSize = pageSizes[NVMCTRL->PARAM.bit.PSZ];
	pageNum = NVMCTRL->PARAM.bit.NVMP;

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
	unsigned long sectorsNum = pageNum / FLASH_ROW_SIZE;
	unsigned long sectorSize = pageSize * FLASH_ROW_SIZE;
	unsigned long i, addr;

	for(i = 0, addr = 0; i < sectorsNum; i++, addr += sectorSize)
		FlashEraseSector(addr);

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
	NVMCTRL->ADDR.reg = sectorAddr / 2;
	NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_ER;
	while (!NVMCTRL->INTFLAG.bit.READY)
	{
	}
	return (0);
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
	unsigned long page;
	unsigned long * Flash;

	Flash = (unsigned long *) pageAddr;
	page = (pageAddr - base_adr) / FLASH_PAGE_SIZE_BYTE;

	// Disable automatic page write
	NVMCTRL->CTRLB.bit.MANW = 1;

	// Do writes in pages
	while (size)
	{
		// Execute "PBC" Page Buffer Clear
		NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_PBC;
		while (NVMCTRL->INTFLAG.bit.READY == 0)
		{
		}

		// Copy to the Write Buffer
		for (size = (size + 3) & ~3; size; size -= 4, buf += 4)
		{
			*Flash++ = *((unsigned long *)buf);
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
	int i;
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

