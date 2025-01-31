/* (C) 2018 Microchip Technology Inc. and its subsidiaries.
 * Subject to your compliance with these terms, you may use Microchip software
 *  and any derivatives exclusively with Microchip products. It is your 
 * responsibility to comply with third party license terms applicable to your
 *  use of third party software (including open source software) that may 
 * accompany Microchip software.
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE.
 * 
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, 
 * IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 
 
 		//This is another way to configure the Pin Mux that might be easier for some to understand
		const PORT_WRCONFIG_Type wrconfig = {
		.bit.WRPINCFG = 1,
		.bit.WRPMUX = 1,
		.bit.PMUX = MUX_PB16C_SERCOM5_PAD0,
		.bit.PMUXEN = 1,
		.bit.HWSEL = 1,
		.bit.INEN = 1,
		.bit.PINMASK = (uint16_t)((PORT_PB16) >> 16)
	};
		PORT->Group[1].WRCONFIG.reg = wrconfig.reg;
	
 		//This is another way to configure the Pin Mux that might be easier for some to understand	
		const PORT_WRCONFIG_Ty
		pe wrconfig1 = {
		.bit.WRPINCFG = 1,
		.bit.WRPMUX = 1,
		.bit.PMUX = MUX_PB22D_SERCOM5_PAD2,
		.bit.PMUXEN = 1,
		.bit.HWSEL = 1,
		.bit.PINMASK = (uint16_t)((PORT_PB22 | PORT_PB23) >> 16)
		};
	
		PORT->Group[1].WRCONFIG.reg |= wrconfig1.reg;
	
 */

#include <stdint.h>
#include "spi_master.h"


void SPI_init(void)
{
	/* Wait for synchronization */
	while(SPI_SERCOM->SPI.SYNCBUSY.bit.ENABLE);

	/* Disable the SERCOM SPI module */
	SPI_SERCOM->SPI.CTRLA.bit.ENABLE = 0;

	/* Wait for synchronization */
	while(SPI_SERCOM->SPI.SYNCBUSY.bit.SWRST);

	/* Perform a software reset */
	SPI_SERCOM->SPI.CTRLA.bit.SWRST = 1;

	/* Wait for synchronization */
	while(SPI_SERCOM->SPI.CTRLA.bit.SWRST);

	/* Wait for synchronization */
	while(SPI_SERCOM->SPI.SYNCBUSY.bit.SWRST || SERCOM5->SPI.SYNCBUSY.bit.ENABLE);
		
	//Using the WRCONFIG register to bulk configure PB16 for being configured the SERCOM SPI MASTER MISO
	uint32_t port = GPIO_PIN_MISO >> 5;
	uint32_t pin = GPIO_PIN_MISO % 32;
	uint32_t mask = (1UL << pin);
	uint8_t pmux;

	// MISO configuration
	PORT->Group[port].DIRCLR.reg = mask;
	PORT->Group[port].PINCFG[pin].bit.INEN = 1;
	PORT->Group[port].PINCFG[pin].bit.PULLEN = 0;

	pmux = PORT->Group[port].PMUX[pin >> 1].reg & (0xF << ((pin & 1) * 4));
	pmux |= (SPI_MISO_PORT_PMUX & 0xF) << ((pin & 1) * 4);
	PORT->Group[port].PMUX[pin >> 1].reg = pmux;
	PORT->Group[port].PINCFG[pin].bit.PMUXEN = 1;
	PORT->Group[port].PINCFG[pin].bit.DRVSTR = 1;

	//Setting SS output
	port = GPIO_PIN_SS >> 5;
	pin = GPIO_PIN_SS % 32;
	mask = (1UL << pin);

	PORT->Group[port].PINCFG[pin].reg = (uint8_t)(PORT_PINCFG_INEN | PORT_PINCFG_DRVSTR);
	PORT->Group[port].DIRSET.reg = mask;
	PORT->Group[port].OUTSET.reg = mask;

	// MOSI configuration
	port = GPIO_PIN_MOSI >> 5;
	pin = GPIO_PIN_MOSI % 32;
	mask = (1UL << pin);

	PORT->Group[port].DIRSET.reg = mask;
	PORT->Group[port].PINCFG[pin].bit.INEN = 0;
	PORT->Group[port].PINCFG[pin].bit.PULLEN = 0;

	pmux = PORT->Group[port].PMUX[pin >> 1].reg;
	pmux |= (SPI_MOSI_PORT_PMUX & 0xF) << ((pin & 1) * 4);
	PORT->Group[port].PMUX[pin >> 1].reg = pmux;
	PORT->Group[port].PINCFG[pin].bit.PMUXEN = 1;
	PORT->Group[port].PINCFG[pin].bit.DRVSTR = 1;

	// SCK configuration
	port = GPIO_PIN_SCK >> 5;
	pin = GPIO_PIN_SCK % 32;
	mask = (1UL << pin);

	PORT->Group[port].DIRSET.reg = mask;
	PORT->Group[port].PINCFG[pin].bit.INEN = 0;
	PORT->Group[port].PINCFG[pin].bit.PULLEN = 0;

	pmux = PORT->Group[port].PMUX[pin >> 1].reg;
	pmux |= (SPI_SCK_PORT_PMUX & 0xF) << ((pin & 1) * 4);
	PORT->Group[port].PMUX[pin >> 1].reg = pmux;
	PORT->Group[port].PINCFG[pin].bit.PMUXEN = 1;
	PORT->Group[port].PINCFG[pin].bit.DRVSTR = 1;

	PM->APBCMASK.reg |= PM_APBCMASK_SERCOM4;								//Enable the SERCOM 5 under the PM
	
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(SPI_SERCOM_GCLK_ID_CORE) |		//Provide necessary clocks to the peripheral
						GCLK_CLKCTRL_CLKEN |
						GCLK_CLKCTRL_GEN(SPI_SERCOM_CLK_GEN);
	
	while(GCLK->STATUS.bit.SYNCBUSY);										//Wait for clock sync
		
	SPI_SERCOM->SPI.CTRLA.reg = SERCOM_SPI_CTRLA_MODE_SPI_MASTER |			//Configure the Peripheral as SPI Master
								SERCOM_SPI_CTRLA_DOPO(SPI_DOPO) |			//DOPO is set to PAD[2,3]
								SERCOM_SPI_CTRLA_DIPO(SPI_DIPO);
	
	SPI_SERCOM->SPI.CTRLB.reg = SERCOM_SPI_CTRLB_RXEN;						//Enable receive on SPI
	
	uint16_t BAUD_REG = ((float)SPI_CLK_FREQ / (float)(2 * SPI_BAUD)) - 1;	//Calculate BAUD value
	SPI_SERCOM->SPI.BAUD.reg =	SERCOM_SPI_BAUD_BAUD(BAUD_REG);				//Set the SPI baud rate	
	SPI_SERCOM->SPI.CTRLA.reg |= SERCOM_SPI_CTRLA_ENABLE;					//Enable the Sercom SPI
	while(SPI_SERCOM->SPI.SYNCBUSY.bit.ENABLE);								//What for the enable to finish
	
}

uint8_t spiSend(uint8_t data)
{	
	while(SPI_SERCOM->SPI.INTFLAG.bit.DRE == 0);
	SPI_SERCOM->SPI.DATA.reg = data;
	while(SPI_SERCOM->SPI.INTFLAG.bit.RXC == 0);
	return (uint8_t)SPI_SERCOM->SPI.DATA.reg;	
}
