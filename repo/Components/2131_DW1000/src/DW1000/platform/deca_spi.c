/*! ----------------------------------------------------------------------------
 * @file	deca_spi.c
 * @brief	SPI access functions
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */
#include <string.h>

#include "deca_device_api.h"
#include "deca_spi.h"
#include "deca_sleep.h"
#include "deca_device_api.h"
#include "platform.h"


/*! ------------------------------------------------------------------------------------------------------------------
 * Function: openspi()
 *
 * Low level abstract function to open and initialise access to the SPI device.
 * returns 0 for success, or -1 for error
 */
int openspi(/*SPI_TypeDef* SPIx*/)
{
	// done by port.c, default SPI used is SPI1

	return 0;

} // end openspi()

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: closespi()
 *
 * Low level abstract function to close the the SPI device.
 * returns 0 for success, or -1 for error
 */
int closespi(void)
{
	return 0;

} // end closespi()

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: writetospi()
 *
 * Low level abstract function to write to the SPI
 * Takes two separate byte buffers for write header and write data
 * returns 0 for success, or -1 for error
 */
int writetospi
(
    uint16       headerLength,
    const uint8 *headerBuffer,
    uint32       bodylength,
    const uint8 *bodyBuffer
)
{
	static uint8 temp;
	int i = 0;
	decaIrqStatus_t stat;

    stat = decamutexon() ;

    // CS = 0
    PORT->Group[SPIx_CS_PIN >> 5].OUTCLR.reg = SPIx_CS_MASK;
    PORT->Group[SPIx_CS2_PIN >> 5].OUTCLR.reg = SPIx_CS2_MASK;

	for (i = 0; i < headerLength; i++)
	{
		while (SPIx->SPI.INTFLAG.bit.DRE == 0);    // Waiting Complete Transmission

		SPIx->SPI.DATA.bit.DATA = headerBuffer[i]; // Writing data into Data register

		while (SPIx->SPI.INTFLAG.bit.RXC == 0);    // Waiting Complete Reception

		temp = SPIx->SPI.DATA.bit.DATA;             // Reading data
	}

	for (i = 0; i < bodylength; i++)
	{
		while (SPIx->SPI.INTFLAG.bit.DRE == 0);    // Waiting Complete Transmission

		SPIx->SPI.DATA.bit.DATA = bodyBuffer[i]; // Writing data into Data register

		while (SPIx->SPI.INTFLAG.bit.RXC == 0);    // Waiting Complete Reception

		temp = SPIx->SPI.DATA.bit.DATA;             // Reading data
	}

	// CS = 1
	PORT->Group[SPIx_CS_PIN >> 5].OUTSET.reg = SPIx_CS_MASK;
	PORT->Group[SPIx_CS2_PIN >> 5].OUTSET.reg = SPIx_CS2_MASK;

	decamutexoff(stat) ;

    return 0;
} // end writetospi()


/*! ------------------------------------------------------------------------------------------------------------------
 * Function: readfromspi()
 *
 * Low level abstract function to read from the SPI
 * Takes two separate byte buffers for write header and read data
 * returns the offset into read buffer where first byte of read data may be found,
 * or returns -1 if there was an error
 */

int readfromspi
(
    uint16       headerLength,
    const uint8 *headerBuffer,
    uint32       readlength,
    uint8       *readBuffer
)
{

	int i=0;

    decaIrqStatus_t  stat ;

    stat = decamutexon() ;

    // CS = 0
    PORT->Group[SPIx_CS_PIN >> 5].OUTCLR.reg = SPIx_CS_MASK;
    PORT->Group[SPIx_CS2_PIN >> 5].OUTCLR.reg = SPIx_CS2_MASK;

	for (i = 0; i < headerLength; i++)
	{
		while (SPIx->SPI.INTFLAG.bit.DRE == 0);    // Waiting Complete Transmission

		SPIx->SPI.DATA.bit.DATA = headerBuffer[i]; // Writing data into Data register

		while (SPIx->SPI.INTFLAG.bit.RXC == 0);    // Waiting Complete Reception

		readBuffer[0] = SPIx->SPI.DATA.bit.DATA;             // Reading data
	}

	for(i=0; i<readlength; i++)
	{
		while (SPIx->SPI.INTFLAG.bit.DRE == 0);    // Waiting Complete Transmission

		SPIx->SPI.DATA.bit.DATA = 0;             // Dummy write as we read the message body

		while (SPIx->SPI.INTFLAG.bit.RXC == 0);

		readBuffer[i] = SPIx->SPI.DATA.bit.DATA; //port_SPIx_receive_data(); //this clears RXNE bit
	}

	// CS = 1
	PORT->Group[SPIx_CS_PIN >> 5].OUTSET.reg = SPIx_CS_MASK;
	PORT->Group[SPIx_CS2_PIN >> 5].OUTSET.reg = SPIx_CS2_MASK;

    decamutexoff(stat) ;

    return 0;
} // end readfromspi()
