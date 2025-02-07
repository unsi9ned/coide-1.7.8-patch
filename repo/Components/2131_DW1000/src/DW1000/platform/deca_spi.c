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

extern DmacDescriptor  *spiReadDescriptor;
extern DmacDescriptor  *spiWriteDescriptor;

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
#if !DW1000_USE_DMA
int writetospi
(
    uint16       headerLength,
    const uint8 *headerBuffer,
    uint32       bodylength,
    const uint8 *bodyBuffer
)
{
	int i = 0;
	decaIrqStatus_t stat;

    stat = decamutexon() ;

    // CS = 0
    PORT->Group[SPIx_CS_PIN >> 5].OUTCLR.reg = SPIx_CS_MASK;

	for (i = 0; i < headerLength; i++)
	{
		SPIx->SPI.DATA.bit.DATA = headerBuffer[i]; // Writing data into Data register

		while (SPIx->SPI.INTFLAG.bit.RXC == 0);    // Waiting Complete Reception

		SPIx->SPI.DATA.bit.DATA;                   // Reading data
	}

	for (i = 0; i < bodylength; i++)
	{
		SPIx->SPI.DATA.bit.DATA = bodyBuffer[i];   // Writing data into Data register

		while (SPIx->SPI.INTFLAG.bit.RXC == 0);    // Waiting Complete Reception

		SPIx->SPI.DATA.bit.DATA;                   // Reading data
	}

	// CS = 1
	PORT->Group[SPIx_CS_PIN >> 5].OUTSET.reg = SPIx_CS_MASK;

	decamutexoff(stat) ;

    return 0;
} // end writetospi()
#else
int writetospi
(
    uint16       headerLength,
    const uint8 *headerBuffer,
    uint32       bodylength,
    const uint8 *bodyBuffer
)
{
	decaIrqStatus_t stat;
	uint8_t dum = 0xFF;

	headerLength &= 0x3;
	bodylength &= 0xffff;

	stat = decamutexon();

	// CS = 0
	PORT->Group[SPIx_CS_PIN >> 5].OUTCLR.reg = SPIx_CS_MASK;

	if(headerLength)
	{
		spiWriteDescriptor->BTCNT.reg = \
		spiReadDescriptor->BTCNT.reg = headerLength;

		// Increment source pointer
		// Указатель на конец буфера
		headerBuffer += headerLength;
		spiWriteDescriptor->SRCADDR.reg       = (uint32_t)headerBuffer;
		spiWriteDescriptor->BTCTRL.bit.SRCINC = 1;

		spiReadDescriptor->DSTADDR.reg        = (uint32_t)&dum;
		spiReadDescriptor->BTCTRL.bit.DSTINC  = 0;

		// Enable the transfer channel
		DMAC->CHID.bit.ID = SPIx_DMA_RX_CH;
		DMAC->CHCTRLA.bit.ENABLE = 1;

		DMAC->CHID.bit.ID = SPIx_DMA_TX_CH;
		DMAC->CHCTRLA.bit.ENABLE = 1;

		// Ждем когда данные будут приняты
		DMAC->CHID.bit.ID = SPIx_DMA_RX_CH;
		while (DMAC->CHINTFLAG.bit.TCMPL == 0 && DMAC->CHINTFLAG.bit.TERR == 0);
		DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_TERR |
							  DMAC_CHINTENCLR_TCMPL |
							  DMAC_CHINTENCLR_SUSP;
	}


	if(bodylength)
	{
		spiWriteDescriptor->BTCNT.reg = \
		spiReadDescriptor->BTCNT.reg = bodylength;

		// Not increment source pointer
		bodyBuffer += bodylength;
		spiWriteDescriptor->SRCADDR.reg       = (uint32_t)bodyBuffer;

		// Enable the transfer channel
		DMAC->CHID.bit.ID = SPIx_DMA_RX_CH;
		DMAC->CHCTRLA.bit.ENABLE = 1;

		DMAC->CHID.bit.ID = SPIx_DMA_TX_CH;
		DMAC->CHCTRLA.bit.ENABLE = 1;

		DMAC->CHID.bit.ID = SPIx_DMA_RX_CH;
		while (DMAC->CHINTFLAG.bit.TCMPL == 0 && DMAC->CHINTFLAG.bit.TERR == 0);
		DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_TERR |
							  DMAC_CHINTENCLR_TCMPL |
							  DMAC_CHINTENCLR_SUSP;
	}

	// CS = 1
	PORT->Group[SPIx_CS_PIN >> 5].OUTSET.reg = SPIx_CS_MASK;

	decamutexoff(stat) ;

    return 0;
} // end writetospi()
#endif


/*! ------------------------------------------------------------------------------------------------------------------
 * Function: readfromspi()
 *
 * Low level abstract function to read from the SPI
 * Takes two separate byte buffers for write header and read data
 * returns the offset into read buffer where first byte of read data may be found,
 * or returns -1 if there was an error
 */
#if !DW1000_USE_DMA
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

	for (i = 0; i < headerLength; i++)
	{
		SPIx->SPI.DATA.bit.DATA = headerBuffer[i]; // Writing data into Data register

		while (SPIx->SPI.INTFLAG.bit.RXC == 0);    // Waiting Complete Reception

		readBuffer[0] = SPIx->SPI.DATA.bit.DATA;             // Reading data
	}

	for(i=0; i<readlength; i++)
	{
		SPIx->SPI.DATA.bit.DATA = 0;             // Dummy write as we read the message body

		while (SPIx->SPI.INTFLAG.bit.RXC == 0);

		readBuffer[i] = SPIx->SPI.DATA.bit.DATA; //port_SPIx_receive_data(); //this clears RXNE bit
	}

	// CS = 1
	PORT->Group[SPIx_CS_PIN >> 5].OUTSET.reg = SPIx_CS_MASK;

    decamutexoff(stat) ;

    return 0;
} // end readfromspi()
#else
int readfromspi
(
    uint16       headerLength,
    const uint8 *headerBuffer,
    uint32       readlength,
    uint8       *readBuffer
)
{

	static uint32_t dum = 0xFFFFFFFF; // Dummy byte for read-only xfers

    decaIrqStatus_t  stat ;

	headerLength &= 0x3;
	readlength &= 0xffff;

	stat = decamutexon() ;

	// CS = 0
	PORT->Group[SPIx_CS_PIN >> 5].OUTCLR.reg = SPIx_CS_MASK;

	if(headerLength)
	{
		spiWriteDescriptor->BTCNT.reg = \
		spiReadDescriptor->BTCNT.reg = headerLength;

		// Increment source pointer
		headerBuffer += headerLength;
		spiWriteDescriptor->SRCADDR.reg       = (uint32_t)headerBuffer;
		spiWriteDescriptor->BTCTRL.bit.SRCINC = 1;

		spiReadDescriptor->DSTADDR.reg        = (uint32_t)&dum;
		spiReadDescriptor->BTCTRL.bit.DSTINC  = 0;

		// Enable the transfer channel
		DMAC->CHID.bit.ID = SPIx_DMA_RX_CH;
		DMAC->CHCTRLA.bit.ENABLE = 1;

		DMAC->CHID.bit.ID = SPIx_DMA_TX_CH;
		DMAC->CHCTRLA.bit.ENABLE = 1;

		DMAC->CHID.bit.ID = SPIx_DMA_RX_CH;
		while (DMAC->CHINTFLAG.bit.TCMPL == 0 && DMAC->CHINTFLAG.bit.TERR == 0);
		DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_TERR |
							  DMAC_CHINTENCLR_TCMPL |
							  DMAC_CHINTENCLR_SUSP;
	}

	if(readlength)
	{
		spiWriteDescriptor->BTCNT.reg = \
		spiReadDescriptor->BTCNT.reg = readlength;

		// Not increment source pointer
		readBuffer += readlength;
		spiWriteDescriptor->SRCADDR.reg       = (uint32_t)&dum;
		spiWriteDescriptor->SRCADDR.reg      += 3;
		spiWriteDescriptor->BTCTRL.bit.SRCINC = 0;

		spiReadDescriptor->DSTADDR.reg       = (uint32_t)readBuffer;
		spiReadDescriptor->BTCTRL.bit.DSTINC  = 1;

		// Enable the transfer channel
		DMAC->CHID.bit.ID = SPIx_DMA_RX_CH;
		DMAC->CHCTRLA.bit.ENABLE = 1;

		DMAC->CHID.bit.ID = SPIx_DMA_TX_CH;
		DMAC->CHCTRLA.bit.ENABLE = 1;

		DMAC->CHID.bit.ID = SPIx_DMA_RX_CH;
		while (DMAC->CHINTFLAG.bit.TCMPL == 0 && DMAC->CHINTFLAG.bit.TERR == 0);
		DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_TERR |
							  DMAC_CHINTENCLR_TCMPL |
							  DMAC_CHINTENCLR_SUSP;
	}

	// CS = 1
	PORT->Group[SPIx_CS_PIN >> 5].OUTSET.reg = SPIx_CS_MASK;

	decamutexoff(stat) ;

	return 0;
} // end readfromspi()
#endif
