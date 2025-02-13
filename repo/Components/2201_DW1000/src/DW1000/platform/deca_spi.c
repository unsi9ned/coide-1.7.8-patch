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
#include <nrf_gpio.h>
#include <nrf_spi.h>
#include <nrf_spim.h>
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
	nrf_gpio_pin_clear(SPIx_CS_PIN);

	for (i = 0; i < headerLength; i++)
	{
		SPIx->TXD = headerBuffer[i];        // Writing data into Data register

		while (!SPIx->EVENTS_READY);    // Waiting Complete Reception
		SPIx->EVENTS_READY = 0;

		SPIx->RXD;                          // Reading data
	}

	for (i = 0; i < bodylength; i++)
	{
		SPIx->TXD = bodyBuffer[i];          // Writing data into Data register

		while (!SPIx->EVENTS_READY);    // Waiting Complete Reception
		SPIx->EVENTS_READY = 0;

		SPIx->RXD;                          // Reading data
	}

	// CS = 1
	nrf_gpio_pin_set(SPIx_CS_PIN);

	decamutexoff(stat) ;

	return 0;
}
#else
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
	uint8_t dummyBuffer[8];

	stat = decamutexon() ;

	// CS = 0
	nrf_gpio_pin_clear(SPIx_CS_PIN);

	// Write header
	nrf_spim_rx_buffer_set(SPIx, dummyBuffer, 1);
	nrf_spim_tx_buffer_set(SPIx, headerBuffer, headerLength);

	nrf_spim_event_clear(SPIx, NRF_SPIM_EVENT_END);

//	nrf_spim_tx_list_enable(SPIx);
//	nrf_spim_rx_list_disable(SPIx);

	nrf_spim_task_trigger(SPIx, NRF_SPIM_TASK_START);

	// Write data
	if(bodylength > 0)
	{
		while(!nrf_spim_event_check(SPIx, NRF_SPIM_EVENT_STARTED));

		nrf_spim_rx_buffer_set(SPIx, dummyBuffer, 1);
		nrf_spim_tx_buffer_set(SPIx, bodyBuffer, bodylength);

		// End of header trasmission
		while(!nrf_spim_event_check(SPIx, NRF_SPIM_EVENT_END));
		nrf_spim_event_clear(SPIx, NRF_SPIM_EVENT_END);

		// Start body transmission
		nrf_spim_task_trigger(SPIx, NRF_SPIM_TASK_START);
	}

	//End of transaction
	while(!nrf_spim_event_check(SPIx, NRF_SPIM_EVENT_END));
	nrf_spim_event_clear(SPIx, NRF_SPIM_EVENT_END);

	// CS = 1
	nrf_gpio_pin_set(SPIx_CS_PIN);

	decamutexoff(stat) ;

	return 0;
}
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
	nrf_gpio_pin_clear(SPIx_CS_PIN);

	for (i = 0; i < headerLength; i++)
	{
		SPIx->TXD = headerBuffer[i];           // Writing data into Data register

		while (!SPIx->EVENTS_READY);       // Waiting Complete Reception
		SPIx->EVENTS_READY = 0;

		readBuffer[0] = SPIx->RXD;             // Reading data
	}

	for(i=0; i<readlength; i++)
	{
		SPIx->TXD = 0xFF;             // Dummy write as we read the message body

		while (!SPIx->EVENTS_READY);
		SPIx->EVENTS_READY = 0;

		readBuffer[i] = SPIx->RXD;
	}

	// CS = 1
	nrf_gpio_pin_set(SPIx_CS_PIN);

	decamutexoff(stat) ;

	return 0;
}
#else
int readfromspi
(
    uint16       headerLength,
    const uint8 *headerBuffer,
    uint32       readlength,
    uint8       *readBuffer
)
{

	uint8_t dummyBuffer[8];

	decaIrqStatus_t  stat ;

	if(!headerLength)
		return 1;

	stat = decamutexon() ;

	// CS = 0
	nrf_gpio_pin_clear(SPIx_CS_PIN);

	// Write header
	nrf_spim_rx_buffer_set(SPIx, dummyBuffer, 1);
	nrf_spim_tx_buffer_set(SPIx, headerBuffer, headerLength);

	nrf_spim_event_clear(SPIx, NRF_SPIM_EVENT_END);

//	nrf_spim_tx_list_enable(SPIx);
//	nrf_spim_rx_list_disable(SPIx);

	nrf_spim_task_trigger(SPIx, NRF_SPIM_TASK_START);

	// Read data
	if(readlength > 0)
	{
		while(!nrf_spim_event_check(SPIx, NRF_SPIM_EVENT_STARTED));

		memset(readBuffer, 0xFF, readlength);

		nrf_spim_rx_buffer_set(SPIx, readBuffer, readlength);
		nrf_spim_tx_buffer_set(SPIx, readBuffer, readlength);

		// End of header trasmission
		while(!nrf_spim_event_check(SPIx, NRF_SPIM_EVENT_END));
		nrf_spim_event_clear(SPIx, NRF_SPIM_EVENT_END);

		// Start body transmission
		nrf_spim_task_trigger(SPIx, NRF_SPIM_TASK_START);
	}

	//End of transaction
	while(!nrf_spim_event_check(SPIx, NRF_SPIM_EVENT_END));
	nrf_spim_event_clear(SPIx, NRF_SPIM_EVENT_END);

	// CS = 1
	nrf_gpio_pin_set(SPIx_CS_PIN);

	decamutexoff(stat) ;

	return 0;
}
#endif
