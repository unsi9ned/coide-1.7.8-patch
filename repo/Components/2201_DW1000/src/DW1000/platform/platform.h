/*! ----------------------------------------------------------------------------
 * @file	port.h
 * @brief	HW specific definitions and functions for portability
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */


#ifndef PORT_H_
#define PORT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <nrf.h>
#include <nrf_uart.h>
#include <nrf_spi.h>
#include "deca_types.h"
#include "deca_device_api.h"

/* Define our wanted value of CLOCKS_PER_SEC so that we have a millisecond tick timer. */
#ifndef CLOCKS_PER_SEC
#define CLOCKS_PER_SEC              1000
#endif

#ifndef DW1000_USE_DMA
#define DW1000_USE_DMA              1
#endif

#if DW1000_USE_DMA
#define SPIx                        NRF_SPIM1
#else
#define SPIx                        NRF_SPI1
#endif
#define SPIx_CS_PIN                 17
#define SPIx_SCK_PIN                16
#define SPIx_MISO_PIN               18
#define SPIx_MOSI_PIN               20

#define SPIx_HIGH_RATE              (8000000UL)
#define SPIx_LOW_RATE               (2000000UL)

#define DW1000_RSTn_PIN             24
#define DW1000_IRQ_PIN              19


#define USARTx                      NRF_UART0
#define USARTx_TX                   5
#define USARTx_RX                   11
#define USARTx_IRQn                 UARTE0_UART0_IRQn

#define USER_LED_PIN                31
#define USER_LED_POLARITY           0


#define port_GET_stack_pointer()	__get_MSP()
#define port_GET_rtc_time()			RTC_GetCounter()
#define port_SET_rtc_time(x)		RTC_SetCounter(x)

void             port_DisableEXT_IRQ();
void             port_EnableEXT_IRQ();
decaIrqStatus_t  port_GetEXT_IRQStatus();

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn peripherals_init()
 *
 * @brief Initialise all peripherals.
 *
 * @param none
 *
 * @return none
 */
void peripherals_init (void);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn port_set_deca_isr()
 *
 * @brief This function is used to install the handling function for DW1000 IRQ.
 *
 * NOTE:
 *   - As EXTI9_5_IRQHandler does not check that port_deca_isr is not null, the user application must ensure that a
 *     proper handler is set by calling this function before any DW1000 IRQ occurs!
 *   - This function makes sure the DW1000 IRQ line is deactivated while the handler is installed.
 *
 * @param deca_isr function pointer to DW1000 interrupt handler to install
 *
 * @return none
 */
void port_set_deca_isr(void (*deca_isr)());

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn spi_set_rate_low()
 *
 * @brief Set SPI rate to less than 3 MHz to properly perform DW1000 initialisation.
 *
 * @param none
 *
 * @return none
 */
void spi_set_rate_low (void);

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn spi_set_rate_high()
 *
 * @brief Set SPI rate as close to 20 MHz as possible for optimum performances.
 *
 * @param none
 *
 * @return none
 */
void spi_set_rate_high (void);

uint32 portGetTickCount(void);
void reset_DW1000(void);
void port_led_write(int enable);
void port_led_toggle();

#ifdef __cplusplus
}
#endif

#endif /* PORT_H_ */
