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

#include <sam.h>
#include "deca_types.h"
#include "deca_device_api.h"

/* Define our wanted value of CLOCKS_PER_SEC so that we have a millisecond tick timer. */
#ifndef CLOCKS_PER_SEC
#define CLOCKS_PER_SEC              1000
#endif

#define SPIx                        SERCOM4
#define SPIx_CS_PIN                 PIN_PA08
#define SPIx_CS_MASK                PORT_PA08
#define SPIx_SCK_PIN                PIN_PB11
#define SPIx_MISO_PIN               PIN_PA12
#define SPIx_MOSI_PIN               PIN_PB10

#define SPIx_SCK_MUX                MUX_PB11D_SERCOM4_PAD3
#define SPIx_MISO_MUX               MUX_PA12D_SERCOM4_PAD0
#define SPIx_MOSI_MUX               MUX_PB10D_SERCOM4_PAD2

#define SPIx_SCK_PAD                3
#define SPIx_MISO_PAD               0
#define SPIx_MOSI_PAD               2

#define SPIx_HIGH_RATE              (12000000UL)
#define SPIx_LOW_RATE               (2000000UL)

#define SPIx_CS2_PIN                PIN_PB02
#define SPIx_CS2_MASK               PORT_PB02

/*
#define DECARSTIRQ                  GPIO_Pin_0
#define DECARSTIRQ_GPIO             GPIOA
#define DECARSTIRQ_EXTI             EXTI_Line0
#define DECARSTIRQ_EXTI_PORT        GPIO_PortSourceGPIOA
#define DECARSTIRQ_EXTI_PIN         GPIO_PinSource0
#define DECARSTIRQ_EXTI_IRQn        EXTI0_IRQn
*/

#define DW1000_RSTn_PIN             PIN_PA14
#define DW1000_RSTn_MASK            PORT_PA14

#define DW1000_IRQ_PIN              PIN_PA09
#define DW1000_IRQ_MASK             PORT_PA09
#define DW1000_IRQ_EIC_IN           PIN_PA09A_EIC_EXTINT9
#define DW1000_IRQ_EIC_MASK         PORT_PA09A_EIC_EXTINT9


#define USARTx                      SERCOM0
#define USARTx_TX                   PIN_PA10         //PAD[2]
#define USARTx_RX                   PIN_PA11         //PAD[3]
#define USARTx_IRQn                 SERCOM0_IRQn

#define USER_LED_PIN                PIN_PB23
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
void setup_DW1000RSTnIRQ(int enable);
void port_led_write(int enable);
void port_led_toggle();

#ifdef __cplusplus
}
#endif

#endif /* PORT_H_ */
