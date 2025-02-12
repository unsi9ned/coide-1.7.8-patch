/*! ----------------------------------------------------------------------------
 * @file    port.c
 * @brief   HW specific definitions and functions for portability
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <nrf.h>
#include <nrf_gpio.h>
#include <nrf_uart.h>
#include <nrf_spi.h>
#include "deca_sleep.h"
#include "deca_device_api.h"
#include "platform.h"

static uint32 sysTickCounter = 0;
static void (*decaIrqHandler)() = NULL;

//------------------------------------------------------------------------------
// Инициализация портов ввода/вывода
//------------------------------------------------------------------------------
static void gpio_init()
{
	nrf_gpio_cfg_input(DW1000_IRQ_PIN, NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_input(DW1000_RSTn_PIN, NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_input(USARTx_RX, NRF_GPIO_PIN_PULLUP);
//	nrf_gpio_cfg_output(USARTx_TX);
	nrf_gpio_cfg_output(SPIx_CS_PIN);
	nrf_gpio_pin_set(SPIx_CS_PIN);
//	nrf_gpio_cfg_output(SPIx_MOSI_PIN);
//	nrf_gpio_cfg_output(SPIx_SCK_PIN);
//	nrf_gpio_cfg_input(SPIx_MISO_PIN, NRF_GPIO_PIN_PULLUP);
}

//------------------------------------------------------------------------------
// Настройка порта светодиода
//------------------------------------------------------------------------------
static int gpio_led_init()
{
	nrf_gpio_cfg_output(USER_LED_PIN);
	nrf_gpio_pin_write(USER_LED_PIN, USER_LED_POLARITY ? 1 : 0);
	return 1;
}

//------------------------------------------------------------------------------
// Зажечь или погасить светодиод
//------------------------------------------------------------------------------
void port_led_write(int enable)
{
	static int init = 0;
	init = !init ? gpio_led_init() : 1;
	nrf_gpio_pin_write(USER_LED_PIN, enable & 1);
}

//------------------------------------------------------------------------------
// Изменить состояние светодиода на противоположное
//------------------------------------------------------------------------------
void port_led_toggle()
{
	static int init = 0;
	init = !init ? gpio_led_init() : 1;
	nrf_gpio_pin_write(USER_LED_PIN, !nrf_gpio_pin_read(USER_LED_PIN));
}

//------------------------------------------------------------------------------
// Аппаратный сброс модуля
//------------------------------------------------------------------------------
void reset_DW1000(void)
{
	nrf_gpio_pin_dir_set(DW1000_RSTn_PIN, NRF_GPIO_PIN_DIR_OUTPUT);
	nrf_gpio_pin_write(DW1000_RSTn_PIN, 0);
	deca_sleep(2);
	nrf_gpio_pin_dir_set(DW1000_RSTn_PIN, NRF_GPIO_PIN_DIR_INPUT);
}

//------------------------------------------------------------------------------
// Отключить прерывания по пину IRQ
//------------------------------------------------------------------------------
void port_DisableEXT_IRQ()
{
//	uint32 in = DW1000_IRQ_EIC_IN;
//
//	if(in == PIN_PA08A_EIC_NMI)
//	{
//		EIC->NMICTRL.bit.NMISENSE = 0; // Turn off detection
//	}
//	else
//	{
//		EIC->INTENCLR.reg = EIC_INTENCLR_EXTINT(1 << in);
//
//		// Disable wakeup capability on pin during sleep
//		EIC->WAKEUP.reg &= ~(1 << in);
//	}
}

//------------------------------------------------------------------------------
// Включить прерывания по пину IRQ
//------------------------------------------------------------------------------
void port_EnableEXT_IRQ()
{
//	uint32_t config;
//	uint32_t pos;
//	uint32 in = DW1000_IRQ_EIC_IN;
//
//	// Enable wakeup capability on pin in case being used during sleep
//	EIC->WAKEUP.reg |= (1 << in);
//
//	// Look for right CONFIG register to be addressed
//	if(in > 7)
//	{
//		config = 1;
//		pos = (in - 8) << 2;
//	}
//	else
//	{
//		config = 0;
//		pos = in << 2;
//	}
//
//	// Reset sense mode, important when changing trigger mode during runtime
//	EIC->CONFIG[config].reg &=~ (EIC_CONFIG_SENSE0_Msk << pos);
//	EIC->CONFIG[config].reg |= EIC_CONFIG_SENSE0_RISE_Val << pos;
//
//	// Enable the interrupt
//	EIC->INTENSET.reg = EIC_INTENSET_EXTINT(1 << in);
}

//------------------------------------------------------------------------------
// Определяет статус прерываний по IRQ пину.
// 1 - включено, 0 - отключено
//------------------------------------------------------------------------------
decaIrqStatus_t port_GetEXT_IRQStatus()
{
//	decaIrqStatus_t bitstatus;
//	uint32 in = DW1000_IRQ_EIC_IN;
//
//	if(in != PIN_PA08A_EIC_NMI)
//	{
//		bitstatus = (EIC->INTENSET.reg & EIC_INTENCLR_EXTINT(1 << in)) ? 1 : 0;
//	}
//
//	return bitstatus;
}

//------------------------------------------------------------------------------
// Установка обработчика прерываний по пину IRQ
//------------------------------------------------------------------------------
void port_set_deca_isr(void (*deca_isr)())
{
//	/* Check DW1000 IRQ activation status. */
//	decaIrqStatus_t en = port_GetEXT_IRQStatus();
//
//	/* If needed, deactivate DW1000 IRQ during the installation of the new handler. */
//	if(en)
//	{
//		port_DisableEXT_IRQ();
//	}
//	decaIrqHandler = deca_isr;
//
//	if(en)
//	{
//		port_EnableEXT_IRQ();
//	}
}

//------------------------------------------------------------------------------
// Обработчик внешних прерываний
//------------------------------------------------------------------------------
void __attribute__ ((weak)) EIC_Handler(void)
{
//	if((EIC->INTFLAG.reg & DW1000_IRQ_EIC_MASK) && decaIrqHandler)
//	{
//		decaIrqHandler();
//	}
//	// Clear the interrupt
//	EIC->INTFLAG.reg = 0xFFFF;
}

//------------------------------------------------------------------------------
// Настройка прерываний периферии
//------------------------------------------------------------------------------
static void interrupt_init()
{
//	NVIC_DisableIRQ(EIC_IRQn);
//	NVIC_ClearPendingIRQ(EIC_IRQn);
//	NVIC_SetPriority(EIC_IRQn, 0);
//	NVIC_EnableIRQ(EIC_IRQn);
//
//	// Enable GCLK for IEC (External Interrupt Controller)
//	GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_EIC_Val));
//
//	// Enable EIC
//	EIC->CTRL.bit.ENABLE = 1;
//	while (EIC->STATUS.bit.SYNCBUSY == 1) { }
//
//	gpio_periph_enable(DW1000_IRQ_PIN, DW1000_IRQ_MUX);
//	port_EnableEXT_IRQ();
}

//------------------------------------------------------------------------------
// Обработчик прерываний системного таймера
//------------------------------------------------------------------------------
void SysTick_Handler(void)
{
	sysTickCounter++;
}

//------------------------------------------------------------------------------
// Настройка системного таймера на частоту 1 кГц
//------------------------------------------------------------------------------
static int systick_init(void)
{
	if(SysTick_Config(SystemCoreClock / CLOCKS_PER_SEC))
	{
		while (1);
	}
	NVIC_SetPriority(SysTick_IRQn, 5);

	return 0;
}

//------------------------------------------------------------------------------
// Вовзращает значение системного тика
//------------------------------------------------------------------------------
uint32 portGetTickCount(void)
{
	return sysTickCounter;
}

//------------------------------------------------------------------------------
// Инициализация отладочного UART
//------------------------------------------------------------------------------
static void usart_init()
{
	// Контроль четности отключен, аппаратный контроль выключен
	USARTx->CONFIG = 0;
	USARTx->PSELTXD = USARTx_TX;
	USARTx->BAUDRATE = 0x01D7E000;  //115200
	USARTx->ENABLE = 4;
	USARTx->TASKS_STARTTX = 1;
}

//------------------------------------------------------------------------------
// Вывод символа через UART. Используется в printf
//------------------------------------------------------------------------------
void UART_PutChar(char c)
{
	USARTx->TXD = c;
	while(USARTx->EVENTS_TXDRDY != 1);
	USARTx->EVENTS_TXDRDY = 0;
}

//------------------------------------------------------------------------------
// Отправка строки через UART
//------------------------------------------------------------------------------
void uartSendStr(const char * str)
{
	while (*str)
		UART_PutChar(*str++);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn spi_peripheral_init()
 *
 * @brief Initialise all SPI peripherals at once.
 *
 * @param none
 *
 * @return none
 */
static void spi_peripheral_init(void)
{
	SPIx->CONFIG = SPI_CONFIG_ORDER_MsbFirst |
				   SPI_CONFIG_CPHA_Leading |
				   SPI_CONFIG_CPOL_ActiveHigh;

	SPIx->FREQUENCY = SPI_FREQUENCY_FREQUENCY_M2;

	SPIx->PSELSCK = SPIx_SCK_PIN;
	SPIx->PSELMOSI = SPIx_MOSI_PIN;
	SPIx->PSELMISO = SPIx_MISO_PIN;

	SPIx->ENABLE = 1;
}

//------------------------------------------------------------------------------
// Изменение скорости передачи данных SPI
//------------------------------------------------------------------------------
static void spi_change_rate(uint32 scalingfactor)
{
	if(scalingfactor == SPIx_LOW_RATE)
		SPIx->FREQUENCY = SPI_FREQUENCY_FREQUENCY_M2;
	else if(scalingfactor == SPIx_HIGH_RATE)
		SPIx->FREQUENCY = SPI_FREQUENCY_FREQUENCY_M8;
	else
		SPIx->FREQUENCY = SPI_FREQUENCY_FREQUENCY_M1;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn spi_set_rate_low()
 *
 * @brief Set SPI rate to less than 3 MHz to properly perform DW1000 initialisation.
 *
 * @param none
 *
 * @return none
 */
void spi_set_rate_low (void)
{
    spi_change_rate(SPIx_LOW_RATE);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn spi_set_rate_high()
 *
 * @brief Set SPI rate as close to 20 MHz as possible for optimum performances.
 *
 * @param none
 *
 * @return none
 */
void spi_set_rate_high (void)
{
	spi_change_rate(SPIx_HIGH_RATE);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn peripherals_init()
 *
 * @brief Initialise all peripherals.
 *
 * @param none
 *
 * @return none
 */
void peripherals_init (void)
{
	gpio_init();
	interrupt_init();
	systick_init();
	spi_peripheral_init();
	usart_init();
}


