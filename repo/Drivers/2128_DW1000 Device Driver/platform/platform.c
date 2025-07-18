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
#include <nrf_gpiote.h>
#include <nrf_uart.h>
#include <nrf_spi.h>
#include <nrf_spim.h>
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
	nrf_gpio_cfg_input(DW1000_RSTn_PIN, NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_input(USARTx_RX, NRF_GPIO_PIN_PULLUP);
//	nrf_gpio_cfg_output(USARTx_TX);
	nrf_gpio_cfg_output(SPIx_CS_PIN);
	nrf_gpio_pin_set(SPIx_CS_PIN);
	nrf_gpio_cfg_output(SPIx_MOSI_PIN);
	nrf_gpio_cfg_output(SPIx_SCK_PIN);
	nrf_gpio_cfg_input(SPIx_MISO_PIN, NRF_GPIO_PIN_PULLUP);
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

	nrf_gpio_pin_toggle(USER_LED_PIN);
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
	nrf_gpiote_int_disable((1ul << DW1000_IRQ_PIN_EVENT_ID));
}

//------------------------------------------------------------------------------
// Включить прерывания по пину IRQ
//------------------------------------------------------------------------------
void port_EnableEXT_IRQ()
{
	nrf_gpiote_int_enable((1ul << DW1000_IRQ_PIN_EVENT_ID));
}

//------------------------------------------------------------------------------
// Определяет статус прерываний по IRQ пину.
// 1 - включено, 0 - отключено
//------------------------------------------------------------------------------
decaIrqStatus_t port_GetEXT_IRQStatus()
{
	decaIrqStatus_t bitstatus;

	if(nrf_gpiote_int_is_enabled(1ul << DW1000_IRQ_PIN_EVENT_ID))
	{
		bitstatus = 1;
	}
	else
	{
		bitstatus = 0;
	}

	return bitstatus;
}

//------------------------------------------------------------------------------
// Установка обработчика прерываний по пину IRQ
//------------------------------------------------------------------------------
void port_set_deca_isr(void (*deca_isr)())
{
	/* Check DW1000 IRQ activation status. */
	decaIrqStatus_t en = port_GetEXT_IRQStatus();

	/* If needed, deactivate DW1000 IRQ during the installation of the new handler. */
	if(en)
	{
		port_DisableEXT_IRQ();
	}
	decaIrqHandler = deca_isr;

	if(en)
	{
		port_EnableEXT_IRQ();
	}
}

//------------------------------------------------------------------------------
// Обработчик внешних прерываний
//------------------------------------------------------------------------------
void GPIOTE_IRQHandler(void)
{
	if(decaIrqHandler)
	{
		decaIrqHandler();
	}
	// Clear the interrupt
	nrf_gpiote_event_clear(DW1000_IRQ_PIN_EVENT(DW1000_IRQ_PIN_EVENT_ID));
}

//------------------------------------------------------------------------------
// Настройка прерываний периферии
//------------------------------------------------------------------------------
static void interrupt_init()
{
	nrf_gpio_cfg_input(DW1000_IRQ_PIN, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpiote_event_enable(DW1000_IRQ_PIN_EVENT_ID);
	nrf_gpiote_event_configure(DW1000_IRQ_PIN_EVENT_ID, DW1000_IRQ_PIN, NRF_GPIOTE_POLARITY_LOTOHI);
	nrf_gpiote_event_clear(DW1000_IRQ_PIN_EVENT(DW1000_IRQ_PIN_EVENT_ID));

	NVIC_DisableIRQ(GPIOTE_IRQn);
	NVIC_ClearPendingIRQ(GPIOTE_IRQn);
	NVIC_SetPriority(GPIOTE_IRQn, 0);
	NVIC_EnableIRQ(GPIOTE_IRQn);

	port_EnableEXT_IRQ();
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

	SPIx->PSEL.SCK = SPIx_SCK_PIN;
	SPIx->PSEL.MOSI = SPIx_MOSI_PIN;
	SPIx->PSEL.MISO = SPIx_MISO_PIN;

#if DW1000_USE_DMA
	SPIx->ORC = 0xFF;
	SPIx->RXD.LIST = 0;
	SPIx->TXD.LIST = 0;
	SPIx->RXD.MAXCNT = 0;
	SPIx->TXD.MAXCNT = 0;
	SPIx->ENABLE = 7;
#else
	SPIx->ENABLE = 1;
#endif
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


