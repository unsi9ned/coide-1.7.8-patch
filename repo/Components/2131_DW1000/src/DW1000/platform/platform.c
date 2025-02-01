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
#include "deca_sleep.h"
#include "deca_device_api.h"
#include "platform.h"

typedef enum
{
	IO_INPUT          = (0x0),
	IO_OUTPUT         = (0x1),
	IO_INPUT_PULLUP   = (0x2),
	IO_INPUT_PULLDOWN = (0x3)
}
tGpioMode;

static uint32 sysTickCounter = 0;
static void (*decaIrqHandler)() = NULL;

//------------------------------------------------------------------------------
// Установка режима работы пина
//------------------------------------------------------------------------------
static void gpio_set_mode( uint32_t ulPin, tGpioMode ulMode )
{
	uint32 port = ulPin >> 5;
	uint32_t pin = ulPin % 32;
	uint32_t pinMask = (1ul << pin);

	// Set pin mode according to chapter '22.6.3 I/O Pin Configuration'
	switch(ulMode)
	{
		case IO_INPUT:
			// Set pin to input mode
			PORT->Group[port].PINCFG[pin].reg = (uint8_t)(PORT_PINCFG_INEN);
			PORT->Group[port].DIRCLR.reg = pinMask;
			break;

		case IO_INPUT_PULLUP:
			// Set pin to input mode with pull-up resistor enabled
			PORT->Group[port].PINCFG[pin].reg = (uint8_t)(PORT_PINCFG_INEN | PORT_PINCFG_PULLEN);
			PORT->Group[port].DIRCLR.reg = pinMask;

			// Enable pull level (cf '22.6.3.2 Input Configuration' and '22.8.7 Data Output Value Set')
			PORT->Group[port].OUTSET.reg = pinMask;
			break;

		case IO_INPUT_PULLDOWN:
			// Set pin to input mode with pull-down resistor enabled
			PORT->Group[port].PINCFG[pin].reg = (uint8_t)(PORT_PINCFG_INEN | PORT_PINCFG_PULLEN);
			PORT->Group[port].DIRCLR.reg = pinMask;

			// Enable pull level (cf '22.6.3.2 Input Configuration' and '22.8.6 Data Output Value Clear')
			PORT->Group[port].OUTCLR.reg = pinMask;
			break;

		case IO_OUTPUT:
			// enable input, to support reading back values, with pullups disabled
			PORT->Group[port].PINCFG[pin].reg = (uint8_t)(PORT_PINCFG_INEN | PORT_PINCFG_DRVSTR);

			// Set pin to output mode
			PORT->Group[port].DIRSET.reg = pinMask;
			break;

		default:
			// do nothing
			break;
	}
}

//------------------------------------------------------------------------------
// Подключение пина к периферии
//------------------------------------------------------------------------------
static void gpio_periph_enable(uint32 p, uint8 muxLine)
{
	uint32 portGrp = p >> 5;
	uint32 pin = p % 32;

	uint8 pmux = PORT->Group[portGrp].PMUX[pin >> 1].reg;
	pmux |= (muxLine & 0xF) << ((pin & 1) * 4);
	PORT->Group[portGrp].PMUX[pin >> 1].reg = pmux;
	// Enable port mux
	PORT->Group[portGrp].PINCFG[pin].reg |= PORT_PINCFG_PMUXEN | PORT_PINCFG_DRVSTR ;
}

//------------------------------------------------------------------------------
// Установка выхода в состояние 0 или 1
//------------------------------------------------------------------------------
void gpio_write(uint32_t ulPin, uint32_t ulVal)
{
	uint32 port = ulPin >> 5;
	uint32_t pin = ulPin % 32;
	uint32_t pinMask = (1ul << pin);

	if((PORT->Group[port].DIRSET.reg & pinMask) == 0)
	{
		// the pin is not an output, disable pull-up if val is LOW, otherwise enable pull-up
		PORT->Group[port].PINCFG[pin].bit.PULLEN = ((ulVal == 0) ? 0 : 1);
	}

	switch(ulVal)
	{
		case 0:
			PORT->Group[port].OUTCLR.reg = pinMask;
			break;

		default:
			PORT->Group[port].OUTSET.reg = pinMask;
			break;
	}

	return;
}

//------------------------------------------------------------------------------
// Чтение состояние вывода
//------------------------------------------------------------------------------
int gpio_read(uint32_t ulPin)
{
	uint32 port = ulPin >> 5;
	uint32_t pin = ulPin % 32;
	uint32_t pinMask = (1ul << pin);

	if((PORT->Group[port].IN.reg & pinMask) != 0)
	{
		return 1;
	}

	return 0;
}

//------------------------------------------------------------------------------
// Инициализация портов ввода/вывода
//------------------------------------------------------------------------------
static void gpio_init()
{
	// Enable GPIO used as DECA IRQ for interrupt
	gpio_set_mode(DW1000_IRQ_PIN, IO_INPUT);
	gpio_set_mode(DW1000_RSTn_PIN, IO_INPUT);
	gpio_set_mode(USARTx_RX, IO_INPUT_PULLUP);
	gpio_set_mode(USARTx_TX, IO_OUTPUT);
	gpio_set_mode(SPIx_CS_PIN, IO_OUTPUT);
	gpio_write(SPIx_CS_PIN, 1);
	gpio_set_mode(SPIx_MOSI_PIN, IO_OUTPUT);
	gpio_set_mode(SPIx_SCK_PIN, IO_OUTPUT);
	gpio_set_mode(SPIx_MISO_PIN, IO_INPUT_PULLUP);
}

//------------------------------------------------------------------------------
// Настройка порта светодиода
//------------------------------------------------------------------------------
static int gpio_led_init()
{
	gpio_set_mode(USER_LED_PIN, IO_OUTPUT);
	gpio_write(USER_LED_PIN, USER_LED_POLARITY ? 1 : 0);
	return 1;
}

//------------------------------------------------------------------------------
// Зажечь или погасить светодиод
//------------------------------------------------------------------------------
void port_led_write(int enable)
{
	static int init = 0;
	init = !init ? gpio_led_init() : 1;
	gpio_write(USER_LED_PIN, enable & 1);
}

//------------------------------------------------------------------------------
// Изменить состояние светодиода на противоположное
//------------------------------------------------------------------------------
void port_led_toggle()
{
	static int init = 0;
	init = !init ? gpio_led_init() : 1;
	gpio_write(USER_LED_PIN, !gpio_read(USER_LED_PIN));
}

//------------------------------------------------------------------------------
// Аппаратный сброс модуля
//------------------------------------------------------------------------------
void reset_DW1000(void)
{
	gpio_set_mode(DW1000_RSTn_PIN, IO_OUTPUT);
	gpio_write(DW1000_RSTn_PIN, 0);
	deca_sleep(2);
	gpio_set_mode(DW1000_RSTn_PIN, IO_INPUT);
}

//------------------------------------------------------------------------------
// Отключить прерывания по пину IRQ
//------------------------------------------------------------------------------
void port_DisableEXT_IRQ()
{
	uint32 in = DW1000_IRQ_EIC_IN;

	if(in == PIN_PA08A_EIC_NMI)
	{
		EIC->NMICTRL.bit.NMISENSE = 0; // Turn off detection
	}
	else
	{
		EIC->INTENCLR.reg = EIC_INTENCLR_EXTINT(1 << in);

		// Disable wakeup capability on pin during sleep
		EIC->WAKEUP.reg &= ~(1 << in);
	}
}

//------------------------------------------------------------------------------
// Включить прерывания по пину IRQ
//------------------------------------------------------------------------------
void port_EnableEXT_IRQ()
{
	uint32_t config;
	uint32_t pos;
	uint32 in = DW1000_IRQ_EIC_IN;

	// Enable wakeup capability on pin in case being used during sleep
	EIC->WAKEUP.reg |= (1 << in);

	// Look for right CONFIG register to be addressed
	if(in > 7)
	{
		config = 1;
		pos = (in - 8) << 2;
	}
	else
	{
		config = 0;
		pos = in << 2;
	}

	// Reset sense mode, important when changing trigger mode during runtime
	EIC->CONFIG[config].reg &=~ (EIC_CONFIG_SENSE0_Msk << pos);
	EIC->CONFIG[config].reg |= EIC_CONFIG_SENSE0_RISE_Val << pos;

	// Enable the interrupt
	EIC->INTENSET.reg = EIC_INTENSET_EXTINT(1 << in);
}

//------------------------------------------------------------------------------
// Определяет статус прерываний по IRQ пину.
// 1 - включено, 0 - отключено
//------------------------------------------------------------------------------
decaIrqStatus_t port_GetEXT_IRQStatus()
{
	decaIrqStatus_t bitstatus;
	uint32 in = DW1000_IRQ_EIC_IN;

	if(in != PIN_PA08A_EIC_NMI)
	{
		bitstatus = (EIC->INTENSET.reg & EIC_INTENCLR_EXTINT(1 << in)) ? 1 : 0;
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
void __attribute__ ((weak)) EIC_Handler(void)
{
	if((EIC->INTFLAG.reg & DW1000_IRQ_EIC_MASK) && decaIrqHandler)
	{
		decaIrqHandler();
	}
	// Clear the interrupt
	EIC->INTFLAG.reg = 0xFFFF;
}

//------------------------------------------------------------------------------
// Настройка прерываний периферии
//------------------------------------------------------------------------------
static void interrupt_init()
{
	NVIC_DisableIRQ(EIC_IRQn);
	NVIC_ClearPendingIRQ(EIC_IRQn);
	NVIC_SetPriority(EIC_IRQn, 0);
	NVIC_EnableIRQ(EIC_IRQn);

	// Enable GCLK for IEC (External Interrupt Controller)
	GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_EIC_Val));

	// Enable EIC
	EIC->CTRL.bit.ENABLE = 1;
	while (EIC->STATUS.bit.SYNCBUSY == 1) { }

	gpio_periph_enable(DW1000_IRQ_PIN, DW1000_IRQ_MUX);
	port_EnableEXT_IRQ();
}

//------------------------------------------------------------------------------
// Инициализация линии прерываний для SERCOM
//------------------------------------------------------------------------------
static void sercom_init_nvic_clock(Sercom   * sercom)
{
	uint32 sercomIdx;
	uint8 clockId;
	IRQn_Type IdNvic;

	sercomIdx = ((uint32)sercom - (uint32)SERCOM0) / 0x400UL;
	clockId = GCLK_CLKCTRL_ID_SERCOM0_CORE_Val + sercomIdx;
	IdNvic = SERCOM0_IRQn + sercomIdx;

	// Setting NVIC
	NVIC_ClearPendingIRQ(IdNvic);
	NVIC_SetPriority(IdNvic, ((1<<__NVIC_PRIO_BITS) - 1));
	NVIC_EnableIRQ(IdNvic);

	// Setting clock
	// Generic Clock 0 (SERCOMx)
	// Generic Clock Generator 0 is source
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID( clockId ) |
						GCLK_CLKCTRL_GEN_GCLK0 |
						GCLK_CLKCTRL_CLKEN;

	// Wait for synchronization
	while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
}

//------------------------------------------------------------------------------
// Обработчик прерываний системного таймера
//------------------------------------------------------------------------------
void __attribute__ ((weak)) SysTick_Handler(void)
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
	//TODO SERCOM может также находиться на линии D
	gpio_periph_enable(USARTx_RX, PORT_PMUX_PMUXE_C_Val);
	gpio_periph_enable(USARTx_TX, PORT_PMUX_PMUXE_C_Val);

	sercom_init_nvic_clock(USARTx);

	// Start the Software Reset
	USARTx->USART.CTRLA.bit.SWRST = 1;

	while (USARTx->USART.CTRLA.bit.SWRST || USARTx->USART.SYNCBUSY.bit.SWRST)
	{
		// Wait for both bits Software Reset from CTRLA and SYNCBUSY coming back to 0
	}

	//Setting the CTRLA register
	//SAMPLE_RATE_x16 = 0x1
	USARTx->USART.CTRLA.reg = SERCOM_USART_CTRLA_MODE(SERCOM_USART_CTRLA_MODE_USART_INT_CLK_Val) |
							  SERCOM_USART_CTRLA_SAMPR(0x1);

	//Setting the Interrupt register
#if 0
	USARTx->USART.INTENSET.reg = SERCOM_USART_INTENSET_RXC |  //Received complete
								 SERCOM_USART_INTENSET_ERROR; //All others errors
#endif

	uint16_t sampleRateValue = 16;

	// Asynchronous fractional mode (Table 24-2 in datasheet)
	//   BAUD = fref / (sampleRateValue * fbaud)
	// (multiply by 8, to calculate fractional piece)
	uint32_t baudTimes8 = (SystemCoreClock * 8) / (sampleRateValue * 115200UL);

	USARTx->USART.BAUD.FRAC.FP   = (baudTimes8 % 8);
	USARTx->USART.BAUD.FRAC.BAUD = (baudTimes8 / 8);

	// Setting the CTRLA register
	// No parity, LSB
	USARTx->USART.CTRLA.reg |= SERCOM_USART_CTRLA_FORM(0) |
							   SERCOM_USART_CTRLA_DORD;

	//Setting the CTRLB register
	//Char size = 8, 1 stop bit,
	USARTx->USART.CTRLB.reg |= SERCOM_USART_CTRLB_CHSIZE(0) |
								0 << SERCOM_USART_CTRLB_SBMODE_Pos |
								0 << SERCOM_USART_CTRLB_PMODE_Pos; //If no parity use default value

	//Setting the CTRLA register
	//TODO Пады могут быть другими, исправить
	USARTx->USART.CTRLA.reg |= SERCOM_USART_CTRLA_TXPO(0x1) |
							 SERCOM_USART_CTRLA_RXPO(0x3);

	// Enable Transceiver and Receiver
	USARTx->USART.CTRLB.reg |= SERCOM_USART_CTRLB_TXEN | SERCOM_USART_CTRLB_RXEN ;

	//Setting  the enable bit to 1
	USARTx->USART.CTRLA.bit.ENABLE = 0x1u;

	//Wait for then enable bit from SYNCBUSY is equal to 0;
	while(USARTx->USART.SYNCBUSY.bit.ENABLE);
}

//------------------------------------------------------------------------------
// Вывод символа через UART. Используется в printf
//------------------------------------------------------------------------------
void UART_PutChar(char c)
{
	while(!USARTx->USART.INTFLAG.bit.DRE);
	USARTx->USART.DATA.reg = (uint16_t)c;
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
	// -------------------- PIO init -------------------------------------------
	gpio_periph_enable(SPIx_MISO_PIN, SPIx_MISO_MUX);
	gpio_periph_enable(SPIx_MOSI_PIN, SPIx_MOSI_MUX);
	gpio_periph_enable(SPIx_SCK_PIN, SPIx_SCK_MUX);

	// -------------------- SPI Disable ----------------------------------------
	//Setting the enable bit to 0
	SPIx->SPI.CTRLA.bit.ENABLE = 0;

	while (SPIx->SPI.SYNCBUSY.bit.ENABLE)
	{
		//Waiting then enable bit from SYNCBUSY is equal to 0;
	}

	// -------------------- SPI Reset ------------------------------------------
	//Setting the Software Reset bit to 1
	SPIx->SPI.CTRLA.bit.SWRST = 1;

	//Wait both bits Software Reset from CTRLA and SYNCBUSY are equal to 0
	while(SPIx->SPI.CTRLA.bit.SWRST || SPIx->SPI.SYNCBUSY.bit.SWRST);

	// -------------------- SPI NVIC init---------------------------------------
	sercom_init_nvic_clock(SPIx);

	//Setting the CTRLA register
	SPIx->SPI.CTRLA.reg = SERCOM_SPI_CTRLA_MODE_SPI_MASTER |
						  SERCOM_SPI_CTRLA_DOPO(SPIx_MOSI_SCK_PAD) |
						  SERCOM_SPI_CTRLA_DIPO(SPIx_MISO_PAD) |
						  0 << SERCOM_SPI_CTRLA_DORD_Pos; //MSB First

	//Setting the CTRLB register
	SPIx->SPI.CTRLB.reg = SERCOM_SPI_CTRLB_CHSIZE(0) |  //8 bit
						  SERCOM_SPI_CTRLB_RXEN;        //Active the SPI receiver.

	while( SPIx->SPI.SYNCBUSY.bit.CTRLB == 1 );

	// -------------------- SPI Clock mode -------------------------------------
	//Setting the CTRLA register
	//Rising, sample Falling, change
	SPIx->SPI.CTRLA.reg |= ( 0 << SERCOM_SPI_CTRLA_CPHA_Pos ) |
						   ( 0 << SERCOM_SPI_CTRLA_CPOL_Pos );

	//Synchronous arithmetic
	uint16 b = SystemCoreClock / (2 * SPIx_LOW_RATE);

	// Don't -1 on baud calc if already at 0
	SPIx->SPI.BAUD.reg = (b > 0) ? b - 1 : 0;

	// -------------------- SPI Enable -----------------------------------------
	//Setting the enable bit to 1
	SPIx->SPI.CTRLA.bit.ENABLE = 1;

	while (SPIx->SPI.SYNCBUSY.bit.ENABLE)
	{
		//Waiting then enable bit from SYNCBUSY is equal to 0;
	}
}

//------------------------------------------------------------------------------
// Изменение скорости передачи данных SPI
//------------------------------------------------------------------------------
static void spi_change_rate(uint32 scalingfactor)
{
	//Setting the enable bit to 0
	SPIx->SPI.CTRLA.bit.ENABLE = 0;

	while (SPIx->SPI.SYNCBUSY.bit.ENABLE)
	{
		//Waiting then enable bit from SYNCBUSY is equal to 0;
	}

	//Synchronous arithmetic
	uint16 b = SystemCoreClock / (2 * scalingfactor);

	// Don't -1 on baud calc if already at 0
	SPIx->SPI.BAUD.reg = (b > 0) ? b - 1 : 0;

	//Setting the enable bit to 1
	SPIx->SPI.CTRLA.bit.ENABLE = 1;

	while (SPIx->SPI.SYNCBUSY.bit.ENABLE)
	{
		//Waiting then enable bit from SYNCBUSY is equal to 0;
	}
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
	// Clock SERCOM for Serial
	PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0 |
						PM_APBCMASK_SERCOM1 |
						PM_APBCMASK_SERCOM2 |
						PM_APBCMASK_SERCOM3 |
						PM_APBCMASK_SERCOM4 |
						PM_APBCMASK_SERCOM5 ;

	gpio_init();
	interrupt_init();
	systick_init();
	spi_peripheral_init();
	usart_init();
}


