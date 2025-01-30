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
#include "deca_sleep.h"
#include "deca_device_api.h"
#include "platform.h"

static uint32 sysTickCounter = 0;
static void (*decaIrqHandler)() = NULL;




int NVIC_DisableDECAIRQ(void)
{
#if 0
    EXTI_InitTypeDef EXTI_InitStructure;

    /* Configure EXTI line */
    EXTI_InitStructure.EXTI_Line = DECAIRQ_EXTI;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  //MPW3 IRQ polarity is high by default
    EXTI_InitStructure.EXTI_LineCmd = DECAIRQ_EXTI_NOIRQ;
    EXTI_Init(&EXTI_InitStructure);
#endif
    return 0;
}

/**
  * @brief  Checks whether the specified EXTI line is enabled or not.
  * @param  EXTI_Line: specifies the EXTI line to check.
  *   This parameter can be:
  *     @arg EXTI_Linex: External interrupt line x where x(0..19)
  * @retval The "enable" state of EXTI_Line (SET or RESET).
  */
#if 0
ITStatus EXTI_GetITEnStatus(uint32_t EXTI_Line)
{
    ITStatus bitstatus = RESET;
    uint32_t enablestatus = 0;
    /* Check the parameters */
    assert_param(IS_GET_EXTI_LINE(EXTI_Line));

    enablestatus =  EXTI->IMR & EXTI_Line;
    if (enablestatus != (uint32_t)RESET)
    {
        bitstatus = SET;
    }
    else
    {
        bitstatus = RESET;
    }
    return bitstatus;
}
#endif

// int RCC_Configuration(void)
// {
//  ErrorStatus HSEStartUpStatus;
//  RCC_ClocksTypeDef RCC_ClockFreq;

//  /* RCC system reset(for debug purpose) */
//  RCC_DeInit();

//  /* Enable HSE */
//  RCC_HSEConfig(RCC_HSE_ON);

//  /* Wait till HSE is ready */
//  HSEStartUpStatus = RCC_WaitForHSEStartUp();

//  if(HSEStartUpStatus != ERROR)
//  {
//      /* Enable Prefetch Buffer */
//      FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

//      /****************************************************************/
//      /* HSE= up to 25MHz (on EVB1000 is 12MHz),
//       * HCLK=72MHz, PCLK2=72MHz, PCLK1=36MHz                         */
//      /****************************************************************/
//      /* Flash 2 wait state */
//      FLASH_SetLatency(FLASH_Latency_2);
//      /* HCLK = SYSCLK */
//      RCC_HCLKConfig(RCC_SYSCLK_Div1);
//      /* PCLK2 = HCLK */
//      RCC_PCLK2Config(RCC_HCLK_Div1);
//      /* PCLK1 = HCLK/2 */
//      RCC_PCLK1Config(RCC_HCLK_Div2);
//      /*  ADCCLK = PCLK2/4 */
//      RCC_ADCCLKConfig(RCC_PCLK2_Div6);

//      /* Configure PLLs *********************************************************/
//      /* PLL2 configuration: PLL2CLK = (HSE / 4) * 8 = 24 MHz */
//      RCC_PREDIV2Config(RCC_PREDIV2_Div4);
//      RCC_PLL2Config(RCC_PLL2Mul_8);

//      /* Enable PLL2 */
//      RCC_PLL2Cmd(ENABLE);

//      /* Wait till PLL2 is ready */
//      while (RCC_GetFlagStatus(RCC_FLAG_PLL2RDY) == RESET){}

//      /* PLL1 configuration: PLLCLK = (PLL2 / 3) * 9 = 72 MHz */
//      RCC_PREDIV1Config(RCC_PREDIV1_Source_PLL2, RCC_PREDIV1_Div3);

//      RCC_PLLConfig(RCC_PLLSource_PREDIV1, RCC_PLLMul_9);

//      /* Enable PLL */
//      RCC_PLLCmd(ENABLE);

//      /* Wait till PLL is ready */
//      while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET){}

//      /* Select PLL as system clock source */
//      RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

//      /* Wait till PLL is used as system clock source */
//      while (RCC_GetSYSCLKSource() != 0x08){}
//  }

//  RCC_GetClocksFreq(&RCC_ClockFreq);

//  /* Enable SPI1 clock */
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

//  /* Enable SPI2 clock */
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

//  /* Enable GPIOs clocks */
//  RCC_APB2PeriphClockCmd(
//                      RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
//                      RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
//                      RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO,
//                      ENABLE);

//  return 0;
// }

int USART_Configuration(void)
{
#if 0
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    // USARTx setup
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USARTx, &USART_InitStructure);

    // USARTx TX pin setup
    GPIO_InitStructure.GPIO_Pin = USARTx_TX;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(USARTx_GPIO, &GPIO_InitStructure);

    // USARTx RX pin setup
    GPIO_InitStructure.GPIO_Pin = USARTx_RX;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(USARTx_GPIO, &GPIO_InitStructure);

    // Enable USARTx
    USART_Cmd(USARTx, ENABLE);
#endif
    return 0;
}

void SPI_ChangeRate(uint16_t scalingfactor)
{
#if 0
    uint16_t tmpreg = 0;

    /* Get the SPIx CR1 value */
    tmpreg = SPIx->CR1;

    /*clear the scaling bits*/
    tmpreg &= 0xFFC7;

    /*set the scaling bits*/
    tmpreg |= scalingfactor;

    /* Write to SPIx CR1 */
    SPIx->CR1 = tmpreg;
#endif
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
#if 0
    SPI_ChangeRate(SPI_BaudRatePrescaler_32);
#endif
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
#if 0
    SPI_ChangeRate(SPI_BaudRatePrescaler_4);
#endif
}

void SPI_ConfigFastRate(uint16_t scalingfactor)
{
#if 0
    SPI_InitTypeDef SPI_InitStructure;

    SPI_I2S_DeInit(SPIx);

    // SPIx Mode setup
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;   //
    //SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; //
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    //SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; //
    //SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = scalingfactor; //sets BR[2:0] bits - baudrate in SPI_CR1 reg bits 4-6
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;

    SPI_Init(SPIx, &SPI_InitStructure);

    // Enable SPIx
    SPI_Cmd(SPIx, ENABLE);
#endif
}

int SPI_Configuration(void)
{
#if 0
    SPI_InitTypeDef SPI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    SPI_I2S_DeInit(SPIx);

    // SPIx Mode setup
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;   //
    //SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; //
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    //SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; //
    //SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    //SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; //sets BR[2:0] bits - baudrate in SPI_CR1 reg bits 4-6
    SPI_InitStructure.SPI_BaudRatePrescaler = SPIx_PRESCALER;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;

    SPI_Init(SPIx, &SPI_InitStructure);

    // SPIx SCK and MOSI pin setup
    GPIO_InitStructure.GPIO_Pin = SPIx_SCK | SPIx_MOSI;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(SPIx_GPIO, &GPIO_InitStructure);

    // SPIx MISO pin setup
    GPIO_InitStructure.GPIO_Pin = SPIx_MISO;
    //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;

    GPIO_Init(SPIx_GPIO, &GPIO_InitStructure);

    // SPIx CS pin setup
    GPIO_InitStructure.GPIO_Pin = SPIx_CS;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(SPIx_CS_GPIO, &GPIO_InitStructure);

    // Disable SPIx SS Output
    SPI_SSOutputCmd(SPIx, DISABLE);

    // Enable SPIx
    SPI_Cmd(SPIx, ENABLE);

    // Set CS high
    GPIO_SetBits(SPIx_CS_GPIO, SPIx_CS);

#endif
    return 0;
}


int SPI2_Configuration(void)
{
#if 0
    SPI_InitTypeDef SPI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    SPI_I2S_DeInit(SPIy);

    // SPIy Mode setup
    //SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    //SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;     //
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; //
    //SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; //
    //SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPIy_PRESCALER;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;

    SPI_Init(SPIy, &SPI_InitStructure);

    // SPIy SCK and MOSI pin setup
    GPIO_InitStructure.GPIO_Pin = SPIy_SCK | SPIy_MOSI;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(SPIy_GPIO, &GPIO_InitStructure);

    // SPIy MISO pin setup
    GPIO_InitStructure.GPIO_Pin = SPIy_MISO;
    //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;

    GPIO_Init(SPIy_GPIO, &GPIO_InitStructure);

    // SPIy CS pin setup
    GPIO_InitStructure.GPIO_Pin = SPIy_CS;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(SPIy_CS_GPIO, &GPIO_InitStructure);

    // Disable SPIy SS Output
    SPI_SSOutputCmd(SPIy, DISABLE);

    // Enable SPIy
    SPI_Cmd(SPIy, ENABLE);

    // Set CS high
    GPIO_SetBits(SPIy_CS_GPIO, SPIy_CS);

    // LCD_RS pin setup
    GPIO_InitStructure.GPIO_Pin = LCD_RS;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(SPIy_GPIO, &GPIO_InitStructure);

    // LCD_RW pin setup
    GPIO_InitStructure.GPIO_Pin = LCD_RW;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(SPIy_GPIO, &GPIO_InitStructure);
#endif
    return 0;
}

void reset_DW1000(void)
{
#if 0
    GPIO_InitTypeDef GPIO_InitStructure;

    // Enable GPIO used for DW1000 reset
    GPIO_InitStructure.GPIO_Pin = DW1000_RSTn;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DW1000_RSTn_GPIO, &GPIO_InitStructure);

    //drive the RSTn pin low
    GPIO_ResetBits(DW1000_RSTn_GPIO, DW1000_RSTn);

    //put the pin back to tri-state ... as input
    GPIO_InitStructure.GPIO_Pin = DW1000_RSTn;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DW1000_RSTn_GPIO, &GPIO_InitStructure);

    deca_sleep(2);
#endif
}


void setup_DW1000RSTnIRQ(int enable)
{
#if 0
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    if(enable)
    {
        // Enable GPIO used as DECA IRQ for interrupt
        GPIO_InitStructure.GPIO_Pin = DECARSTIRQ;
        //GPIO_InitStructure.GPIO_Mode =    GPIO_Mode_IPD;  //IRQ pin should be Pull Down to prevent unnecessary EXT IRQ while DW1000 goes to sleep mode
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
        GPIO_Init(DECARSTIRQ_GPIO, &GPIO_InitStructure);

        /* Connect EXTI Line to GPIO Pin */
        GPIO_EXTILineConfig(DECARSTIRQ_EXTI_PORT, DECARSTIRQ_EXTI_PIN);

        /* Configure EXTI line */
        EXTI_InitStructure.EXTI_Line = DECARSTIRQ_EXTI;
        EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
        EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  //MP IRQ polarity is high by default
        EXTI_InitStructure.EXTI_LineCmd = ENABLE;
        EXTI_Init(&EXTI_InitStructure);

        /* Set NVIC Grouping to 16 groups of interrupt without sub-grouping */
        NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

        /* Enable and set EXTI Interrupt to the lowest priority */
        NVIC_InitStructure.NVIC_IRQChannel = DECARSTIRQ_EXTI_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 15;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

        NVIC_Init(&NVIC_InitStructure);
    }
    else
    {
        //put the pin back to tri-state ... as input
        GPIO_InitStructure.GPIO_Pin = DW1000_RSTn;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(DW1000_RSTn_GPIO, &GPIO_InitStructure);

        /* Configure EXTI line */
        EXTI_InitStructure.EXTI_Line = DECARSTIRQ_EXTI;
        EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
        EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  //MP IRQ polarity is high by default
        EXTI_InitStructure.EXTI_LineCmd = DISABLE;
        EXTI_Init(&EXTI_InitStructure);
    }
#endif
}

#if 0
void EXTI0_IRQHandler(void)
{
	if(EXTI_GetITStatus(DECARSTIRQ_EXTI) != RESET)
	{
		dwt_isr();

		/* Clear the  EXTI line 0 pending bit */
		EXTI_ClearITPendingBit(DECARSTIRQ_EXTI);
	}
}
#endif


int ETH_GPIOConfigure(void)
{
#if 0
    GPIO_InitTypeDef GPIO_InitStructure;

    /* ETHERNET pins configuration */
    /* AF Output Push Pull:
    - ETH_MII_MDIO / ETH_RMII_MDIO: PA2
    - ETH_MII_MDC / ETH_RMII_MDC: PC1
    - ETH_MII_TXD2: PC2
    - ETH_MII_TX_EN / ETH_RMII_TX_EN: PB11
    - ETH_MII_TXD0 / ETH_RMII_TXD0: PB12
    - ETH_MII_TXD1 / ETH_RMII_TXD1: PB13
    - ETH_MII_PPS_OUT / ETH_RMII_PPS_OUT: PB5
    - ETH_MII_TXD3: PB8 */

    /* Configure PA2 as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Configure PC1 and PC2 as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* Configure PB5, PB8, PB11, PB12 and PB13 as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_8 | GPIO_Pin_11 |
                                  GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /**************************************************************/
    /*               For Remapped Ethernet pins                   */
    /*************************************************************/
    /* Input (Reset Value):
    - ETH_MII_CRS CRS: PA0
    - ETH_MII_RX_CLK / ETH_RMII_REF_CLK: PA1
    - ETH_MII_COL: PA3
    - ETH_MII_RX_DV / ETH_RMII_CRS_DV: PD8
    - ETH_MII_TX_CLK: PC3
    - ETH_MII_RXD0 / ETH_RMII_RXD0: PD9
    - ETH_MII_RXD1 / ETH_RMII_RXD1: PD10
    - ETH_MII_RXD2: PD11
    - ETH_MII_RXD3: PD12
    - ETH_MII_RX_ER: PB10 */

    /* Configure PA0, PA1 and PA3 as input */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Configure PB10 as input */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Configure PC3 as input */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* Configure PD8, PD9, PD10, PD11 and PD12 as input */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &GPIO_InitStructure); /**/



    /* MCO pin configuration------------------------------------------------- */
    /* Configure MCO (PA8) as alternate function push-pull */
    //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    //GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif

    return 0;
}

#if 0
int is_button_low(uint16_t GPIOpin)
{
    int result = 1;

    if (GPIO_ReadInputDataBit(TA_BOOT1_GPIO, TA_BOOT1))
        result = 0;

    return result;
}

//when switch (S1) is 'on' the pin is low
int is_switch_on(uint16_t GPIOpin)
{
    int result = 1;

    if(GPIOpin == TA_SW1_5)
    {
        if (GPIO_ReadInputDataBit(TA_SW1_GPIOA, GPIOpin))
            result = 0;
    }
    else
    {
        if (GPIO_ReadInputDataBit(TA_SW1_GPIOC, GPIOpin))
            result = 0;
    }

    return result;
}

void led_off (led_t led)
{
    switch (led)
    {
        case LED_PC6:
            GPIO_ResetBits(GPIOC, GPIO_Pin_6);
            break;
        case LED_PC7:
            GPIO_ResetBits(GPIOC, GPIO_Pin_7);
            break;
        case LED_PC8:
            GPIO_ResetBits(GPIOC, GPIO_Pin_8);
            break;
        case LED_PC9:
            GPIO_ResetBits(GPIOC, GPIO_Pin_9);
            break;
        case LED_ALL:
            GPIO_ResetBits(GPIOC, GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_6 | GPIO_Pin_7);
            break;
        default:
            // do nothing for undefined led number
            break;
    }
}

void led_on (led_t led)
{
    switch (led)
    {
        case LED_PC6:
            GPIO_SetBits(GPIOC, GPIO_Pin_6);
            break;
        case LED_PC7:
            GPIO_SetBits(GPIOC, GPIO_Pin_7);
            break;
        case LED_PC8:
            GPIO_SetBits(GPIOC, GPIO_Pin_8);
            break;
        case LED_PC9:
            GPIO_SetBits(GPIOC, GPIO_Pin_9);
            break;
        case LED_ALL:
            GPIO_SetBits(GPIOC, GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_6 | GPIO_Pin_7);
            break;
        default:
            // do nothing for undefined led number
            break;
    }
}
#endif

/**
  * @brief  Configures COM port.
  * @param  USART_InitStruct: pointer to a USART_InitTypeDef structure that
  *   contains the configuration information for the specified USART peripheral.
  * @retval None
  */
//#include "stm32_eval.h"
#if 0
void usartinit(void)
{
	
    USART_InitTypeDef USART_InitStructure;
    //GPIO_InitTypeDef GPIO_InitStructure;

    /* USARTx configured as follow:
          - BaudRate = 115200 baud
          - Word Length = 8 Bits
          - One Stop Bit
          - No parity
          - Hardware flow control disabled (RTS and CTS signals)
          - Receive and transmit enabled
    */
   USART_InitStructure.USART_BaudRate = 115200;
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
   USART_InitStructure.USART_StopBits = USART_StopBits_1;
   USART_InitStructure.USART_Parity = USART_Parity_No;
   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

   USART_Init(USART1, &USART_InitStructure);
   USART_Cmd(USART1, ENABLE);
	
}
#endif

void PrintChar(char c)
{
#if 0
	USART_SendData(USARTx, c);

	/* Loop until the end of transmission */
	while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
	{
	}
#endif
}

//void USART_puts(uint8_t *s,uint8_t len)
//{
//    int i;
//    for(i=0; i<len; i++)
//    {
//        putchar(s[i]);
//    }
//}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */



/*
void USART_puts(const char *s)
{
    int i;
    for(i=0; s[i]!=0; i++)
    {
        USART_putc(s[i]);
    }
}*/




int is_IRQ_enabled(void)
{
#if 0
    return ((   NVIC->ISER[((uint32_t)(DECAIRQ_EXTI_IRQn) >> 5)]
                & (uint32_t)0x01 << (DECAIRQ_EXTI_IRQn & (uint8_t)0x1F)  ) ? 1 : 0) ;
#endif
    return 0;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn LCD_configuration()
 *
 * @brief Initialise LCD screen.
 *
 * @param none
 *
 * @return none

static void LCD_Configuration(void)
{
    unsigned char initseq[9] = { 0x39, 0x14, 0x55, 0x6D, 0x78, 0x38, 0x0C, 0x01, 0x06 };
    unsigned char command = 0x0;

    // Write initialisation sequence.
    writetoLCD(9, 0, initseq);
    deca_sleep(10);

    // Return cursor home and clear screen.
    command = 0x2;
    writetoLCD(1, 0, &command);
    command = 0x1;
    writetoLCD(1, 0, &command);
} */

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
    //spi_init();//for dwm1000

    // Initialise SPI2 peripheral for LCD control
	/*
    SPI2_Configuration();
    port_LCD_RS_clear();
    port_LCD_RW_clear();
	*/

    // Wait for LCD to power on.
    //deca_sleep(100);
}

//------------------------------------------------------------------------------
// Инициализация портов ввода/вывода
//------------------------------------------------------------------------------
static void gpio_init()
{
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
	uint32 inMask = DW1000_IRQ_EIC_MASK;

	// Enable wakeup capability on pin in case being used during sleep
	EIC->WAKEUP.reg |= (1 << in);

	// Store interrupts to service in order of when they were attached
	// to allow for first come first serve handler
	uint32_t current = 0;

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

	// Enable GPIO used as DECA IRQ for interrupt
	uint32 portGrp = DW1000_IRQ_PIN >> 5;
	uint32 pin = DW1000_IRQ_PIN % 32;
	uint32 pinMask = DW1000_IRQ_MASK;

	// Set pin to input mode
	PORT->Group[portGrp].PINCFG[pin].reg = (uint8)(PORT_PINCFG_INEN);
	PORT->Group[portGrp].DIRCLR.reg = pinMask;
	port_EnableEXT_IRQ();
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
// Подключение пина к периферие
//------------------------------------------------------------------------------
static void gpio_periph_enable(uint32 p, uint8 muxLine)
{
	uint32 portGrp = p >> 5;
	uint32 pin = p % 32;
	uint32 pinMask = (1 << pin);

	uint8 pmux = PORT->Group[portGrp].PMUX[pin >> 1].reg & (0xF << ((pin & 1) * 4));
	pmux |= (muxLine & 0xF) << ((pin & 1) * 4);
	PORT->Group[portGrp].PMUX[pin >> 1].reg = pmux;
	// Enable port mux
	PORT->Group[portGrp].PINCFG[pin].reg |= PORT_PINCFG_PMUXEN | PORT_PINCFG_DRVSTR ;
}

//------------------------------------------------------------------------------
// Инициализация отладочного UART
//------------------------------------------------------------------------------
static void usart_init()
{
	//TODO SERCOM может также находиться на линии D
	gpio_periph_enable(USARTx_RX, PORT_PMUX_PMUXE_C_Val);
	gpio_periph_enable(USARTx_TX, PORT_PMUX_PMUXE_C_Val);

	// Clock SERCOM for Serial
	PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0 |
						PM_APBCMASK_SERCOM1 |
						PM_APBCMASK_SERCOM2 |
						PM_APBCMASK_SERCOM3 |
						PM_APBCMASK_SERCOM4 |
						PM_APBCMASK_SERCOM5 ;

	uint32 sercomIdx;
	uint8 clockId;
	IRQn_Type IdNvic;

	sercomIdx = (USARTx - SERCOM0) / 0x400;
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


