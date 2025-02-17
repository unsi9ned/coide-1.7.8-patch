/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */
#ifndef DWM1001_DEV_H
#define DWM1001_DEV_H

#ifdef __cplusplus
extern "C" {
#endif

#define LED_START      30
#define LED_1          30
#define LED_2          31
#define LED_3          22
#define LED_4          14
#define LED_STOP       22

#define BSP_LED_0      LED_1
#define BSP_LED_1      LED_2
#define BSP_LED_2      LED_3
#define BSP_LED_3      LED_3

#define BUTTON_START   2
#define SW_1           2
#define BUTTON_STOP    2
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

#define BSP_BUTTON_0   SW_1

// For nrf_dfu.h
#define BOOTLOADER_BUTTON   SW_1

#define BUTTONS_NUMBER 1
#define LEDS_NUMBER    4
#define BUTTONS_MASK   0x00000001

#define BSP_BUTTON_0_MASK (1<<BSP_BUTTON_0)

#define BUTTONS_LIST { SW_1 }
#define LEDS_LIST { LED_1, LED_2, LED_3, LED_4 }

#define BSP_LED_0_MASK    (1<<LED_1)
#define BSP_LED_1_MASK    (1<<LED_2)
#define BSP_LED_2_MASK    (1<<LED_3)
#define BSP_LED_3_MASK    (1<<LED_4)

#define LEDS_MASK      (BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK | BSP_LED_3_MASK)
/* all LEDs are lit when GPIO is low */
#define LEDS_INV_MASK  LEDS_MASK


#define RX_PIN_NUMBER  11
#define TX_PIN_NUMBER  5
#define CTS_PIN_NUMBER 0xFFFFFFFF
#define RTS_PIN_NUMBER 0xFFFFFFFF
#define HWFC           false

#define SER_CON_RX_PIN              11
#define SER_CON_TX_PIN              5
#define SER_CON_CTS_PIN             0xFFFFFFFF
#define SER_CON_RTS_PIN             0xFFFFFFFF

// Low frequency clock source to be used by the SoftDevice
#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_XTAL,            \
								 .rc_ctiv       = 0,                                \
								 .rc_temp_ctiv  = 0,                                \
								 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}

#ifdef __cplusplus
}
#endif

#endif
