/* (C) 2018 Microchip Technology Inc. and its subsidiaries.
 * Subject to your compliance with these terms, you may use Microchip software
 *  and any derivatives exclusively with Microchip products. It is your 
 * responsibility to comply with third party license terms applicable to your
 *  use of third party software (including open source software) that may 
 * accompany Microchip software.
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE.
 * 
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, 
 * IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 */ 
#include "sam.h"


#define SERCOM_ID                      4

#define __SERCOM_GCLK_ID_CORE__(x)     SERCOM##x##_GCLK_ID_CORE
#define __SERCOM_GCLK_ID_CORE(x)     __SERCOM_GCLK_ID_CORE__(x)
#define __SERCOM__(x)                  SERCOM##x
#define __SERCOM(x)                  __SERCOM__(x)

#define SPI_SERCOM_GCLK_ID_CORE      __SERCOM_GCLK_ID_CORE(SERCOM_ID)
#define SPI_SERCOM_CLK_GEN             0
#define SPI_SERCOM                   __SERCOM(SERCOM_ID)
#define SPI_CLK_FREQ                   48000000UL
#define SPI_BAUD                       2000000UL


#define GPIO_PIN_SS						PIN_PA08
#define GPIO_PIN_MISO					PIN_PA12
#define GPIO_PIN_MOSI					PIN_PB10
#define GPIO_PIN_SCK					PIN_PB11

#define SPI_MOSI_PORT_PMUX				MUX_PB10D_SERCOM4_PAD2
#define SPI_MISO_PORT_PMUX				MUX_PA12D_SERCOM4_PAD0
#define SPI_SCK_PORT_PMUX				MUX_PB11D_SERCOM4_PAD3

#define SPI_DOPO                        (1) //PAD[2,3]
#define SPI_DIPO                        (0) //PAD[0]

#define GPIO_SS_LOW()					PORT->Group[GPIO_PIN_SS >> 5].OUTCLR.reg = (1 << (GPIO_PIN_SS % 32))
#define GPIO_SS_HIGH()					PORT->Group[GPIO_PIN_SS >> 5].OUTSET.reg = (1 << (GPIO_PIN_SS % 32))

void SPI_init(void);
uint8_t spiSend(uint8_t data);
