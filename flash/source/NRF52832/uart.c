#include <stdint.h>

#ifdef __cplusplus
  #define   __I     volatile             /*!< Defines 'read only' permissions */
#else
  #define   __I     volatile const       /*!< Defines 'read only' permissions */
#endif
#define     __O     volatile             /*!< Defines 'write only' permissions */
#define     __IO    volatile             /*!< Defines 'read / write' permissions */

/* ================================================================================ */
/* ================                      UART                      ================ */
/* ================================================================================ */


/**
  * @brief Universal Asynchronous Receiver/Transmitter (UART)
  */

typedef struct {                                    /*!< UART Structure                                                        */
  __O  uint32_t  TASKS_STARTRX;                     /*!< Start UART receiver                                                   */
  __O  uint32_t  TASKS_STOPRX;                      /*!< Stop UART receiver                                                    */
  __O  uint32_t  TASKS_STARTTX;                     /*!< Start UART transmitter                                                */
  __O  uint32_t  TASKS_STOPTX;                      /*!< Stop UART transmitter                                                 */
  __I  uint32_t  RESERVED0[3];
  __O  uint32_t  TASKS_SUSPEND;                     /*!< Suspend UART                                                          */
  __I  uint32_t  RESERVED1[56];
  __IO uint32_t  EVENTS_CTS;                        /*!< CTS is activated (set low). Clear To Send.                            */
  __IO uint32_t  EVENTS_NCTS;                       /*!< CTS is deactivated (set high). Not Clear To Send.                     */
  __IO uint32_t  EVENTS_RXDRDY;                     /*!< Data received in RXD                                                  */
  __I  uint32_t  RESERVED2[4];
  __IO uint32_t  EVENTS_TXDRDY;                     /*!< Data sent from TXD                                                    */
  __I  uint32_t  RESERVED3;
  __IO uint32_t  EVENTS_ERROR;                      /*!< Error detected                                                        */
  __I  uint32_t  RESERVED4[7];
  __IO uint32_t  EVENTS_RXTO;                       /*!< Receiver timeout                                                      */
  __I  uint32_t  RESERVED5[46];
  __IO uint32_t  SHORTS;                            /*!< Shortcut register                                                     */
  __I  uint32_t  RESERVED6[64];
  __IO uint32_t  INTENSET;                          /*!< Enable interrupt                                                      */
  __IO uint32_t  INTENCLR;                          /*!< Disable interrupt                                                     */
  __I  uint32_t  RESERVED7[93];
  __IO uint32_t  ERRORSRC;                          /*!< Error source                                                          */
  __I  uint32_t  RESERVED8[31];
  __IO uint32_t  ENABLE;                            /*!< Enable UART                                                           */
  __I  uint32_t  RESERVED9;
  __IO uint32_t  PSELRTS;                           /*!< Pin select for RTS                                                    */
  __IO uint32_t  PSELTXD;                           /*!< Pin select for TXD                                                    */
  __IO uint32_t  PSELCTS;                           /*!< Pin select for CTS                                                    */
  __IO uint32_t  PSELRXD;                           /*!< Pin select for RXD                                                    */
  __I  uint32_t  RXD;                               /*!< RXD register                                                          */
  __O  uint32_t  TXD;                               /*!< TXD register                                                          */
  __I  uint32_t  RESERVED10;
  __IO uint32_t  BAUDRATE;                          /*!< Baud rate                                                             */
  __I  uint32_t  RESERVED11[17];
  __IO uint32_t  CONFIG;                            /*!< Configuration of parity and hardware flow control                     */
} NRF_UART_Type;

#define NRF_UART0                       ((NRF_UART_Type           *) (0x40002000UL))

//------------------------------------------------------------------------------
// Uart initialization
//------------------------------------------------------------------------------
void uartInit()
{
	// Контроль четности отключен, аппаратный контроль выключен
	NRF_UART0->CONFIG = 0;
	NRF_UART0->PSELTXD = 4;
	NRF_UART0->BAUDRATE = 0x01D7E000;
	NRF_UART0->ENABLE = 4;
	NRF_UART0->TASKS_STARTTX = 1;
}

void uartUninit()
{
	NRF_UART0->TASKS_STARTTX = 0;
	NRF_UART0->ENABLE = 0;
}

//------------------------------------------------------------------------------
// Printf putc
//------------------------------------------------------------------------------
void uartPutChar(char c)
{
	NRF_UART0->TXD = c;
	while(NRF_UART0->EVENTS_TXDRDY != 1);
	NRF_UART0->EVENTS_TXDRDY = 0;
}

