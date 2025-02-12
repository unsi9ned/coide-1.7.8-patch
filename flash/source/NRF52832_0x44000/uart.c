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
	NRF_UART0->BAUDRATE = 0x0EBED000;  //921600
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

//------------------------------------------------------------------------------
// Отправка строки через UART
//------------------------------------------------------------------------------
void uartSendStr(const char * str)
{

	while (*str)
		uartPutChar(*str++);
}

//------------------------------------------------------------------------------
// Перевод знакового целого в строку
//------------------------------------------------------------------------------
void uartPrintLongint32(long int val, unsigned char dec_places)
{
	unsigned char minus = 0;
	char dst_str[16];
	char * ptr = &dst_str[15];
	unsigned long uval = (unsigned long)val;
	unsigned char digit = 0;

	if(uval == 0)
	{
		uartPutChar('0');
		return;
	}

	//Отрицательное число
	if(uval & 0x80000000)
	{
		uval = ~uval;
		uval++;
		minus = 1;
	}

	//Дробная точка выходит за размер числа (макс 10 разрядов)
	if(dec_places > 10)
	{
		dec_places = 10;
	}

	//Обозначаем конец строки
	*ptr-- = '\0';

	//Заносим каждый разряд с учетом дробной точки
	while(uval > 0)
	{
		if(dec_places && dec_places == digit)
		{
			*ptr-- = '.';
			dec_places = 0;
			continue;
		}

		*ptr-- = (uval % 10) + '0';
		uval /= 10;
		digit++;
	}

	//Дополняем нулями после запятой
	while(dec_places > digit)
	{
		*ptr-- = '0';
		digit++;
	}

	//Ставим точку
	if(dec_places)
	{
		*ptr-- = '.';
		*ptr-- = '0';
		dec_places = 0;
	}

	//Ставим знак
	if(minus)
	{
		*ptr = '-';
	}
	else
	{
		ptr++;
	}

	uartSendStr(ptr);
}

//------------------------------------------------------------------------------
// Вывод в uart числа в hex-формате
//------------------------------------------------------------------------------
void uartPrintHex(unsigned long val, int length)
{
	unsigned char chr;

	if(length == 0)
		return;
	else if(length > 8)
		length = 8;

	uartPutChar('0');
	uartPutChar('x');

	for(int i = length; i > 0; i--)
	{
		chr = (val >> ((i - 1) * 4)) & 0xFul;

		if(chr >= 0 && chr < 10)
			uartPutChar('0' + chr);
		else if(chr >= 10 && chr < 16)
			uartPutChar('A' + (chr - 10));
	}
}

//------------------------------------------------------------------------------
// Вывод параметра на печать
//------------------------------------------------------------------------------
void uartPrintHexParameter(const char * paramName, unsigned long paramVal, int len)
{
	uartSendStr(paramName);
	uartSendStr(" = ");
	uartPrintHex(paramVal, len);
	uartPutChar('\n');
}

//------------------------------------------------------------------------------
// Вывод параметра на печать
//------------------------------------------------------------------------------
void uartPrintIntParameter(const char * paramName, signed long paramVal, unsigned char dotPos)
{
	uartSendStr(paramName);
	uartSendStr(" = ");
	uartPrintLongint32(paramVal, dotPos);
	uartPutChar('\n');
}
