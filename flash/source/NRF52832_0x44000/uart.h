#ifndef UART_H_
#define UART_H_

void uartInit();
void uartUninit();
void uartPutChar(char c);
void uartSendStr(const char * str);
void uartPrintLongint32(long int val, unsigned char dec_places);
void uartPrintHex(unsigned long val, int length);
void uartPrintHexParameter(const char * paramName, unsigned long paramVal, int len);
void uartPrintIntParameter(const char * paramName, signed long paramVal, unsigned char dotPos);

#endif
