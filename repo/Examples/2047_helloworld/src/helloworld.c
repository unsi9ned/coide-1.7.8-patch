#include "nrf.h"
#include "nrf_delay.h"
#include "nordic_common.h"
#include "nrf_drv_uart.h"

typedef struct
{
	uint8_t data[1024];
	uint16_t cursor;
	uint16_t len;
}
rxbuf_t;

static nrf_drv_uart_t uart_instance = NRF_DRV_UART_INSTANCE(0);
static nrf_drv_uart_config_t config = NRF_DRV_UART_DEFAULT_CONFIG;

static rxbuf_t rxbuf = {.len = sizeof(rxbuf.data), .cursor = 0};

static const char hello_world[] = "Hello world!\n";

static void nrf_uart_event_handler(nrf_drv_uart_event_t * p_event, void * p_context);
static void uart_init();

//------------------------------------------------------------------------------
// Print hello world
//------------------------------------------------------------------------------
void helloworld()
{
	uart_init();
	nrf_drv_uart_tx(&uart_instance, (uint8_t const *)hello_world, sizeof(hello_world));
}

//------------------------------------------------------------------------------
// UART event handler
//------------------------------------------------------------------------------
static void nrf_uart_event_handler(nrf_drv_uart_event_t * p_event, void * p_context)
{
	switch(p_event->type)
	{
		case NRF_DRV_UART_EVT_RX_DONE:
			rxbuf.cursor += p_event->data.rxtx.bytes;

			if(*p_event->data.rxtx.p_data == '\r')
			{
				nrf_drv_uart_rx_abort(&uart_instance);
			}
			else
			{
				nrf_drv_uart_rx(&uart_instance, &rxbuf.data[rxbuf.cursor], sizeof(char));
			}
			break;

		case NRF_DRV_UART_EVT_ERROR:
			nrf_drv_uart_rx_abort(&uart_instance);
			nrf_drv_uart_tx_abort(&uart_instance);
			break;

		default:
			break;
	}
}

//------------------------------------------------------------------------------
// Init UART
//------------------------------------------------------------------------------
static void uart_init()
{
	static bool state = false;

	if(state)
		return;

	config.pselrxd = 11;          // RX pin
	config.pseltxd = 4;           // TX pin
	config.use_easy_dma = false;

	nrf_drv_uart_uninit(&uart_instance);
	nrf_drv_uart_init(&uart_instance, &config, nrf_uart_event_handler);

	state = true;
}

