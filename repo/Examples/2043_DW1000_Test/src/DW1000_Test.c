#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_param_types.h"
#include "platform.h"
#include "deca_sleep.h"

/* PAN ID/EUI/short address. See NOTE 1 and 2 below. */
static uint16 pan_id = 0x1234;
//static uint8 eui[] = {'A', 'C', 'K', 'D', 'A', 'T', 'R', 'X'};
static uint16 short_addr; /* "RX" */

dwt_config_t configData =
{
	.chan = 5,                      //!< channel number {1, 2, 3, 4, 5, 7 }
	.prf = DWT_PRF_64M,             //!< Pulse Repetition Frequency {DWT_PRF_16M or DWT_PRF_64M}
	.txPreambLength = DWT_PLEN_128, //!< DWT_PLEN_64..DWT_PLEN_4096
	.rxPAC = DWT_PAC8,              //!< Acquisition Chunk Size (Relates to RX preamble length)
	.txCode = 9,                    //!< TX preamble code
	.rxCode = 9,                    //!< RX preamble code
	.nsSFD = 0,                     //!< Boolean should we use non-standard SFD for better performance
	.dataRate = DWT_BR_6M8,         //!< Data Rate {DWT_BR_110K, DWT_BR_850K or DWT_BR_6M8}
	.phrMode = DWT_PHRMODE_EXT,     //!< PHR mode {0x0 - standard DWT_PHRMODE_STD, 0x3 - extended frames DWT_PHRMODE_EXT}
	.sfdTO  = (129 + 8 - 8)         //!SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only.
};

int main(void)
{

	/* Start with board specific hardware init. */
	peripherals_init();

	/* Reset and initialise DW1000. See NOTE 2 below.
	 * For initialisation, DW1000 clocks must be temporarily set to crystal speed. After initialisation SPI rate can be increased for optimum
	 * performance. */
	reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */
	spi_set_rate_low();

	if(dwt_initialise(DWT_LOADUCODE | DWT_READ_OTP_PID) == DWT_ERROR)
	{
		printf("\r\nDWT initialization ERROR\r\n");
		while (1)
		{
		};
	}
	else
	{
		printf("\r\nDWT initialization DONE\r\n");
	}

	spi_set_rate_high();

	/* Configure DW1000. */
	dwt_configure(&configData);

	/* Set PAN ID, EUI and short address. See NOTE 2 below. */
	uint8_t nodeId[8];

	*((uint32_t*)&nodeId[4]) = dwt_readdevid();
	printf("Device ID: %04x%04x\r\n", *(uint16_t*)(&nodeId[6]), *(uint16_t*)(&nodeId[4]));

	*((uint32_t*)&nodeId[0]) = dwt_getpartid();
	short_addr = (uint16_t)(dwt_getpartid() & 0xFFFF);

	printf("Chip ID: %04x%04x\r\n", *(uint16_t*)(&nodeId[2]), *(uint16_t*)(&nodeId[0]));

	dwt_seteui(nodeId);
	dwt_setpanid(pan_id);
	dwt_setaddress16(short_addr);

	dwt_geteui(nodeId);
	printf("Node ID: %02x %02x %02x %02x %02x %02x %02x %02x\r\n",
		   nodeId[7], nodeId[6], nodeId[5], nodeId[4],
		   nodeId[3], nodeId[2], nodeId[1], nodeId[0]);

	/* Configure frame filtering. Only data frames are enabled in this example. Frame filtering must be enabled for Auto ACK to work. */
	dwt_enableframefilter(DWT_FF_DATA_EN);

	// Настройка прерываний
	dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | (DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO | DWT_INT_RXPTO), 1);
	//dwt_setcallbacks(ancTxDoneCb, ancRxOkCb, ancRxTimeoutCb, ancRxErrorCb);

	//port_EnableEXT_IRQ();
	dwt_setleds(DWT_LEDS_DISABLE);
	//instanceConfig(&instance);

	while(1);
}
