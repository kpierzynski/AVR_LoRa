#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

#include <string.h>

#include "lora.h"
#include "lora_mem.h"
#include "MK_USART/mkuart.h"

void parse_uart(char * buf);
void parse_lora( uint8_t * buf, uint8_t len, uint8_t status );

int main() {

	char uart_buf[128];
	uint8_t cnt = 0;

	//Led indicator --
	DDRC |= (1<<PC5);
	PORTC |= (1<<PC5);
	//----------------

	//Button for sending data to other module -----
	DDRC &= ~(1<<PC4);
	PORTC |= (1<<PC4);
	uint8_t btn_lock = 0; //Variable for debouncing
	//---------------------------------------------

	//Uart init --------------------------------------
	USART_Init(__UBRR);
	register_uart_str_rx_event_callback( parse_uart );
	//------------------------------------------------

	//LoRa SX1278 init -----------------------------------------------------------------
	if( !lora_init() ) {
		while(1); //If init returns 0, error occur. Check connections and try again.
	}
	register_lora_rx_event_callback( parse_lora );
	//----------------------------------------------------------------------------------

	PORTC &= ~(1<<PC5);

	sei();
	uart_puts("STARTING..\r\n");

	while( 1 ) {

		lora_event();

		UART_RX_STR_EVENT(uart_buf);

		if( !btn_lock && !(PINC & (1<<PC4)) ) {
			btn_lock = 1;

			uint8_t buf[] = { 'c', 'n', 't', ':', (cnt++ % 10) + 48 };
			lora_putd( buf, 5 );
			PORTC ^= (1<<PC5);

		} else if( btn_lock && ( PINC & (1<<PC4) ) ) btn_lock++;
	}

	return 0;
}

void parse_lora( uint8_t * buf, uint8_t len, uint8_t status ) {
	//If error, return.
	if( status == IRQ_PAYLOAD_CRC_ERROR_MASK ) {
		uart_puts("CRC ERROR\n");
		return;
	}

	uart_puts("Data: ");
	for( uint8_t i = 0; i < len; i++ ) {
		uart_putc(buf[i]);
	}
	uart_puts("\n\rRSSI: ");
	uart_putint(lora_last_packet_rssi(433E6),10);
	uart_puts("\n\r");

	//Blink led
	PORTC ^= (1<<PC5);
}

void parse_uart(char * buf) {
	//Enable reset for bootloader (if using one).
	if( !strncasecmp("AT+RST?", buf, 7) ) {
		cli();
		wdt_enable( 0 );
		while(1);
	}

	//Echo data
	uart_puts(buf);
}

