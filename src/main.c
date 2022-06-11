#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

#include <string.h>

#include "lora.h"
#include "lora_mem.h"

void parse_lora( uint8_t * buf, uint8_t len, uint8_t status );

int main() {

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

	//LoRa SX1278 init -----------------------------------------------------------------
	if( !lora_init() ) {
		while(1); //If init returns 0, error occur. Check connections and try again.
	}
	register_lora_rx_event_callback( parse_lora );
	//----------------------------------------------------------------------------------

	PORTC &= ~(1<<PC5);

	sei();

	while( 1 ) {

		lora_event();

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
		// ...process error
		return;
	}

	// ...use received data

	//Blink led
	PORTC ^= (1<<PC5);
}
