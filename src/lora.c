
#include "lora.h"
#include "lora_mem.h"
#include "spi.h"

//Buffer for receiving data
uint8_t buf[MAX_PKT_LENGTH];

//IRQ pin flag. Note volative specifier, because this variable is used in interrupt
volatile uint8_t rx_done_flag;

//Callback function pointer
static void (*lora_rx_event_callback)(uint8_t * buf, uint8_t len, uint8_t status);

#define INTERRUPT_vect		INT0_vect

void lora_interrupt_init() {
	//Set INT0 to rising edge
	EICRA |= (1<<ISC01)|(1<<ISC00);
	//Allow INT0 to trigger
	EIMSK |= (1<<INT0);
}

uint8_t lora_init() {
	spi_init();

	DDR_RST |= RST;

	PORT_RST |= RST;
	_delay_ms(50);
	PORT_RST &= ~RST;
	_delay_ms(1);
	PORT_RST |= RST;
	_delay_ms(10);

	PORT_SS |= SS;

	lora_interrupt_init();

	uint8_t version = lora_read_register( REG_VERSION );
	if( version != 0x12 ) return 0;

	lora_sleep();
	lora_write_register( REG_OP_MODE, MODE_LONG_RANGE_MODE );

	lora_set_freq( FREQUENCY );

	lora_write_register( REG_FIFO_TX_BASE_ADDR, 0 );
	lora_write_register( REG_FIFO_RX_BASE_ADDR, 0 );

	//Datasheet page: 95
	//RegLna: 7-5 lnaGain, 4-3 lnaBoostLf, 2 reserved, 1-0 lnaboostHf
	lora_write_register( REG_LNA, lora_read_register(REG_LNA) | 0b11 );

	//Datasheet page: 114
	//RegModemConfig3: 7-4 unused, 3 LowDataRateOptimize, 2 AgcAutoOn, 1-0 reserved
	lora_write_register( REG_MODEM_CONFIG_3, 0b100 );

	//Map DIO0 to RX_DONE irq
	lora_write_register(REG_DIO_MAPPING_1, 0x00);

	lora_set_bandwidth(BANDWIDTH);
	lora_set_spreading_factor(SPREADING_FACTOR);
	lora_set_coding_rate(CODING_RATE);

	lora_tx_power( 20 );

	lora_explicit_header();

	lora_standby();

	_delay_ms(50);

	lora_rx_continuous();

	return 1;
}

uint8_t lora_exchange(uint8_t addr, uint8_t val) {
	register uint8_t data;

	//Set chip select (slave select or NSS) pin to low to indicate spi data transmission
	PORT_SS &= ~SS;
	//Send register address
	spi_tx(addr);
	//Read or send register value from module
	data = spi_x(val);
	//End transmission
	PORT_SS |= SS;

	return data;
}

uint8_t lora_read_register(uint8_t reg) {
	//To read register, 8th bit has to be set to 0, which is achieved with masking with 0x7f
	return lora_exchange( reg & 0x7f, 0x00 );
}

void lora_write_register(uint8_t reg, uint8_t value) {
	//When writting to register, 8th bit has to be 1.
	lora_exchange( reg | 0x80, value );
}

void lora_sleep() {
	lora_write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void lora_standby() {
	lora_write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void lora_set_freq( uint32_t freq ) {
	//Datasheet page 37
	//frf (desired freq) = F_XOSC * REG_FRF / 2**(19)
	//So REG_FRF = 2**19 * frf / F_XOSC

	//Note: divide by 2 to n power is equal to shifting by n to left
	//And multiply by 2 to n power is equal to shifting by n to left

	#define F_XOSC 32000000UL
	uint64_t f_Rf = ( (uint64_t)(freq) << 19 ) / F_XOSC;

	lora_write_register( REG_FRF_MSB, (f_Rf >> 16) & 0xFF );
	lora_write_register( REG_FRF_MID, (f_Rf >> 8) & 0xFF );
	lora_write_register( REG_FRF_LSB, (f_Rf >> 0) & 0xFF );
}

//OverCurrentProtection
uint8_t lora_set_ocp( uint8_t max_current ) {
	//Datasheet page 85
	uint8_t ocp_trim;

	if( max_current > 240 ) return 0;

	if( max_current <= 120 ) {
		//max_current = 45 + 5 * ocp_trim;
		ocp_trim = (max_current - 45) / 5;
	} else if( max_current <= 240 ) {
		//max_current = 10 * ocp_trim - 30;
		ocp_trim = (max_current + 30) / 10;
	} else {
		ocp_trim = 27;
	}

	//REG_OCP: 7-6 unused, 5 enable overload current protection, 4-0 ocp_trim
	lora_write_register( REG_OCP, (1<<5) | ( ocp_trim & 0b00011111) );

	return 1;
}

void lora_explicit_header() {
	//Datasheet page 29
	//Explicit mode: preamble + header + crc + payload + payload_crc

	register uint8_t reg_modem_config_1 = lora_read_register( REG_MODEM_CONFIG_1 );

	//RegModemConfig1: 7-4 signal bandwith, 3-1 error coding rate, 0 header type
	lora_write_register( REG_MODEM_CONFIG_1, reg_modem_config_1 & 0b11111110 );
}

//Note: RSSI can be as low as -164. Then its outside of int8_t range (-128 to 127)
int16_t lora_last_packet_rssi(uint32_t freq) {
	uint8_t rssi = lora_read_register(REG_PKT_RSSI_VALUE);
	return -(freq < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT) + rssi;
}

void lora_tx_power( uint8_t db ) {
	//Datasheet page 84

	//RegPaDac: 7-3 reserved with 0x10 default value, which has to be ratained (!), 2-0 pa dac
	//Note: 0x10 << 3 = 0x80

	//RegPaConfig -> datasheet page 109
	//RegPaConfig: 7 PaSelect, 6-4 MaxPower, 3-0 OutputPower

	//db = 17-(15-OutputPower) = 2+OutputPower
	//OutputPower = db - 2

	//Set max current as 150mA
	lora_set_ocp(150);

	if( db > 17 )  {
		if( db > 20 ) db = 20; //Clamp max power to 20
		lora_write_register( REG_PA_DAC, (0x10 << 3) | 0x07 );
		lora_write_register( REG_PA_CONFIG, PA_BOOST | ( (db - 2) - 3 ) );
	} else {
		if( db < 2 ) db = 2; //Clamp min power to 2
		lora_write_register( REG_PA_DAC, (0x10 << 3) | 0x04 );
		lora_write_register( REG_PA_CONFIG, PA_BOOST | (db - 2) );
	}

}

void lora_set_bandwidth( uint8_t mode ) {
	//RegModemConfig1: 7-4 Bandwidth, 3-1 CodingRate, 0 ImplicitHeaderModeOn
	//Datasheet page 112

	lora_write_register( REG_MODEM_CONFIG_1, ( lora_read_register(REG_MODEM_CONFIG_1) & 0b00001111 ) | (mode << 4) );
}

void lora_set_spreading_factor(uint8_t sf)
{
	//DataSheet page 113
	//RegModemConfig2: 7-4 SpreadingFactor 3 TxContinuousMode 2 RxPauloadCrcOn 1-0 SymbolTimeout (msb)
	lora_write_register(REG_MODEM_CONFIG_2, (lora_read_register(REG_MODEM_CONFIG_2) & 0b00001111) | (sf << 4));
}

void lora_set_coding_rate( uint8_t rate ) {
	//Datasheet page 27

	//Datasheet page 112
	//RegModemConfig1: 7-4 Bandwidth, 3-1 CodingRate, 0 ImplicitHeaderModeOn

	lora_write_register( REG_MODEM_CONFIG_1, ( lora_read_register( REG_MODEM_CONFIG_1 ) & 0b11110001 ) | (rate << 1) );
}

void register_lora_rx_event_callback(void (*callback)(uint8_t * buf, uint8_t len, uint8_t status)) {
	lora_rx_event_callback = callback;
}

void lora_rx_continuous() {
	lora_write_register(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS );
}

void lora_putd( uint8_t * buf, uint8_t len ) {
	//Datasheet page 38
	//1. Mode request STAND-BY
	//2. TX init
	//3. Write data to FIFO
	//4. Mode request TX
	//5. Wait for IRQ TxDone
	//6. Auto mode change to STAND-BY
	//7. If new tx, go to 3
	//8. New mode request

	//The LoRaTM FIFO can only be filled in Standby mode.

	if( len == 0 ) return;

	lora_standby();

	lora_write_register( REG_FIFO_ADDR_PTR, 0 );

	for( uint8_t i = 0; i < len; i++ ) lora_write_register( REG_FIFO, buf[i] );
	lora_write_register( REG_PAYLOAD_LENGTH, len );

	lora_write_register( REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX );

	uint8_t irq;
	while( (irq = lora_read_register(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0 );
	lora_write_register( REG_IRQ_FLAGS, irq );

	lora_rx_continuous();
}

void lora_event() {
	//Datasheet page 39
	//1. Mode request STAND-BY
	//2. RX init
	//3. Request mode rx continues
	//4. Wait for IRQ
	//5. RxDone -> IRQ error, back to 4
	//6. Read rx data
	//7. New mode request

	if( rx_done_flag ) {
		uint8_t len;
		uint8_t irq = lora_read_register(REG_IRQ_FLAGS);

		//Clear irq status
		lora_write_register( REG_IRQ_FLAGS, irq );

		//Check if crc error occur
		if( (irq & IRQ_PAYLOAD_CRC_ERROR_MASK) ) {
			//If yes, run callback with crc error status and none data
			if( lora_rx_event_callback ) lora_rx_event_callback( 0, 0, IRQ_PAYLOAD_CRC_ERROR_MASK );
		} else if( (irq & IRQ_RX_DONE_MASK) ) {
			//Check how many data arrived
			len = lora_read_register(REG_RX_NB_BYTES);

			//Set FIFO address to begining of the last received packet.
			lora_write_register( REG_FIFO_ADDR_PTR, lora_read_register(REG_FIFO_RX_CURRENT_ADDR) );

			//Read FIFO to buffer
			for( uint8_t i = 0; i < len; i++ ) buf[i] = lora_read_register( REG_FIFO );
			//Run callback with data
			if( lora_rx_event_callback) lora_rx_event_callback( buf, len, IRQ_RX_DONE_MASK );
		}
		rx_done_flag = 0;
	}
}

ISR( INTERRUPT_vect ) {
	rx_done_flag = 1;
}
