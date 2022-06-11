#ifndef __LORA_H_
#define __LORA_H_

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

//==============================================
//=================== CONFIG ===================
#define SPREADING_FACTOR		SF7
#define CODING_RATE				CODING_RATE_4_5
#define BANDWIDTH				BANDWIDTH_125_KHZ
#define FREQUENCY				433E6

#define SS		(1<<PB2)
#define DDR_SS		DDRB
#define PORT_SS		PORTB

#define RST		(1<<PB1)
#define DDR_RST		DDRB
#define PORT_RST	PORTB

#define SCK		(1<<PB5)
#define DDR_SCK		DDRB
#define PORT_SCK	PORTB

#define MOSI		(1<<PB3)
#define DDR_MOSI	DDRB
#define PORT_MOSI	PORTB

#define MISO		(1<<PB4)
#define DDR_MISO	DDRB
#define PORT_MISO	PORTB
//==============================================
//==============================================

//Init SX1278 module
uint8_t lora_init();

//Read register on 'reg' address
uint8_t lora_read_register( uint8_t reg );

//Write register on address 'reg' with 'value' value
void lora_write_register( uint8_t reg, uint8_t value );

//Put module into sleep mode with LoRa
void lora_sleep();
//Put module into standby mode with LoRa
void lora_standby();
//Put module into receive continuous mode
void lora_rx_continuous();

//Main library event function. This should run in non-blocked main loop
void lora_event();

//Register callback function for receiving data
void register_lora_rx_event_callback(void (*callback)(uint8_t * buf, uint8_t len, uint8_t status));

//Set over current protection on module
uint8_t lora_set_ocp( uint8_t max_current );

//Change bandwidth
//Use provided definitions from lora_mem.h
void lora_set_bandwidth( uint8_t mode );

//Change spreading factor
//Use provided definitions from lora_mem.h
void lora_set_spreading_factor( uint8_t sf );

//Set coding rate
//Use provided definitions from lora_mem.h
void lora_set_coding_rate( uint8_t rate );

//Read Received Signal Strength Indicator (RSSI) from last received packet
int16_t lora_last_packet_rssi( uint32_t freq );

//Use explicit header mode. Module send: Preamble + Header + CRC + Payload + Payload CRC
void lora_explicit_header();

//Set transmitter power value can be set between 2 and 20.
void lora_tx_power( uint8_t db );
//Set working frequency. For SX1278 default value is 433 MHz
void lora_set_freq( uint32_t freq );

//Transmit data from buf
void lora_putd( uint8_t * buf, uint8_t len );

#endif
