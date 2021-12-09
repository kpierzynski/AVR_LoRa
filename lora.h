#ifndef __LORA_H_
#define __LORA_H_

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

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

uint8_t lora_init();

uint8_t lora_read_register( uint8_t reg );
void lora_write_register( uint8_t reg, uint8_t value );
uint8_t lora_exchange( uint8_t addr, uint8_t val );

void lora_sleep();
void lora_standby();

void lora_rx_continuous();
void lora_event();

void register_lora_rx_event_callback(void (*callback)(uint8_t * buf, uint8_t len, uint8_t status));
uint8_t lora_set_ocp( uint8_t max_current );
int16_t lora_last_packet_rssi( uint32_t freq );
void lora_implicit_header();
void lora_explicit_header();

void lora_tx_power( uint8_t db );
void lora_set_freq( uint32_t freq );
void lora_putd( uint8_t * buf, uint8_t len );

#endif
