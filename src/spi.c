#include "spi.h"
#include "lora.h"

void spi_init() {
	DDR_SCK |= SCK;
	DDR_MOSI |= MOSI;
	DDR_MISO &= ~MISO;

	DDR_RST |= RST;
	DDR_SS |= SS;

	PORT_SCK |= SCK;
	PORT_MOSI |= MOSI;
	PORT_SS |= SS;
	PORT_RST |= RST;

	SPCR |= (0<<SPIE)|(1<<SPE)|(0<<DORD)|(1<<MSTR)|(0<<CPOL)|(0<<CPHA)|(0<<SPR0)|(1<<SPR1);
	//SPSR |= (1<<SPI2X);
}

void spi_tx( uint8_t byte ) {
	SPDR = byte;
	while(!(SPSR & (1<<SPIF)));
}

uint8_t spi_rx() {
	SPDR = 0x00;
	while(!(SPSR & (1<<SPIF)));
	return SPDR;
}

uint8_t spi_x( uint8_t byte ) {
	SPDR = byte;
	while(!(SPSR & (1<<SPIF)));
	return SPDR;
}
