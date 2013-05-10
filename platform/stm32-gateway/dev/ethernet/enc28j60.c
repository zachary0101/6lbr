#include <stdio.h>
#include <errno.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/usart.h>
#include "debug-uart.h"
#include "enc28j60.h"
#include "eth-drv.h"
#include "dev/models.h"

#define DEBUG 1
#include "net/uip-debug.h"
static uint8_t Enc28j60Bank;
static uint16_t NextPacketPtr;

void _enc28j60Delay(uint32_t x){
	uint32_t i;
	for(i=x; i > 0; i--)
	{
		asm ("nop");
    }
}

void assertCS(void) {
	gpio_clear(ENC_CS_PORT, ENC_CS_PIN);			// assert CS
}

void releaseCS(void) {
	gpio_set(ENC_CS_PORT, ENC_CS_PIN);				// release CS
}

void enc28j60_hardReset(void) {
	gpio_clear(ENC_RST_PORT, ENC_RST_PIN);		// perform hard reset
	_enc28j60Delay(18000);
	gpio_set(ENC_RST_PORT, ENC_RST_PIN);		// release reset
}
static void spi_setup(void)
{
	rcc_peripheral_enable_clock(&RCC_APB2ENR,
      RCC_APB2ENR_IOPAEN |
      RCC_APB2ENR_IOPBEN |
      RCC_APB2ENR_IOPCEN |
      RCC_APB2ENR_AFIOEN );
	rcc_peripheral_enable_clock(&RCC_APB1ENR,RCC_APB1ENR_SPI2EN);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
			GPIO13 | GPIO14 | GPIO15);
	gpio_set(GPIOB, GPIO12);
	gpio_set(GPIOC, GPIO7);
	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,GPIO12);
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,GPIO7);
	spi_disable(SPI2);
	SPI2_I2SCFGR = 0;
	/* Setup SPI parameters. */
	spi_init_master(ENC_SPI, SPI_CR1_BAUDRATE_FPCLK_DIV_2, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
			SPI_CR1_CPHA_CLK_TRANSITION_1, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
	spi_set_unidirectional_mode(SPI2);
	spi_set_full_duplex_mode(SPI2); /* Not receive-only */
	/* We want to handle the CS signal in software. */
	spi_enable_software_slave_management(SPI2);
	spi_set_nss_high(SPI2);
	spi_enable_ss_output(SPI2);
	/* Finally enable the SPI. */
	spi_enable(SPI2);
}
static u8 spi_readwrite(u32 spi, u8 data)
{
	while (!(SPI_SR(spi) & SPI_SR_TXE))
		;
	SPI_DR(spi) = data;
	while (!(SPI_SR(spi) & SPI_SR_RXNE))
		;
	return SPI_DR(spi);
}
uint8_t enc28j60ReadOp(uint8_t op, uint8_t address) {
	uint8_t data = 0;
	assertCS();							// assert CS signal

	spi_readwrite(ENC_SPI,op | (address & ADDR_MASK));	// issue read command
	data = spi_readwrite(ENC_SPI,0Xff);				// read data (send zeroes)

	if(address & 0x80) {				// do dummy read if needed
		data = spi_readwrite(ENC_SPI,0xff);			// read data (send zeroes)
	}

	releaseCS();						// release CS signal
	return data;
}

void enc28j60WriteOp(uint8_t op, uint8_t address, uint8_t data) {
	assertCS();
	spi_readwrite(ENC_SPI,op | (address & ADDR_MASK));// issue write command
	if (op != ENC28J60_SOFT_RESET) spi_readwrite(ENC_SPI,data);	// send data
    	releaseCS();
}

void enc28j60ReadBuffer(uint16_t len, uint8_t* data) {
#if UIP_CONF_LLH_LEN == 0
  uint8_t header_counter = 0;
#endif
	assertCS();

	spi_readwrite(ENC_SPI,ENC28J60_READ_BUF_MEM);		// issue read command
#if UIP_CONF_LLH_LEN == 0
  header_counter = 0;
  while(header_counter < ETHERNET_LLH_LEN) {
    len--;
    // read data
    ll_header[header_counter] = spi_readwrite(ENC_SPI,0);
    header_counter++;
  }
  while(len) {
    len--;
    // read data
    *data = spi_readwrite(ENC_SPI,0);
    data++;
  }
  *data = '\0';
#elif UIP_CONF_LLH_LEN == 14
  while(len) {
    len--;
    // read data
    *data = spi_readwrite(ENC_SPI,0);
    data++;
  }
  *data = '\0';
#else
#error "UIP_CONF_LLH_LEN value neither 0 nor 14."
#endif
	releaseCS();
}
void enc28j60WriteBuffer(uint16_t len, uint8_t* data) {
#if UIP_CONF_LLH_LEN == 0
  uint8_t header_counter = 0;
#endif
	assertCS();
	spi_readwrite(ENC_SPI,ENC28J60_WRITE_BUF_MEM);		// issue write command
#if UIP_CONF_LLH_LEN == 0
  header_counter = 0;
  while(header_counter < ETHERNET_LLH_LEN) {
    len--;
    // write data
    spi_readwrite(ENC_SPI,ll_header[header_counter]);
    header_counter++;
  }
  while(len) {
    len--;
    // write data
    spi_readwrite(ENC_SPI,*data);
    data++;
  }
#elif UIP_CONF_LLH_LEN == 14
  while(len) {
    len--;
    // write data
    spi_readwrite(ENC_SPI,*data);
    data++;
  }
#else
#error "UIP_CONF_LLH_LEN value neither 0 nor 14."
#endif
	releaseCS();
}
void enc28j60SetBank(uint8_t address) {
	if((address & BANK_MASK) != Enc28j60Bank) {	// set the bank (if needed)
		enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, (ECON1_BSEL1|ECON1_BSEL0));
		enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, (address & BANK_MASK)>>5);
		Enc28j60Bank = (address & BANK_MASK);
	}
}

uint8_t enc28j60Read(uint8_t address) {
	enc28j60SetBank(address);									// set the bank
	return enc28j60ReadOp(ENC28J60_READ_CTRL_REG, address);		// do the read
}

void enc28j60Write(uint8_t address, uint8_t data) {
	enc28j60SetBank(address);									// set the bank
	enc28j60WriteOp(ENC28J60_WRITE_CTRL_REG, address, data);	// do the write
}
uint16_t enc28j60PhyRead(uint8_t address) {
	uint16_t data;

	enc28j60Write(MIREGADR, address);		// set the PHY register address
	enc28j60Write(MICMD,enc28j60Read(MICMD) | MICMD_MIIRD);

	while(enc28j60Read(MISTAT) & MISTAT_BUSY);	// wait until the PHY read completes
	{
		//_delay_us(15);
		//clock_delay(15);
	}

  enc28j60Write(MICMD, enc28j60Read(MICMD) & ~MICMD_MIIRD);	// Clear MICMD.MIIRD bit
  data = (uint16_t) enc28j60Read(MIRDL);
  data |= ((uint16_t) enc28j60Read(MIRDH) << 8);

  //DPRINTF("read phy reg %u:%u\r\n", phyreg, data);
  return data;
}
void enc28j60PhyWrite(uint8_t address, uint16_t data) {
	enc28j60Write(MIREGADR, address);		// set the PHY register address
	enc28j60Write(MIWRL, data);				// write the PHY data
	enc28j60Write(MIWRH, data>>8);
	uint8_t status;

	while((status = enc28j60Read(MISTAT)) & MISTAT_BUSY) {	// wait until the PHY write completes
		//_delay_us(15);
		//clock_delay(15);
	}
}

void enc28j60_clkOut(uint8_t clk)
{
	//setup clkout: 2 is 12.5MHz:
	enc28j60Write(ECOCON, clk & 0x7);
}

int enc28j60_init(uint8_t * macaddr) {
	int i=0;
//	printf("enc28j60 init start.\r\n");
	spi_setup();
	// set output clock disabled
	enc28j60Write(ECOCON, 0x00);
	// perform system reset
	// Suggested Workaround for errata#1 (wait 1ms)
	// as XT2 freq is 16MHz, waiting 16000 cicles (aprox) will do the job.
	// Improvement of the workaround achieved by means of a hard reset.
	enc28j60_hardReset();
	enc28j60WriteOp(ENC28J60_SOFT_RESET, 0, ENC28J60_SOFT_RESET); 
	_enc28j60Delay(18000); // no longer needed
	// check CLKRDY bit to see if reset is complete
	while(!(enc28j60Read(ESTAT) & ESTAT_CLKRDY));
	// Set LED configuration
	enc28j60PhyWrite(PHLCON, 0x3ba6);

	// do bank 0 stuff
	// initialize receive buffer
	// 16-bit transfers, must write low byte first
	// ERXWRPT set automatically when ERXST and ERXND set
	// set receive buffer start address
	NextPacketPtr = RXSTART_INIT;
	enc28j60Write(ERXSTL, RXSTART_INIT&0xFF);
	enc28j60Write(ERXSTH, RXSTART_INIT>>8);
	// set receive pointer address (-1 due to errata #11)
	if ((NextPacketPtr - 1 < RXSTART_INIT) ||
			(NextPacketPtr - 1 > RXSTOP_INIT)) {
		enc28j60Write(ERXRDPTL, (RXSTOP_INIT)&0xFF);
		enc28j60Write(ERXRDPTH, (RXSTOP_INIT)>>8);
	} else { //
		enc28j60Write(ERXRDPTL, (RXSTART_INIT-1)&0xFF);
		enc28j60Write(ERXRDPTH, (RXSTART_INIT-1)>>8);
	}
	// set receive buffer end
	// ERXND defaults to 0x1FFF (end of ram)
	enc28j60Write(ERXNDL, RXSTOP_INIT&0xFF);
	enc28j60Write(ERXNDH, RXSTOP_INIT>>8);
	// set buffer read pointer
	enc28j60Write(ERDPTL, RXSTART_INIT&0xFF);
	enc28j60Write(ERDPTH, RXSTART_INIT>>8);
	// set transmit buffer start
	// ETXST defaults to 0x0000 (beginnging of ram)
	enc28j60Write(ETXSTL, TXSTART_INIT&0xFF);
	enc28j60Write(ETXSTH, TXSTART_INIT>>8);
	// set end of transmit buffer
	enc28j60Write(ETXNDL, TXEND_INIT&0xFF);
	enc28j60Write(ETXNDH, TXEND_INIT>>8);
	// disable filter out of multicast packets
	//enc28j60Write(ERXFCON, 0xA3);	
	// All packet with a valid CRC will be accepted.	
	enc28j60Write(ERXFCON, 0x20);
	
	// do bank 2 stuff
	// enable MAC receive
	enc28j60Write(MACON1, MACON1_MARXEN|MACON1_TXPAUS|MACON1_RXPAUS);
	// enable automatic padding and CRC operations
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, MACON3, MACON3_PADCFG0|MACON3_TXCRCEN|MACON3_FRMLNEN);
	// set MACON 4 bits (necessary for half-duplex only)
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, MACON4, MACON4_DEFER);

	// set inter-frame gap (non-back-to-back)
	enc28j60Write(MAIPGL, 0x12);
	enc28j60Write(MAIPGH, 0x0C);
	// set inter-frame gap (back-to-back)
	enc28j60Write(MABBIPG, 0x12);
	// Set the maximum packet size which the controller will accept
	enc28j60Write(MAMXFLL, MAX_FRAMELEN&0xFF);
	enc28j60Write(MAMXFLH, MAX_FRAMELEN>>8);

	// do bank 3 stuff
	// write MAC address
	// NOTE: MAC address in ENC28J60 is byte-backward
	enc28j60Write(MAADR6, macaddr[0]);
	enc28j60Write(MAADR5, macaddr[1]);
	enc28j60Write(MAADR4, macaddr[2]);
	enc28j60Write(MAADR3, macaddr[3]);
	enc28j60Write(MAADR2, macaddr[4]);
	enc28j60Write(MAADR1, macaddr[5]);
/*
	printf("MAADR6 = 0x%x\r\n", enc28j60Read(MAADR6));
	printf("MAADR5 = 0x%x\r\n", enc28j60Read(MAADR5));
	printf("MAADR4 = 0x%x\r\n", enc28j60Read(MAADR4));
	printf("MAADR3 = 0x%x\r\n", enc28j60Read(MAADR3));
	printf("MAADR2 = 0x%x\r\n", enc28j60Read(MAADR2));
	printf("MAADR1 = 0x%x\r\n", enc28j60Read(MAADR1));
	*/
	// no loopback of transmitted frames
	enc28j60PhyWrite(PHCON2, PHCON2_HDLDIS);

	// switch to bank 0
	enc28j60SetBank(ECON1);
	// enable packet reception
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_RXEN);
	printf("ENC28 REV %u\r\n", enc28j60_getRev());
	
	/*    //Debug code
	    enc28j60PhyWrite(PHLCON,0x3880);//0x476);
	    printf("MAC0 %x\r\n", enc28j60Read(MAADR1));
	    printf("MICMD %x\r\n", enc28j60Read(MICMD));
	    printf("MAC5 %x\r\n", enc28j60Read(MAADR6));
	    printf("MAC1 %x\r\n", enc28j60Read(MAADR2));
	    printf("PHY: %x\r\n", enc28j60PhyRead(PHLCON));
	*/
	return 1;
}


// read the revision of the chip:
uint8_t enc28j60_getRev(void)
{
	return(enc28j60Read(EREVID));
}

void enc28j60PacketSend(uint16_t len, uint8_t* packet) {

	// Set the write pointer to start of transmit buffer area
	enc28j60Write(EWRPTL, TXSTART_INIT&0xFF);
	enc28j60Write(EWRPTH, TXSTART_INIT>>8);

	// Set the TXND pointer to correspond to the packet size given
	enc28j60Write(ETXNDL, (TXSTART_INIT+len)&0xFF);
	enc28j60Write(ETXNDH, (TXSTART_INIT+len)>>8);

	// write per-packet control byte
	enc28j60WriteOp(ENC28J60_WRITE_BUF_MEM, 0, 0x00);

	// copy the packet into the transmit buffer
	enc28j60WriteBuffer(len, packet);

	// workaround due to errata#10
	// perform transmit only reset
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRST);
	enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_TXRST);
	// send the contents of the transmit buffer onto the network
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRTS);

	// wait until transmission ends
	while(enc28j60Read(ECON1) & ECON1_TXRST);
#if UIP_LOGGING
	// check if there was an error
	if(enc28j60Read(ESTAT) & ESTAT_TXABRT){
		// TODO: workaround for errata#13 (read_TSV)
		
	} else {
		
	}
#endif
}
int enc28j60_pending_packet() {
	return enc28j60Read(EPKTCNT);
}
/*-----------------------------------------------------------------
 Gets a packet from the network receive buffer, if one is available.
 The packet will by headed by an ethernet header.
      maxlen  The maximum acceptable length of a retrieved packet.
      packet  Pointer where packet data should be stored.
 Returns: Packet length in bytes if a packet was retrieved, zero otherwise.
-------------------------------------------------------------------*/
uint16_t enc28j60PacketReceive(uint16_t maxlen, uint8_t* packet) {
	uint16_t rxstat;
	uint16_t len;

	// check if a packet has been received and buffered
	if(!enc28j60Read(EPKTCNT)){
		return 0;
	}

	// Set the read pointer to the start of the received packet
	enc28j60Write(ERDPTL, (NextPacketPtr)&0xFF);
	enc28j60Write(ERDPTH, (NextPacketPtr)>>8);
	// read the next packet pointer
	NextPacketPtr  = enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0);
	NextPacketPtr |= enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0)<<8;

	//debug
	// read the packet length
	len  = enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0);
	len |= enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0)<<8;
	// read the receive status
	rxstat  = enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0);
	rxstat |= enc28j60ReadOp(ENC28J60_READ_BUF_MEM, 0)<<8;

	// limit retrieve length
	len = MIN(len, maxlen);

	// copy the packet from the receive buffer
	enc28j60ReadBuffer(len, packet);

	// Move the RX read pointer to the start of the next received packet
	// This frees the memory we just read out
	// (workaround is needed due to errata #11)
	if ((NextPacketPtr - 1 < RXSTART_INIT) ||
			(NextPacketPtr - 1 > RXSTOP_INIT)){
		enc28j60Write(ERXRDPTL, (RXSTOP_INIT)&0xFF);
		enc28j60Write(ERXRDPTH, (RXSTOP_INIT)>>8);
	} else {
        enc28j60Write(ERXRDPTL, (NextPacketPtr-1)&0xFF);
        enc28j60Write(ERXRDPTH, (NextPacketPtr-1)>>8);	
	}

		// decrement the packet counter indicate we are done with this packet
	enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON2, ECON2_PKTDEC);

	return len;
}
unsigned long int enc28j60BlinkLeds(unsigned long int interval, uint8_t times){
	uint16_t old;
	unsigned long int i;
	uint8_t j;
	old = enc28j60PhyRead(PHLCON);
	// turn leds off
	enc28j60PhyWrite(PHLCON, 0x3882);
	// make led A blink slow, B fast
	enc28j60PhyWrite(PHLCON, 0x3BA2);
	for(j = 0; j < times; j++){
		// wait interval
		for(i = 0; i < interval; i++){
			asm ("nop");
			asm ("nop");
		}
	}
	//restore PHLCON
	enc28j60PhyWrite(PHLCON, old);
	return old;
}

void read_TSV(uint8_t *tsv){
	uint16_t read_pt;
	read_pt = enc28j60Read(ETXNDL);
	read_pt |= enc28j60Read(ETXNDH)<<8;
	enc28j60Write(ERDPTL, (read_pt+1)&0xFF);
	enc28j60Write(ERDPTH, (read_pt+1)>>8);
	enc28j60ReadBuffer(7, tsv);
}
