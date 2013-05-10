#include <debug-uart.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include "dev/models.h"


typedef s32 ring_size_t;

struct ring {
	u8 *data;
	ring_size_t size;
	u32 begin;
	u32 end;
};

#define RING_SIZE(RING)  ((RING)->size - 1)
#define RING_DATA(RING)  (RING)->data
#define RING_EMPTY(RING) ((RING)->begin == (RING)->end)

static void ring_init(struct ring *ring, u8 *buf, ring_size_t size)
{
	ring->data = buf;
	ring->size = size;
	ring->begin = 0;
	ring->end = 0;
}

static s32 ring_write_ch(struct ring *ring, u8 ch)
{
	if( ch == '\n')
		ring_write_ch(ring,'\r');
	if (((ring->end + 1) % ring->size) != ring->begin) {
		ring->data[ring->end++] = ch;
		ring->end %= ring->size;
		return (u32)ch;
	}

	return -1;
}

static s32 ring_write(struct ring *ring, u8 *data, ring_size_t size)
{
	s32 i;

	for (i = 0; i < size; i++) {
		if (ring_write_ch(ring, data[i]) < 0)
			return -i;
	}

	return i;
}

static s32 ring_read_ch(struct ring *ring, u8 *ch)
{
	s32 ret = -1;

	if (ring->begin != ring->end) {
		ret = ring->data[ring->begin++];
		ring->begin %= ring->size;
		if (ch)
			*ch = ret;
	}

	return ret;
}

/* Not used!
static s32 ring_read(struct ring *ring, u8 *data, ring_size_t size)
{
	s32 i;

	for (i = 0; i < size; i++) {
		if (ring_read_ch(ring, data + i) < 0)
			return i;
	}

	return -i;
}
*/
#define BUFFER_SIZE 1024

struct ring output_ring;
u8 output_ring_buffer[BUFFER_SIZE];

int _write(int file, char *ptr, int len);


void
dbg_setup_uart_default(void)
{
	/* Initialize output ring buffer. */
	ring_init(&output_ring, output_ring_buffer, BUFFER_SIZE);

	/* Enable the USART1 interrupt. */
	nvic_enable_irq(NVIC_USART2_IRQ);
	gpio_set_mode(DBG_UART_GPIO, GPIO_MODE_OUTPUT_50_MHZ,
			  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, DBG_UART_TX_PIN);

	gpio_set_mode(DBG_UART_GPIO, GPIO_MODE_INPUT,
			  GPIO_CNF_INPUT_FLOAT, DBG_UART_RX_PIN);

	/* Setup UART parameters. */
	usart_set_baudrate(DBG_UART, 115200);
	usart_set_databits(DBG_UART, 8);
	usart_set_stopbits(DBG_UART, USART_STOPBITS_1);
	usart_set_parity(DBG_UART, USART_PARITY_NONE);
	usart_set_flow_control(DBG_UART, USART_FLOWCONTROL_NONE);
	usart_set_mode(DBG_UART, USART_MODE_TX_RX);
	/* Enable USART1 Receive interrupt. */
	USART_CR1(DBG_UART) |= USART_CR1_RXNEIE;
//	usart_enable_rx_interrupt(DBG_UART);
	/* Finally enable the USART. */
	usart_enable(DBG_UART);

	// turn off buffers, so IO occurs immediately
	setvbuf(stdin, NULL, _IONBF, 0);
	setvbuf(stdout, NULL, _IONBF, 0);
	setvbuf(stderr, NULL, _IONBF, 0);
}

unsigned int
dbg_send_bytes(const unsigned char *seq, unsigned int len)
{
	unsigned int i=0;
	while(seq && *seq!=0) {
		if( i >= len) { break; }
		if(*seq == '\n')
		{
			usart_send_blocking(DBG_UART, '\r');
			usart_send_blocking(DBG_UART, '\n');
		}
		else
		{
			usart_send_blocking(DBG_UART, *seq);
		}
		seq++;
		i++;
	}
	return i;
}
static unsigned char dbg_write_overrun = 0;

void
dbg_putchar(const char ch)
{
  if (dbg_write_overrun) {
    if (dbg_send_bytes((const unsigned char*)"^",1) != 1) return;
  }
  dbg_write_overrun = 0;
  if (dbg_send_bytes((const unsigned char*)&ch,1) != 1) {
    dbg_write_overrun = 1;
  }
}

void
dbg_blocking_putchar(const char ch)
{
  if (dbg_write_overrun) {
    while (dbg_send_bytes((const unsigned char*)"^",1) != 1);
  }
  dbg_write_overrun = 0;
  while (dbg_send_bytes((const unsigned char*)&ch,1) != 1);
}
void
dbg_drain()
{
	usart_wait_send_ready(DBG_UART);
}

void usart2_isr(void)
{
	/* Check if we were called because of RXNE. */
	if (((USART_CR1(USART2) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_SR(USART2) & USART_SR_RXNE) != 0)) {

		/* Retrieve the data from the peripheral. */
		ring_write_ch(&output_ring, usart_recv(USART2));

		/* Enable transmit interrupt so it sends back the data. */
		USART_CR1(USART2) |= USART_CR1_TXEIE;
	}

	/* Check if we were called because of TXE. */
	if (((USART_CR1(USART2) & USART_CR1_TXEIE) != 0) &&
	    ((USART_SR(USART2) & USART_SR_TXE) != 0)) {

		s32 data;

		data = ring_read_ch(&output_ring, NULL);

		if (data == -1) {
			/* Disable the TXE interrupt, it's no longer needed. */
			USART_CR1(USART2) &= ~USART_CR1_TXEIE;
		} else {
			/* Put data into the transmit register. */
			usart_send(USART2, data);
		}
	}
}
