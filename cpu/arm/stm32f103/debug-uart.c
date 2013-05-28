#include <debug-uart.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <sys/energest.h>
#include <lib/ringbuf.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include "dev/models.h"

static struct ringbuf ringbuf_tx, ringbuf_rx;
static uint8_t buf_tx[128];
static uint8_t buf_rx[128];

void
dbg_setup_uart_default(void)
{
	/* Initialize output ring buffer. */
	ringbuf_init(&ringbuf_tx, buf_tx, sizeof (buf_tx));
	/* Initialize input ring buffer. */
	ringbuf_init(&ringbuf_rx, buf_rx, sizeof (buf_rx));
	/* Enable the USART1 interrupt. */
	nvic_enable_irq(NVIC_USART2_IRQ);
	nvic_set_priority(NVIC_USART2_IRQ, 2);
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
	usart_enable_rx_interrupt(DBG_UART);
	/* Finally enable the USART. */
	usart_enable(DBG_UART);

	// turn off buffers, so IO occurs immediately
	setvbuf(stdin, NULL, _IONBF, 0);
	setvbuf(stdout, NULL, _IONBF, 0);
	setvbuf(stderr, NULL, _IONBF, 0);
}

/**
 *  \brief      Pushes a char into the output buffer
 * This is blocking.  It used to return -ENOMEM, which is very nice for posix,
 * but no-one was ever looking at it, so we simply truncated lots of writes
 */
int
uart_putchar(char c)
{
	if (c == '\n')
		uart_putchar('\r');
	/* try, try try again! */
	while (ringbuf_put(&ringbuf_tx, c) == 0);
	usart_enable_tx_interrupt(DBG_UART);
	return 0;
}
int
uart_getchar(char *c)
{
	if (ringbuf_elements(&ringbuf_rx) > 0) {
		c = ringbuf_get(&ringbuf_rx);
	} else {
		return -ENODATA;
	}
}
unsigned int
dbg_send_bytes(const unsigned char *seq, unsigned int len)
{
	unsigned int i=0;
	while(seq && *seq!=0) {
		if( i >= len) { break; }
		uart_putchar(*seq);
		seq++;
		i++;
	}
	return i;
}

void
dbg_drain()
{
	usart_wait_send_ready(DBG_UART);
}

void usart2_isr(void)
{
	ENERGEST_ON(ENERGEST_TYPE_IRQ);
	char c;
	if (usart_get_flag(DBG_UART, USART_SR_TXE)) {
		if (ringbuf_elements(&ringbuf_tx) > 0) {
			c = ringbuf_get(&ringbuf_tx);
			usart_send(DBG_UART, (uint16_t) c);
		} else {
			usart_disable_tx_interrupt(DBG_UART);
		}
	}

	if (usart_get_flag(DBG_UART, USART_SR_RXNE)) {
		c = usart_recv(DBG_UART);
		ringbuf_put(&ringbuf_rx, c);
		}
	ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}
