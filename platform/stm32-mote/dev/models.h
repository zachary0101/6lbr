#ifndef __MODELS_H__
#define __MODELS_H__

#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/rcc.h>

/* default setup */
//#define VESNA_SNC

/* pin assignments */
#define LED_GPIO    GPIOA
#define LED_RED     GPIO8
#define LED_YELLOW  GPIO11
#define LED_GREEN   GPIO12

#define BEEP_GPIO		GPIOC
#define BEEP_PIN		GPIO13

#define BTN_GPIO		GPIOA
#define BTN0_PIN		GPIO0
#define BTN1_PIN		GPIO1
#define BTN0_EXIT		EXTI0
#define BTN1_EXIT		EXTI1
#define BTN0_NVIC_IRQ	NVIC_EXTI0_IRQ
#define BTN1_NVIC_IRQ	NVIC_EXTI1_IRQ

#ifndef DBG_UART
#define DBG_UART USART2
#define DBG_UART_GPIO     GPIOA
#define DBG_UART_TX_PIN		GPIO_USART2_TX
#define DBG_UART_RX_PIN		GPIO_USART2_RX
#endif

#define RF_SPI		SPI1

#define RF_IRQ_PORT	GPIOC
#define RF_IRQ_PIN	GPIO4

#define ENC_SPI		SPI2

#define ENC_IRQ_PORT	GPIOC
#define ENC_IRQ_PIN	GPIO6

#define ENC_CS_PORT	GPIOB
#define ENC_CS_PIN	GPIO12

#define ENC_RST_PIN	GPIO7
#define ENC_RST_PORT GPIOC

#endif
