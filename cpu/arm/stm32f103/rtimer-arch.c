

/**
 * \file
 *         stm32l-specific rtimer code
 * \author
 *         Erik Jansson
 */

#include <signal.h>
#include <stdio.h>
#include "sys/energest.h"
#include "sys/rtimer.h"
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>

/* We use the TIM2 peripheral as RTIMER */
#define RTIMER_NUMBER 2

/* Some preprocessor stupidity to give us... */
#define CONCAT(s1, s2) s1 ## s2
#define CONCAT3(s1, s2, s3) s1 ## s2 ## s3
#define _RTIMER_PERIPH(n) CONCAT(TIM, n)
/* ...these: */

#define RTIMER_PERIPH _RTIMER_PERIPH(RTIMER_NUMBER)
#define RTIMER_IRQ_HANDLER(n) CONCAT3(tim, n, _isr)
#define RTIMER_IRQ_CHANNEL(n) CONCAT3(NVIC_TIM, n, _IRQ)

static rtimer_clock_t offset=0;

/* IRQ handler */
void RTIMER_IRQ_HANDLER(RTIMER_NUMBER)(void)
{
  if (timer_get_flag(RTIMER_PERIPH,TIM_SR_CC1IF)) {
    /* printf("Timer counter match\n"); */
    /* Clear capture/compare interrupt flag */
	  timer_clear_flag(RTIMER_PERIPH,TIM_SR_CC1IF);
	  rtimer_run_next();
    /* Disable capture/compare interrupt */
	  timer_disable_irq(RTIMER_PERIPH,TIM_DIER_CC1IE);
	  printf("rtimer irq cc\r\n");
  }
}

/* Need to use EXTI for Compare events... */

void
rtimer_arch_init(void)
{
	/* Enable RTIMER clock. */
	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM2EN);
//	rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_TIM4EN);
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN);
  /* Enable interrupt */
	nvic_enable_irq(RTIMER_IRQ_CHANNEL(RTIMER_NUMBER));
	/* Reset RTIMER peripheral. */
	timer_reset(RTIMER_PERIPH);
	/* Timer global mode:
	 * - No divider
	 * - Alignment edge
	 * - Direction up
	 */
	timer_set_mode(RTIMER_PERIPH, TIM_CR1_CKD_CK_INT,
		       TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
  /* Set up prescaler so we end up with desired frequency */
	timer_set_prescaler(RTIMER_PERIPH, MCK/RTIMER_ARCH_SECOND-1);
  /* Disable */
	/* Enable preload. */
	timer_disable_preload(RTIMER_PERIPH);
	timer_disable_counter(RTIMER_PERIPH);
	/* Continous mode. */
	timer_continuous_mode(RTIMER_PERIPH);
	timer_set_period(RTIMER_PERIPH,0xffff);
	timer_disable_oc_output(RTIMER_PERIPH,TIM_OC1);
  /* Counter to zero */
//	timer_set_counter(RTIMER_PERIPH,0);
	/* -- OC1 configuration -- */

	/* Configure global mode of line 1. */
	timer_disable_oc_clear(RTIMER_PERIPH, TIM_OC1);
	timer_disable_oc_preload(RTIMER_PERIPH, TIM_OC1);
	timer_set_oc_slow_mode(RTIMER_PERIPH, TIM_OC1);
  /* Toggle output when a match is detected */
	timer_set_oc_mode(RTIMER_PERIPH,TIM_OC1,TIM_OCM_TOGGLE);
	/* ARR reload enable. */
	timer_disable_preload(RTIMER_PERIPH);
	/* Counter enable. */
	timer_enable_counter(RTIMER_PERIPH);
	/* Enable commutation interrupt. */
//	timer_enable_irq(RTIMER_PERIPH, TIM_DIER_CC1IE);

	printf("Rtimer init done\r\n");
}

void
rtimer_arch_schedule(rtimer_clock_t t)
{
  /* printf("schedule %d\n", t); */
	volatile uint32_t now;
	now = rtimer_arch_now();
	printf("rtimer_arch_schedule time %u; now is %u\r\n", t,now);
  /* Set compare register */
	timer_set_oc_value(RTIMER_PERIPH,TIM_OC1,(uint16_t)t);
  /* Enable interrupt */
	timer_enable_irq(RTIMER_PERIPH,TIM_DIER_CC1IE);
}
void
rtimer_arch_set(rtimer_clock_t t)
{
  offset = t - timer_get_counter(RTIMER_PERIPH);
}

rtimer_clock_t rtimer_arch_now(void)
{
  /* Just return the counter */
	volatile uint32_t now;
	now = timer_get_counter(RTIMER_PERIPH)+offset;
//	printf("rtimer_arch_now:%u\r\n",now);
	return now;
}
