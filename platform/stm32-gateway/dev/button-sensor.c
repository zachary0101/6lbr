/*
 * Copyright (c) 2010, Mariano Alvira <mar@devl.org> and other contributors
 * to the MC1322x project (http://mc1322x.devl.org) and Contiki.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki OS.
 *
 * $Id: button-sensor.c,v 1.1 2010/06/09 14:46:30 maralvira Exp $
 */

#include "dev/button-sensor.h"
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include "dev/models.h"

const struct sensors_sensor button_0_sensor;
const struct sensors_sensor button_1_sensor;
static struct timer debouncetimer;

static int status_b0(int type);
static int status_b1(int type);

void exti0_isr(void)
{
	ENERGEST_ON(ENERGEST_TYPE_IRQ);
	if(timer_expired(&debouncetimer)) {
		timer_set(&debouncetimer, CLOCK_SECOND / 8);
		sensors_changed(&button_0_sensor);
	}
	exti_reset_request(EXTI0);
	ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}
void exti1_isr(void)
{
	ENERGEST_ON(ENERGEST_TYPE_IRQ);
	if(timer_expired(&debouncetimer)) {
		timer_set(&debouncetimer, CLOCK_SECOND / 8);
		sensors_changed(&button_1_sensor);
	}
	exti_reset_request(EXTI1);
	ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}
static int
value_b0(int type)
{
	return gpio_get(BTN_GPIO, BTN0_PIN) || !timer_expired(&debouncetimer);
}
static int
value_b1(int type)
{
	return gpio_get(BTN_GPIO, BTN0_PIN) || !timer_expired(&debouncetimer);
}
static int
configure_b0(int type, int c)
{
	switch (type) {
	case SENSORS_HW_INIT:
		gpio_set_mode(BTN_GPIO, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, BTN0_PIN);
		/* Configure the EXTI subsystem. */
		exti_select_source(BTN0_EXIT, BTN_GPIO);
		exti_set_trigger(BTN0_EXIT, EXTI_TRIGGER_FALLING);
		exti_enable_request(BTN0_EXIT);
		exti_reset_request(BTN0_EXIT);
		/* Enable EXTI0 interrupt. */
		nvic_enable_irq(BTN0_NVIC_IRQ);
		nvic_set_priority(BTN0_NVIC_IRQ,6);
	case SENSORS_ACTIVE:
		if (c) {
			if(!status_b0(SENSORS_ACTIVE)) {
				timer_set(&debouncetimer, 0);
				exti_reset_request(BTN0_EXIT);
				exti_enable_request(BTN0_EXIT);
			}
		} else {
			exti_disable_request(BTN0_EXIT);
		}
		return 1;
	}
	return 0;
}
static int
configure_b1(int type, int c)
{
	switch (type) {
	case SENSORS_HW_INIT:
		gpio_set_mode(BTN_GPIO, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, BTN0_PIN);
		/* Configure the EXTI subsystem. */
		exti_select_source(BTN1_EXIT, BTN_GPIO);
		exti_set_trigger(BTN1_EXIT, EXTI_TRIGGER_FALLING);
		exti_enable_request(BTN1_EXIT);
		exti_reset_request(BTN1_EXIT);
		/* Enable EXTI0 interrupt. */
		nvic_enable_irq(BTN1_NVIC_IRQ);
		nvic_set_priority(BTN1_NVIC_IRQ,6);
	case SENSORS_ACTIVE:
		if (c) {
			if(!status_b1(SENSORS_ACTIVE)) {
				timer_set(&debouncetimer, 0);
				exti_reset_request(BTN1_EXIT);
				exti_enable_request(BTN1_EXIT);
			}
		} else {
			exti_disable_request(BTN1_EXIT);
		}
		return 1;
	}
	return 0;
}
static int
status_b0(int type)
{
	switch (type) {
	case SENSORS_ACTIVE:
	case SENSORS_READY:
		return EXTI_IMR &BTN0_EXIT; /* check if kbi4 irq is enabled */
	}
	return 0;
}
static int
status_b1(int type)
{
	switch (type) {
	case SENSORS_ACTIVE:
	case SENSORS_READY:
		return EXTI_IMR &BTN1_EXIT; /* check if kbi4 irq is enabled */
	}
	return 0;
}

SENSORS_SENSOR(button_0_sensor, BUTTON_SENSOR, value_b0, configure_b0, status_b0);
SENSORS_SENSOR(button_1_sensor, BUTTON_SENSOR, value_b1, configure_b1, status_b1);
