Introduction
=====================
**6LBR is a 6LoWPAN Border Router based on The Contiki Operating System.**

This repository ports cetic/6lbr onto stm32f103 platform.
The hardware uses stm32f103ret6 as the core control unit.
Ethernet part uses enc28j60, and radio part uses at86rf231.

Toolchain
=====================
This project uses [summon-arm-toolchain](https://github.com/esden/summon-arm-toolchain) as the cross compilation toolchain. It is a very simple build script for bare metal arm toolchain and it will automaticly intergate libopencm3 as part of the toolchain. 

For the installation refer to [this](http://gnuarmeclipse.livius.net/wiki/Summon_ARM_toolchain_installation_on_Linux), or [my blog](http://zachary.42qu.com/10746872).

The author now suggest using [gcc-arm-embedded](https://launchpad.net/gcc-arm-embedded) instead! But I am too lazy to change the toolchain. Maybe sometime later I will do it when I'm in good mood.

Modification
=====================
# The modification mainly include three part:

## First, cpu/arm/stm32f103/

This project uses libopencm3 as the firmware library for stm32.
So I modify the cpu related code with libopencm3.

NOTE: libopencm3 has a function called `timer_reset()`, it conflicts with
 the same name function in contiki `sys/timer.c`, so I change all of the 
 `timer_reset()` in contiki into `contiki_timer_reset()`.

## Second, platform/stm32-gateway/
This is the platform code for my gateway hardware. Another folder called 
`stm32-mote` is the leaf node, which is used together with `examples/er-rest-example` 
to test the function of the gateway. The two hardware is compatible except that 
the gateway has enc28j60 but the mote doesn't.

## Third, examples/6lbr-router/
This is the main code for 6lbr. 6lbr has three modes but I only use the router mode.
You can easily change the code into the other two modes. And `examples/er-rest-example`
is a CoAP server example, which is used on stm32-mote hardware for a test.

Now it can perform the basic router function.

The code is really rough, hope it has some reference value for you!
