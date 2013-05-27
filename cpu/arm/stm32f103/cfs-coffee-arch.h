/*
 * Copyright (c) 2010, STMicroelectronics.
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
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *  Coffee architecture-dependent header for the STM32W108-based mb851
 *  platform.
 *  STM32W108 has 128KB of program flash.
 * \author
 *  Salvatore Pitrulli <salvopitru@users.sourceforge.net>
 */

#ifndef CFS_COFFEE_ARCH_H
#define CFS_COFFEE_ARCH_H

#include "contiki-conf.h"
#include <libopencm3/stm32/f1/flash.h>

/* STM32f103RE has 256 pages of 2048 bytes each = 512KB
 * The smallest erasable unit is one page and the smallest writable
 * unit is an aligned 16-bit half-word.
 */

/* Byte page size, starting address on page boundary, and size of the file system */
#define FLASH_START               0x8000000
/* Minimum erasable unit.Page size = 2KByte */
#define FLASH_PAGE_SIZE           2048
/* Last 6 pages reserved for NVM. */
#define FLASH_PAGES               235


/* Minimum reservation unit for Coffee. It can be changed by the user.  */
#define COFFEE_PAGE_SIZE          (FLASH_PAGE_SIZE/4)	//512B

#define COFFEE_ADDRESS            0x8032000	//reserve 200K for program

#if (COFFEE_ADDRESS & 0x3FF) !=0
 #error "COFFEE_ADDRESS not aligned to a 1024-bytes page boundary."
#endif
#define COFFEE_PAGES              ((FLASH_PAGES*FLASH_PAGE_SIZE-(COFFEE_ADDRESS-FLASH_START))/COFFEE_PAGE_SIZE)	//600 pages
#define COFFEE_START              (COFFEE_ADDRESS & ~(COFFEE_PAGE_SIZE-1))	//0x8032000
#define COFFEE_SIZE               (COFFEE_PAGES*COFFEE_PAGE_SIZE)		//300 KB

/* These must agree with the parameters passed to makefsdata */
#define COFFEE_SECTOR_SIZE        FLASH_PAGE_SIZE
#define COFFEE_NAME_LENGTH        20

/* These are used internally by the coffee file system */
/* Micro logs are not needed for single page sectors */
#define COFFEE_MAX_OPEN_FILES     4
#define COFFEE_FD_SET_SIZE        8
#define COFFEE_DYN_SIZE           (COFFEE_PAGE_SIZE*1)
#define COFFEE_MICRO_LOGS         0
#define COFFEE_LOG_TABLE_LIMIT    16    // It doesnt' matter as
#define COFFEE_LOG_SIZE           128   // COFFEE_MICRO_LOGS is 0.


#if COFFEE_PAGES <= 127
#define coffee_page_t u8_t
#elif COFFEE_PAGES <= 0x7FFF
#define coffee_page_t u16_t
#endif


#define COFFEE_WRITE(buf, size, offset) \
        stm32_flash_write(COFFEE_START + offset, buf, size)

#define COFFEE_READ(buf, size, offset) \
        stm32_flash_read(COFFEE_START + offset, buf, size)

#define COFFEE_ERASE(sector) \
        stm32_flash_erase(sector)

void stm32_flash_init(void);
void stm32_flash_read(u32_t address, void * data, u32_t length);
void stm32_flash_write(u32_t address, const void * data, u32_t length);
void stm32_flash_erase(u8_t sector);
void stm32_flash_erase_all(void);
int coffee_file_test(void);

#endif /* !COFFEE_ARCH_H */
