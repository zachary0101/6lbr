/*
 * eeprom.h
 *
 *  Created on: 2013-3-19
 *      Author: zachary
 */

#ifndef EEPROM_H_
#define EEPROM_H_
 /* Page size = 2KByte */
#define PAGE_SIZE  (uint16_t)0x800
/* nvm page where conf will be stored. something wrong here, if address greater than 479K it can't write. */
#define STM32_CONFIG_PAGE (0x8000000+470*1024)
#define STM32_CONFIG_VERSION 1
#define STM32_CONFIG_MAGIC 0x32103E

void EE_Init(void);
void EE_BufferWrite(uint16_t RomAddr,uint16_t NumByteToWrite,uint8_t *pRomData);
void EE_BufferRead(uint16_t RomAddr,uint16_t NumByteToRead,uint8_t *pRomData);


#endif /* EEPROM_H_ */
