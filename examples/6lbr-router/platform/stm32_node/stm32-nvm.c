#include "contiki.h"
#include "contiki-lib.h"

#include "cetic-6lbr.h"
#include "nvm-config.h"
#include "nvm-itf.h"

#include "stm32-eeprom.h"

#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define CETIC_6LBR_NVM_ADDRESS (0x100)

void
nvm_data_read(void)
{
  PRINTF("Reading 6LBR NVM\r\n");
  EE_BufferRead(CETIC_6LBR_NVM_ADDRESS,sizeof(nvm_data_t),(uint8_t *) & nvm_data);
}

void
nvm_data_write(void)
{
  PRINTF("Flashing 6LBR NVM\r\n");
  EE_BufferWrite(CETIC_6LBR_NVM_ADDRESS,sizeof(nvm_data_t),(uint8_t *) & nvm_data);
}
