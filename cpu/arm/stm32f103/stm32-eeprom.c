/*
 * eeprom.c
 *
 *  Created on: 2013-3-19
 *      Author: zachary
 */
#include <libopencm3/stm32/f1/flash.h>
#include <libopencm3/stm32/f1/rcc.h>
#include "stm32-eeprom.h"
#include <stdio.h>
#include <string.h>

void EE_Init(void)
{
	flash_prefetch_buffer_enable();
	flash_unlock();
	flash_clear_status_flags();
}

/*******************************************************************************
* Function Name  : EE_BufferRead
* Description    : Reads a block of data from the EEPROM. from 0x08000000+320K
* Input          : -RomAddr
*                  -NumByteToRead
*                  -pRomData
* Output         : None
* Return         : None
*******************************************************************************/
void EE_BufferRead(u16 RomAddr,u16 NumByteToRead,u8 *pRomData)
{
    u32 param_flashbase;
    u8* ptr;
    param_flashbase = STM32_CONFIG_PAGE;
    ptr = (u8*)(param_flashbase + RomAddr);
    while( NumByteToRead-- >0)
    {
        *pRomData = *ptr;            //直接赋值即可
         //printf("0x%x ",*pRomData);
         pRomData++;
         ptr++;
    }
    return;
}

/*******************************************************************************
* Function Name  : EE_BufferWrite
* Description    : Write a block of data to the EEPROM.
* Input          :
*                  -RomAddr
*                  -NumByteToRead
*                  -pRomData
* Output         : None
* Return         : None
*******************************************************************************/
void EE_BufferWrite(u16 RomAddr,u16 NumByteToWrite,u8 *pRomData)
{
	uint32_t param_flashbase;
	uint32_t  tempaddress;
	uint32_t  startaddress;
	uint32_t FlashAddress;
	uint32_t datasource;
	u8 buf1[PAGE_SIZE];
	u8 buf2[PAGE_SIZE];
	u32 pagenumber = 0x0;
	u32 EraseCounter = 0x0;
	u32 i = 0;

	param_flashbase = STM32_CONFIG_PAGE;
	startaddress=tempaddress = param_flashbase+RomAddr;
	/*********************起始指针不在Flash页的开始端*********************/
	if( (tempaddress%PAGE_SIZE) != 0)
	{
//		printf("startptr not in Page head \r\n");
		if(((startaddress%PAGE_SIZE)+NumByteToWrite) > PAGE_SIZE) //超出一页范围
		{
			EE_BufferRead((tempaddress-(tempaddress % PAGE_SIZE)),PAGE_SIZE,buf1);  //把起始地址所在页的内容读到内存buf1中
			memcpy(buf1+(tempaddress % PAGE_SIZE),pRomData,PAGE_SIZE-(tempaddress % PAGE_SIZE));  //把需要写入的数据覆盖到buf1中
			flash_erase_page(tempaddress);       //buf1写入到Flash

			i=PAGE_SIZE/4;
			datasource = (uint32_t)buf1;

			FlashAddress = tempaddress-(tempaddress % PAGE_SIZE);
			while(i-- >0)
			{
				flash_program_word(FlashAddress,*(uint32_t*)datasource);
				if (*(uint32_t*)FlashAddress != *(uint32_t*)datasource)
				{
					printf("EE_BufferWrite error!\r\n");
					return ;
				}
				datasource += 4;
				FlashAddress += 4;
			}

			NumByteToWrite -= PAGE_SIZE-(startaddress % PAGE_SIZE); //需要写入字节数减去，上面覆盖上去的数据的字节数
			tempaddress +=  PAGE_SIZE-(tempaddress % PAGE_SIZE);        //把ptr指针指向下一个页起始位置

			if((NumByteToWrite % PAGE_SIZE) != 0) //末指针不在Flash页的开始端
			{
				//读取1 PAGE 数据到内存，修改，然后写进去
				EE_BufferRead(tempaddress,PAGE_SIZE,buf2);
				memcpy(buf2,pRomData+PAGE_SIZE-startaddress%PAGE_SIZE+NumByteToWrite-NumByteToWrite%PAGE_SIZE,(NumByteToWrite%PAGE_SIZE));
				flash_erase_page( tempaddress+NumByteToWrite);   //把buf2写入到Flash中*
				i=PAGE_SIZE/4;
				datasource = (uint32_t)buf2;
				FlashAddress = (tempaddress+NumByteToWrite-(NumByteToWrite % PAGE_SIZE));  //末地址指针的页首
				while(i-- >0)
				{
					flash_program_word(FlashAddress,*(uint32_t*)datasource);
					if (*(uint32_t*)FlashAddress != *(uint32_t*)datasource)
					{
						printf("EE_BufferWrite error!\r\n");
						return ;
					}
					datasource += 4;
					FlashAddress += 4;
				}
			}

			NumByteToWrite -= NumByteToWrite % PAGE_SIZE;

			//擦除Flash
			pagenumber =  NumByteToWrite/PAGE_SIZE;
			for (EraseCounter = 0; EraseCounter < pagenumber; EraseCounter++)
			{
				flash_erase_page( tempaddress + PAGE_SIZE*EraseCounter );
			}
			//写Flash
			datasource = *(uint32_t *)(pRomData+ PAGE_SIZE-(startaddress % PAGE_SIZE)  );
			FlashAddress = tempaddress;

			while( pagenumber-- > 0 )
			{
				i=PAGE_SIZE/4;
				while(i -- >0)
				{
					flash_program_word(FlashAddress,*(uint32_t*)datasource);
					if (*(uint32_t*)FlashAddress != *(uint32_t*)datasource)
					{
						printf("EE_BufferWrite error!\r\n");
						return ;
					}
					datasource += 4;
					FlashAddress += 4;
				}
			}
		}
		else //写的内容没有超出一页范围
		{
//			printf("FlashWrire --in one page \r\n");
			EE_BufferRead((startaddress-(startaddress % PAGE_SIZE)),PAGE_SIZE,buf1);     //把起始地址所在页的内容读到内存buf1中
			memcpy( (buf1+(tempaddress % PAGE_SIZE)),pRomData, NumByteToWrite );  //把需要写入的数据覆盖到buf1中
			flash_erase_page(tempaddress);
			i=PAGE_SIZE/4;
			datasource = (uint32_t)buf1;
			FlashAddress = tempaddress-(tempaddress % PAGE_SIZE);
			while(i-- >0)
			{
				flash_program_word(FlashAddress,*(uint32_t*)datasource);
				if (*(uint32_t *)FlashAddress != *(uint32_t *)datasource) //读取Flash中的数据，看是否写入正确
				{
					printf("EE_BufferWrite error!\r\n");
					return ;
				}
				datasource += 4;
				FlashAddress += 4;
			}
		}
	}
	/*******************起始指针在Flash页的开始端****************************/
	else
	{
//		printf("startptr  in Page head \r\n");
		if((NumByteToWrite % PAGE_SIZE) != 0)
		{
			//读取1 PAGE 数据到内存，修改，然后写进去
			EE_BufferRead((u16)(tempaddress+NumByteToWrite-(NumByteToWrite % PAGE_SIZE)),PAGE_SIZE,buf1);
//			printf("already copy to bug1 \r\n");
			memcpy(buf1,pRomData+NumByteToWrite-(NumByteToWrite % PAGE_SIZE),(NumByteToWrite % PAGE_SIZE));
			//end of debug
		}
		//擦除Flash
		if( (NumByteToWrite%PAGE_SIZE) == 0 )
		{
			pagenumber = NumByteToWrite/PAGE_SIZE;
		}
		else
		{
			pagenumber = NumByteToWrite/PAGE_SIZE + 1;
		}
		for (EraseCounter = 0; EraseCounter < pagenumber; EraseCounter++)
		{
			flash_erase_page(startaddress + (PAGE_SIZE * EraseCounter));
		}
		//写Flash
		if( pagenumber == 1)   //只有一页
		{
			i=PAGE_SIZE/4;
			datasource = (uint32_t)buf1;
			FlashAddress = startaddress;
			while(i-- >0)
			{
				flash_program_word(FlashAddress,*(uint32_t *)datasource);
				if (*(uint32_t *)FlashAddress != *(uint32_t *)datasource)
				{
					printf("EE_BufferWrite error!\r\n");
					return ;
				}
				datasource +=4;
				FlashAddress +=4;
			}
		}
		else //很多页时，先写前面页，最后写buf1
		{
			while( pagenumber-- > 1 )
			{
				datasource = (u32)pRomData;
				FlashAddress = startaddress;
				i=PAGE_SIZE/4;
				while(i -- >0)
				{
					flash_program_word( FlashAddress, *(uint32_t *)datasource );
					if (*(uint32_t *)FlashAddress != *(uint32_t *)datasource)
					{
						printf("EE_BufferWrite error!\r\n");
						return ;
					}
					datasource += 4;
					FlashAddress += 4;
				}
			}
			//写后面的页
			datasource = (uint32_t)buf1;
			FlashAddress = startaddress+(pagenumber-1)*PAGE_SIZE;
			i=PAGE_SIZE/4;
			while(i -- >0)
			{
				flash_program_word( FlashAddress, *(uint32_t *)datasource );
				if (*(uint32_t *)FlashAddress != *(uint32_t *)datasource)
				{
					printf("EE_BufferWrite error!\r\n");
					return ;
				}
				datasource += 4;
				FlashAddress += 4;
			}
		}
	}
}
