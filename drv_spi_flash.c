/*!
	@file   drv_spi_flash.c
	@brief  <brief description here>
	@t.odo	-
	---------------------------------------------------------------------------

	MIT License
	Copyright (c) 2022 Federico Carnevale, Ioannis Deligiannis
	
	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:
	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.
	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.
*/
/******************************************************************************
* Preprocessor Definitions & Macros
******************************************************************************/

/******************************************************************************
* Includes
******************************************************************************/

#include "drv_spi_flash.h"

#ifdef DRV_SPI_FLASH_ENABLED
/******************************************************************************
* Preprocessor Post-Definitions & Macros
******************************************************************************/

#define CMD_RESET 0xFF
#define CMD_GET_FEATURES 0x0F
#define CMD_SET_FEATURES 0x1F
#define CMD_READ_ID 0x9F
#define CMD_PAGE_READ 0x13
#define CMD_READ_PAGE_CACHE_RANDOM 0x30
#define CMD_READ_PAGE_CACHE_LAST 0X3F
#define CMD_READ_PAGE_CACHE_x1 0X03
#define CMD_READ_PAGE_CACHE_x2 0X3B
#define CMD_READ_PAGE_CACHE_x4 0X6B
#define CMD_READ_PAGE_CACHE_DUAL_I_0 0XBB
#define CMD_READ_PAGE_CACHE_QUAD_I_0 0XEB
#define CMD_WRITE_ENABLE 0X06
#define CMD_WRITE_DISABLE 0x04
#define CMD_BLOCK_ERASE 0XD8
#define CMD_PROGRAM_EXECUTE 0x10
#define CMD_PROGRAM_LOAD_x1 0x02
#define CMD_PROGRAM_LOAD_x4 0x02
#define CMD_PROGRAM_LOAD_RANDOM_DATA_X1 0x84
#define CMD_PROGRAM_LOAD_RANDOM_DATA_X4 0x34
#define CMD_PERMANENT_BLOCK_LOCK_PROTECTION 0x2C

/***CMD FEARTURE SETTING**************/

#define BLOCK_LOCK 0xA0		//read and write
#define CONFIGURATION 0xB0	//read and write
#define STATUS 0xC0		//read only
#define DIE 0xD0		//read and write ONLY USED IN 4 GB

/******************************************************************************
* Enumerations, structures & Variables
******************************************************************************/

static uint8_t res;

/******************************************************************************
* Declaration | Static Functions
******************************************************************************/

i_status cmd_reset(spi_flash_t* instance);
i_status cmd_write_en(spi_flash_t* instance);
i_status init_setting(spi_flash_t* instance);
i_status cmd_get_feature(spi_flash_t* instance, uint8_t feature_addr);
i_status cmd_set_feature(spi_flash_t* instance, uint8_t feature_addr, uint8_t data);

#ifndef DRV_SPI_FLASH_DEBUG
i_status cmd_send_to_flash(spi_flash_t* instance, uint16_t size);
i_status read_id(spi_flash_t* instance);
i_status read_parameter_page(spi_flash_t* instance);
#endif

/******************************************************************************
* Definition  | Static Functions
******************************************************************************/

i_status cmd_reset(spi_flash_t* instance)
{
	memset(instance->buf_in, 0, instance->total_sz);
	memset(instance->buf_out, 0, instance->total_sz);
	instance->buf_in[0] = CMD_RESET;
	return cmd_send_to_flash(instance, 1);
}

i_status cmd_write_en(spi_flash_t* instance)
{
	memset(instance->buf_in, 0, instance->total_sz);
	memset(instance->buf_out, 0, instance->total_sz);
	instance->buf_in[0] = CMD_WRITE_ENABLE;
	if (cmd_send_to_flash(instance, 1) != I_OK)
		return I_ERROR;
	if (cmd_get_feature(instance, STATUS) != I_OK)
		return I_ERROR;
	return instance->f_sett.status.WEL == 1 ? I_OK : I_ERROR;
}

i_status cmd_send_to_flash(spi_flash_t* instance, uint16_t size)
{
	uint8_t r = HAL_OK;
	HAL_GPIO_WritePin(instance->nss_gpio_port, instance->nss_gpio_pin, RESET);
	__ISB();
	__DSB();
	r = HAL_SPI_TransmitReceive(instance->handler, instance->buf_in, instance->buf_out, size, size);
	__ISB();
	__DSB();
	HAL_GPIO_WritePin(instance->nss_gpio_port, instance->nss_gpio_pin, SET);
	return r == HAL_OK ? I_OK : I_FAILED;
}

i_status init_setting(spi_flash_t* instance)
{
	if (cmd_get_feature(instance, BLOCK_LOCK) != I_OK)
		return I_ERROR;
	if (cmd_get_feature(instance, CONFIGURATION) != I_OK)
		return I_ERROR;
	if (cmd_get_feature(instance, STATUS) != I_OK)
		return I_ERROR;
	return I_OK;
}

i_status cmd_get_feature(spi_flash_t* instance, uint8_t feature_addr)
{
#ifdef DRV_SPI_FLASH_DEBUG
	memset(instance->buf_in, 0, instance->total_sz);
	memset(instance->buf_out, 0, instance->total_sz);
#endif
	if ((feature_addr != BLOCK_LOCK) && (feature_addr != CONFIGURATION) && (feature_addr != STATUS) && (feature_addr != DIE))
	{
		return I_INVALID;
	}

	instance->buf_in[0] = CMD_GET_FEATURES;		// 	CMD
	instance->buf_in[1] = feature_addr;		//	ADD
	instance->buf_in[2] = 0;

	if (cmd_send_to_flash(instance, 3) != I_OK)
		return I_ERROR;

	if (feature_addr == BLOCK_LOCK) /*PAG. 37 */
	{
		instance->f_sett.b_lock.BRWD = (instance->buf_out[2] & 0x80) >> 7;
		instance->f_sett.b_lock.BP3 = (instance->buf_out[2] & 0x40) >> 6;
		instance->f_sett.b_lock.BP2 = (instance->buf_out[2] & 0x20) >> 5;
		instance->f_sett.b_lock.BP1 = (instance->buf_out[2] & 0x10) >> 4;
		instance->f_sett.b_lock.BP0 = (instance->buf_out[2] & 0x08) >> 3;
		instance->f_sett.b_lock.TB = (instance->buf_out[2] & 0x04) >> 2;
		instance->f_sett.b_lock.WP_HOLD = (instance->buf_out[2] & 0x02) >> 1;
	}
	else if (feature_addr == CONFIGURATION)
	{
		instance->f_sett.config.CFG2 = (instance->buf_out[2] & 0x80) >> 7;
		instance->f_sett.config.CFG1 = (instance->buf_out[2] & 0x40) >> 6;
		instance->f_sett.config.LOT_EN = (instance->buf_out[2] & 0x20) >> 5;
		instance->f_sett.config.ECC_EN = (instance->buf_out[2] & 0x10) >> 4;
		instance->f_sett.config.CFG0 = (instance->buf_out[2] & 0x02) >> 1;
	}
	else if (feature_addr == STATUS)
	{
		instance->f_sett.status.CRBSY = (instance->buf_out[2] & 0x80) >> 7;
		instance->f_sett.status.ECCS2 = (instance->buf_out[2] & 0x40) >> 6;
		instance->f_sett.status.ECCS1 = (instance->buf_out[2] & 0x20) >> 5;
		instance->f_sett.status.ECCS0 = (instance->buf_out[2] & 0x10) >> 4;
		instance->f_sett.status.P_FAIL = (instance->buf_out[2] & 0x08) >> 3;
		instance->f_sett.status.E_FAIL = (instance->buf_out[2] & 0x04) >> 2;
		instance->f_sett.status.WEL = (instance->buf_out[2] & 0x02) >> 1;
		instance->f_sett.status.OIP = (instance->buf_out[2] & 0x01) >> 0;
	}
	return I_OK;
}

i_status cmd_set_feature(spi_flash_t* instance, uint8_t feature_addr, uint8_t data)
{
#ifdef DRV_SPI_FLASH_DEBUG
	memset(instance->buf_in, 0, instance->total_sz);
	memset(instance->buf_out, 0, instance->total_sz);
#endif
	if ((feature_addr != BLOCK_LOCK) && (feature_addr != CONFIGURATION) && (feature_addr != STATUS))
	{
		return I_INVALID;
	}

	instance->buf_in[0] = CMD_SET_FEATURES; 	// 	CMD
	instance->buf_in[1] = feature_addr;//	ADD
	instance->buf_in[2] = data;
	//todo: farlo lo stesso come il get
	if (cmd_send_to_flash(instance, 3) != I_OK)
		return I_ERROR;

	//after setting update internal value
	if (cmd_get_feature(instance, feature_addr) != I_OK)
		return I_ERROR;

	return I_OK;
}

/****these function are used only for test the memory ******************/

/* name : read_id
 * param in : buffer to store the information about the id
 * note :	READ ID reads the 2-byte identifier code programmed into the device, which includes
 *			ID and device configuration data as shown in the table below.
 *---------------------------------------------------------------------------
 * Byte		|	Description					|7 6 5 4 3 2 1 0	|	Value	|
 * Byte 0 	|	Manufacturer ID (Micron)	|0 0 1 0 1 1 0 0 	|	2Ch		|
 * Byte 1 	|	1Gb 3.3V Device ID 			|0 0 0 1 0 1 0 0 	|	14h		|
 * --------------------------------------------------------------------------
 */

i_status read_id(spi_flash_t* instance)
{
#ifdef DRV_SPI_FLASH_DEBUG
	memset(instance->buf_in, 0, instance->total_sz);
	memset(instance->buf_out, 0, instance->total_sz);
#endif

	instance->buf_in[0] = CMD_READ_ID;
	instance->buf_in[1] = CMD_READ_ID;
	instance->buf_in[2] = CMD_READ_ID;
	instance->buf_in[3] = CMD_READ_ID;

	if (cmd_send_to_flash(instance, 4) != I_OK)
		return I_ERROR;

	//check instance->buf_in[2] == 0x2C
	//check instance->buf_in[3] == 0x14

	return I_OK;
}

i_status read_parameter_page(spi_flash_t* instance)
{
#ifdef DRV_SPI_FLASH_DEBUG
	memset(instance->buf_in, 0, instance->total_sz);
	memset(instance->buf_out, 0, instance->total_sz);
#endif

	//Step 1
	cmd_set_feature(instance, CONFIGURATION, 0x40);

	//Step 2
	instance->buf_in[0] = CMD_PAGE_READ; 	//	CMD
	instance->buf_in[1] = 0x00; 			//	[1]	ADD x 3 byte
	instance->buf_in[2] = 0x00; 			//	[2]	ADD x 3
	instance->buf_in[3] = 0x01; 			//	[3]	ADD x 3

	if (cmd_send_to_flash(instance, 4) != I_OK)
		return I_ERROR;

	HAL_Delay(2000); // delay per aspettare il trasferimento da nand to cache

	//Step2.1
	if (cmd_get_feature(instance, STATUS) != I_OK)
		return I_ERROR;

	//Step3 - read from cache
	instance->buf_in[0] = 0x03; 	// 	CMD
	instance->buf_in[1] = 0x00; 	//	ADD

	if (cmd_send_to_flash(instance, 2048) != I_OK)
		return I_ERROR;

	//Step 4 - Exit
	if (cmd_set_feature(instance, CONFIGURATION, 0x00) != I_OK)
		return I_ERROR;
	return I_OK;
}

/******************************************************************************
* Definition  | Public Functions
******************************************************************************/

i_status spi_flash_init(spi_flash_t* instance)
{
	if (cmd_reset(instance) != I_OK)
		return I_ERROR;

	if (init_setting(instance) != I_OK)
		return I_ERROR;

	if((instance->f_sett.b_lock.BP0 ==1) && (instance->f_sett.b_lock.BP1 == 1)&&
		(instance->f_sett.b_lock.BP2 ==1)&&(instance->f_sett.b_lock.BP3 ==1)&&
		(instance->f_sett.b_lock.TB == 1)) // controllo se sono bloccati al primo power on e li sblocco tutti da controllare
	{
		cmd_set_feature(instance, BLOCK_LOCK, 0);
	}

	return I_OK;
}


/*name : flash_erase
 *  this funtion is used to erase one block (136kB)
 */
static uint32_t appo0 = 0;
static uint32_t appo1 = 0;
static uint32_t appo2 = 0;

i_status spi_flash_erase(spi_flash_t* instance, uint32_t block_erase)
{
#ifdef DRV_SPI_FLASH_DEBUG
	memset(instance->buf_in, 0, instance->total_sz);
	memset(instance->buf_out, 0, instance->total_sz);
#endif
	//Step 1
	if(cmd_write_en(instance)!= I_OK )
		return I_ERROR;

	appo0 = (0x3FF) & block_erase;	// clear high part; 15-6
	appo0 = appo0 << 6;			//shift
	//Step 2
	instance->buf_in[0] = CMD_BLOCK_ERASE;	//	Block Erase
	instance->buf_in[1] = appo0>>16; 	//	[1]	ADD x 3 byte
	instance->buf_in[2] = appo0>>8; 	//	[2]	ADD x 3
	instance->buf_in[3] = appo0; 		//	[3]	ADD x 3

	if (cmd_send_to_flash(instance, 4) != I_OK)
		return I_FAILED;

	HAL_Delay(10); // todo: check the best time

	if(cmd_get_feature(instance, STATUS) != I_OK) // check if E_FAIL = 0 | OIP =0 | CRBSY =0
		return I_FAILED;
	return instance->f_sett.status.E_FAIL == 1 ? I_ERROR : instance->f_sett.status.OIP == 1 ? I_ERROR : instance->f_sett.status.CRBSY == 1  ?I_ERROR: I_OK;
}

/*
 * input : block(dec)0-1023 // blocchi totali 1024
 * 		 : page(dec)0-63   // pagine per blocco 64
 * 		 : address
 * NB:(8 dummy bits followed by an 16-bit block/page address for 1Gb) PAGE_READ
 */


i_status spi_flash_read(spi_flash_t* instance,uint16_t block,uint8_t page,uint16_t add)
{
	memset(instance->buf_in,0,instance->total_sz);
	memset(instance->buf_out,0,instance->total_sz);

	appo0 = (0x3FF) & block;	// clear high part; 15-6
	appo0 = appo0 << 6;			//shift

	appo1 = (0x3F)&page;		// clear high part;  0-5
	appo2 = appo0 | appo1;		// merge

	//Step 1
	instance->buf_in[0] = CMD_PAGE_READ; 	//	CMD
	instance->buf_in[1] = 00; 				//	[1]	8dummy bits
	instance->buf_in[2] = appo2 >> 8; 		//	[2]	15-6
	instance->buf_in[3] = appo2 >> 0; 		//	[3]	ADD x 3

	if(I_OK!=cmd_send_to_flash(instance,4))
	{
		return  I_ERROR;
	}


	HAL_Delay(500); //todo: delay per aspettare il trasferimento da nand to cache
	cmd_get_feature(instance, STATUS);

	memset(instance->buf_in,0,instance->total_sz);

	appo0 = (0x3F & add);

	//Step3 - read from cache
	instance->buf_in[0] = CMD_READ_PAGE_CACHE_x1; 	// 	CMD
	instance->buf_in[1] = appo0 >> 8 ;	//	ADD
	instance->buf_in[2] = appo0; 		//	ADD
	instance->buf_in[3] = 0x00; 		//	ADD 1byte dummy bits


	if(I_OK!=cmd_send_to_flash(instance,2048+4))
	{
		return  I_ERROR;
	}

	HAL_Delay(500); //todo: delay per aspettare il trasferimento da cache ot buffer
	cmd_get_feature(instance, STATUS);

	return instance->f_sett.status.CRBSY == 0 ? I_OK : I_ERROR ;
}

/* Page 29.
 * The page program sequence is as follows:
 * 1• 06h (WRITE ENABLE command)
 * 2• 02h (PROGRAM LOAD command)
 * 3• 10h (PROGRAM EXECUTE command)
 * 4• 0Fh (GET FEATURES command to read the status)
 * ***************************************/

i_status spi_flash_write(spi_flash_t* instance,uint16_t block,uint8_t page,uint16_t add, uint8_t* buffer,uint32_t size)
{
	memset(instance->buf_in,0,instance->total_sz);
	memset(instance->buf_out,0,instance->total_sz);

	//Step 1 - Enable the write on flash

	cmd_write_en(instance);
	cmd_get_feature(instance, STATUS);

	if(instance->f_sett.status.WEL ==0 )
		return I_ERROR;


	// Step 2 - PROGRAM LOAD

	memmove(&instance->buf_in[3],buffer,size);
	appo0 = (0x0FFF & add);

	instance->buf_in[0] = 0x02; 	//PROGRAM LOAD
	instance->buf_in[1] = appo0 >> 8; 	//bit:xxx|x| - 0000 = column address first 3 dummy after 12 bit.
	instance->buf_in[2] = appo0; 		//bit:0000 - 0000 = column address


	if(I_OK!=cmd_send_to_flash(instance,size))
	{
		return  I_ERROR;
	}

	HAL_Delay(10); // todo: remove..

	appo0 = (0x3FF) & block;	// clear high part; 15-6
	appo0 = appo0 << 6;			//shift

	appo1 = (0x3F)&page;		// clear high part;  0-5
	appo2 = appo0 | appo1;		// merge

	// Step 3 - PROGRAM EXECUTE
	instance->buf_in[0] = 0x10; 		//PROGRAM EXECUTE
	instance->buf_in[1] = 00; 			//addres bit
	instance->buf_in[2] = appo2 >> 8; 	//addres bit
	instance->buf_in[3] = appo2 >> 0; 	//addres bit

	cmd_send_to_flash(instance,4); // todo:check the size

	cmd_get_feature(instance, STATUS);
	while(instance->f_sett.status.OIP != 0) // todo gira finche operation in progress
	{
		cmd_get_feature(instance, STATUS);
		HAL_Delay(1);
	}

	return instance->f_sett.status.P_FAIL == 0 ? I_OK : I_ERROR ; // check se prog is ok
}

/******************************************************************************
* EOF - NO CODE AFTER THIS LINE
******************************************************************************/
#endif


