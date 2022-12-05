/*!
	@file   drv_spi_flash.h
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

#ifndef INC_DRV_SPI_FLASH_H_
#define INC_DRV_SPI_FLASH_H_

/******************************************************************************
* Includes
******************************************************************************/

#include <inttypes.h>
#include <limits.h>
#include <string.h>

#if __has_include("FreeRTOS.h")
#include "FreeRTOS.h"
#endif

#if __has_include("task.h")
#include "task.h"
#endif

#if __has_include("cmsis_os.h")
#include "cmsis_os.h"
#endif

#if __has_include("spi.h")
	//#if __has_include("../config_drv_spi_flash.h")
	#include "spi.h"
	//#include "../config_drv_spi_flash.h"
		#define DRV_SPI_FLASH_ENABLED
	#endif
#endif

#ifdef DRV_SPI_FLASH_ENABLED
/******************************************************************************
* Enumerations, structures & Variables
******************************************************************************/

#if !defined(ENUM_I_STATUS)
#define ENUM_I_STATUS
typedef enum
{
	I_OK 			= 0x00,
	I_INVALID 		= 0x01,
	I_EXISTS 		= 0x02,
	I_NOTEXISTS 		= 0x03,
	I_FAILED 		= 0x04,
	I_EXPIRED 		= 0x05,
	I_UNKNOWN 		= 0x06,
	I_INPROGRESS 		= 0x07,
	I_IDLE			= 0x08,
	I_FULL			= 0x09,
	I_EMPTY			= 0x0A,
	I_YES			= 0x0B,
	I_NO			= 0x0C,
	I_SKIP			= 0x0D,
	I_LOCKED 		= 0x0E,
	I_INACTIVE 		= 0x0F,
	I_ACTIVE 		= 0x10,
	I_READY		 	= 0x11,
	I_WAIT 			= 0x12,
	I_OVERFLOW 		= 0x13,
	I_CONTINUE 		= 0x14,
	I_STOPPED 		= 0x15,
	I_WARNING 		= 0x16,
	I_SLEEP 		= 0x17,
	I_DEEPSLEEP 		= 0x18,
	I_STANDBY 		= 0x19,
	I_GRANTED 		= 0x1A,
	I_DENIED 		= 0x1B,
	I_DEBUG_01 		= 0xE0,
	I_DEBUG_02 		= 0xE1,
	I_DEBUG_03 		= 0xE2,
	I_DEBUG_04 		= 0xE3,
	I_DEBUG_05 		= 0xE4,
	I_DEBUG_06 		= 0xE5,
	I_DEBUG_07 		= 0xE6,
	I_DEBUG_08 		= 0xE7,
	I_DEBUG_09 		= 0xE8,
	I_DEBUG_10 		= 0xE9,
	I_DEBUG_11 		= 0xEA,
	I_DEBUG_12 		= 0xEB,
	I_DEBUG_13 		= 0xEC,
	I_DEBUG_14 		= 0xED,
	I_DEBUG_15 		= 0xEE,
	I_DEBUG_16 		= 0xEF,
	I_MEMALIGNED		= 0xFC,
	I_MEMUNALIGNED		= 0xFD,
	I_NOTIMPLEMENTED 	= 0xFE,
	I_ERROR 		= 0xFF
}i_status;
#endif

/*FEATURE SETTING      PAG.37(datasheet)*/

typedef struct{
	uint8_t BRWD;
	uint8_t BP3;
	uint8_t BP2;
	uint8_t BP1;
	uint8_t BP0;
	uint8_t TB;
	uint8_t WP_HOLD;
}b_lock_t;

typedef struct{
	uint8_t CFG2;
	uint8_t CFG1;
	uint8_t LOT_EN;
	uint8_t ECC_EN;
	uint8_t CFG0;
}config_t;

typedef struct{
	uint8_t CRBSY;	//cache busy
	uint8_t ECCS2; 	//correction error
	uint8_t ECCS1; 	//correction error
	uint8_t ECCS0; 	//correction error
	uint8_t P_FAIL; //error during programming
	uint8_t E_FAIL; //error during erase
	uint8_t WEL;	//write enable Bit
	uint8_t OIP;	//Operation in progress
}status_t;

typedef struct{
	b_lock_t b_lock;
	config_t config;
	status_t status;
}feature_setting_t ;

typedef struct{
	uint8_t* buf_in;
	uint8_t* buf_out;
	uint32_t total_sz;
	SPI_HandleTypeDef* handler;
	GPIO_TypeDef *nss_gpio_port;//HW information - Port Chip select
	uint16_t nss_gpio_pin;//HW information - Pin Chip select
	feature_setting_t f_sett;
}spi_flash_t;

/******************************************************************************
* Declaration | Public Functions
******************************************************************************/

i_status spi_flash_init(spi_flash_t* instance);
i_status spi_flash_write(spi_flash_t* instance,uint16_t block,uint8_t page,uint16_t add, uint8_t* buffer,uint32_t size);
i_status spi_flash_read(spi_flash_t* instance,uint16_t block,uint8_t page,uint16_t add);
i_status spi_flash_erase(spi_flash_t* instance, uint32_t block_erase);
i_status cmd_send_to_flash(spi_flash_t* instance, uint16_t size);

#ifdef DRV_SPI_FLASH_DEBUG
	i_status read_id(spi_flash_t* instance);
	i_status read_parameter_page(spi_flash_t* instance);
#endif

/******************************************************************************
* EOF - NO CODE AFTER THIS LINE
******************************************************************************/
#endif
//#endif
