# stm32-drv-spi-flash
STM32 Custom Framework Submodule for SPI Flash Memory

NAND Flash Memory
Serial Peripheral Interface (SPI)
MT29F1G01ABAFDSF, MT29F1G01ABAFD12, MT29F1G01ABAFDWB

## Supported Hardware

- first version tested on CNODE board

## Functions Guide
## How To Use

- Include the header file `drv_spi_flash.h`
- define 2 buffer
`static uint8_t msg_tx[2048];`
`static uint8_t msg_rx[2048];`

-initialize the struct

`spi_flash_t sflash = {
		.buf_in = msg_tx,
		.buf_out = msg_rx,
		.handler =&hspi1,
		.nss_gpio_port = MEM_CS_GPIO_Port,
		.nss_gpio_pin  = MEM_CS_Pin,
		.total_sz = 2048
};`

## Example

../SPI_FLASH
In this folder you can find a simple Example (this example work with the CNODE).

## Development & Contribution

Feel free to suggest anything using the Github ISSUES.

## License

This project is released under the MIT License
