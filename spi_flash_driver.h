#ifndef SPI_FLASH_DRIVER_H
#define SPI_FLASH_DRIVER_H

#include "ql_type.h"

#define FLASH_CMD_READ_JEDEC_ID  0x9F
#define FLASH_CMD_WRITE_ENABLE   0x06
#define FLASH_CMD_SECTOR_ERASE   0x20
#define FLASH_CMD_PAGE_PROGRAM   0x02
#define FLASH_CMD_READ_DATA      0x03
#define FLASH_CMD_READ_STATUS1   0x05

#define FLASH_STATUS_WIP         0x01

#define FLASH_TX_BUF_SIZE        260
#define FLASH_RX_BUF_SIZE        260

// All helpers now take shared buffers from caller
void spi_flash_wait_for_ready(u8 ch, u8 *tx, u8 *rx);
void spi_flash_read_jedec_id(u8 ch, u8 *idOut, u8 *tx, u8 *rx);
void spi_flash_write_enable(u8 ch, u8 *tx);
void spi_flash_sector_erase(u8 ch, u32 addr, u8 *tx, u8 *rx);
void spi_flash_page_program(u8 ch, u32 addr, const u8 *data, u32 len, u8 *tx, u8 *rx);
void spi_flash_read_data(u8 ch, u32 addr, u8 *buffer, u32 len, u8 *tx, u8 *rx);

#endif // SPI_FLASH_H