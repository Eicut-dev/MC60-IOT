#include "spi_flash_driver.h"
#include "ql_type.h"
#include "ql_gpio.h"
#include "ql_spi.h"
#include "ql_stdlib.h"

void spi_flash_wait_for_ready(u8 ch, u8 *tx, u8 *rx) {
    do {
        tx[0] = FLASH_CMD_READ_STATUS1;
        Ql_SPI_WriteRead(ch, tx, 1, rx, 1);
    } while (rx[0] & FLASH_STATUS_WIP);
}

void spi_flash_read_jedec_id(u8 ch, u8 *idOut, u8 *tx, u8 *rx) {
    tx[0] = FLASH_CMD_READ_JEDEC_ID;
    Ql_memset(rx, 0, 3);
    Ql_SPI_WriteRead(ch, tx, 1, rx, 3);
    idOut[0] = rx[0];
    idOut[1] = rx[1];
    idOut[2] = rx[2];
}

void spi_flash_write_enable(u8 ch, u8 *tx) {
    tx[0] = FLASH_CMD_WRITE_ENABLE;
    Ql_SPI_Write(ch, tx, 1);
}

void spi_flash_sector_erase(u8 ch, u32 addr, u8 *tx, u8 *rx) {
    tx[0] = FLASH_CMD_SECTOR_ERASE;
    tx[1] = (addr >> 16) & 0xFF;
    tx[2] = (addr >> 8) & 0xFF;
    tx[3] = addr & 0xFF;
    Ql_SPI_Write(ch, tx, 4);
    spi_flash_wait_for_ready(ch, tx, rx);
}

void spi_flash_page_program(u8 ch, u32 addr, const u8 *data, u32 len, u8 *tx, u8 *rx) {
    tx[0] = FLASH_CMD_PAGE_PROGRAM;
    tx[1] = (addr >> 16) & 0xFF;
    tx[2] = (addr >> 8) & 0xFF;
    tx[3] = addr & 0xFF;
    Ql_memcpy(&tx[4], data, len);
    Ql_SPI_Write(ch, tx, 4 + len);
    spi_flash_wait_for_ready(ch, tx, rx);
}

void spi_flash_read_data(u8 ch, u32 addr, u8 *buffer, u32 len, u8 *tx, u8 *rx) {
    tx[0] = FLASH_CMD_READ_DATA;
    tx[1] = (addr >> 16) & 0xFF;
    tx[2] = (addr >> 8) & 0xFF;
    tx[3] = addr & 0xFF;
    Ql_SPI_WriteRead(ch, tx, 4, buffer, len);
}
