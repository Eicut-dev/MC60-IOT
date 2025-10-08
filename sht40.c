#include "sht40.h"
#include "ql_type.h"
#include "ql_stdlib.h"
#include "ql_trace.h"
#include "ql_gpio.h"
#include "ql_iic.h"

#define SHT4X_CMD_MEASURE_MED_PREC  0xF6
#define SHT4X_CMD_READ_SERIAL       0x89

static int sht4x_write(uint8_t channel, uint8_t addr, uint8_t cmd) {
    return Ql_IIC_Write(channel, addr, &cmd, 1);
}

int sht4x_init(uint8_t channel, uint8_t slave_addr) {
    return Ql_IIC_Config(channel, TRUE, slave_addr, 150);
}

uint8_t sht4x_read_serial(uint8_t channel, uint8_t slave_addr) {
    uint8_t buf = 0;
    sht4x_write(channel, slave_addr, SHT4X_CMD_READ_SERIAL);
    Ql_Sleep(1);
    Ql_IIC_Read(channel, slave_addr, &buf, 1);
    return buf;
}

int sht4x_measure(uint8_t channel, uint8_t slave_addr, sht4x_data_t *data) {
    uint8_t buf[6];
    uint32_t temp_raw, rh_raw;

    sht4x_write(channel, slave_addr, SHT4X_CMD_MEASURE_MED_PREC);
    Ql_Sleep(1);
    Ql_IIC_Read(channel, slave_addr, buf, 6);

    temp_raw = (buf[0] << 8) | buf[1];
    rh_raw   = (buf[3] << 8) | buf[4];

    data->temperature_c = -45 + (175 * (int32_t)temp_raw) / 65535;
    data->humidity_pct  = -6  + (125 * (int32_t)rh_raw) / 65535;

    if (data->humidity_pct > 100) data->humidity_pct = 100;
    if (data->humidity_pct < 0)   data->humidity_pct = 0;

    return 0;
}
