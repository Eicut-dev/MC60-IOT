#include "icm42670.h"
#include "ql_type.h"
#include "ql_stdlib.h"
#include "ql_trace.h"
#include "ql_gpio.h"
#include "ql_iic.h"
#include "ql_error.h"
#include "ql_system.h"
#define ICM_MAX_RETRIES 3
// -------------------------
// Low-level I2C helpers
// -------------------------



s32 icm_write(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    for (int i = 0; i < ICM_MAX_RETRIES; i++) {
        s32 res = Ql_IIC_Write(ICM42670_CHNNL_NO, ICM42670_SLAVE_ADDR_WRITE, buf, 2);
        if (res >= 0) return QL_RET_OK;
        Ql_Sleep(5);
    }
    // APP_DEBUG("icm_write failed (reg=0x%02X)\r\n", reg);
    return -1;
}

s32 icm_read(uint8_t reg, uint8_t *buf, uint8_t len) {
    for (int i = 0; i < ICM_MAX_RETRIES; i++) {
        s32 res = Ql_IIC_Write_Read(ICM42670_CHNNL_NO, ICM42670_SLAVE_ADDR_WRITE, &reg, 1, buf, len);
        if (res >= 0) return QL_RET_OK;
        Ql_Sleep(5);
    }
    // APP_DEBUG("icm_read failed (reg=0x%02X)\r\n", reg);
    return -1;
}

  void icm_write_bank(uint8_t bank) {
    icm_write(REG_BANK_SEL, bank);
}

// -------------------------
// Device-specific routines
// -------------------------
  s32 icm_init(void) {
    s32 res = Ql_IIC_Config(ICM42670_CHNNL_NO, TRUE, ICM42670_SLAVE_ADDR_WRITE, 150);
    if (res != QL_RET_OK) {
        // APP_DEBUG("ICM42670: IIC config failed!\r\n");
    }

    // Bank 4: Set power management
    icm_write_bank(4);
    icm_write(REG_PWR_MGMT0, PWR_LN_GYRO_ACCEL);
    icm_write(REG_PWR_MGMT1, 0x00);

    // Return to Bank 0
    icm_write_bank(0);

    // Configure gyro and accel
    icm_write(REG_GYRO_CONFIG0, GYRO_CONFIG0_VALUE);
    icm_write(REG_ACCEL_CONFIG0, ACCEL_CONFIG0_VALUE);
    return res;
}

  uint8_t icm_who_am_i(void) {
    uint8_t buf[1];
    icm_read(REG_WHO_AM_I, buf, 1);
    return buf[0];
}

  int16_t icm_read_temp(void) {
    uint8_t buf[2];
    icm_read(REG_TEMPERATURE, buf, 2);
    int16_t raw = (buf[0] << 8) | buf[1];
    return (raw / 128) + 25;
}

  s32 icm_read_accel(int16_t *ax, int16_t *ay, int16_t *az) {
    uint8_t buf[6];
    s32 res=0;
    res=icm_read(REG_ACCEL_XOUT_H, buf, 6);
    *ax = (buf[0] << 8) | buf[1];
    *ay = (buf[2] << 8) | buf[3];
    *az = (buf[4] << 8) | buf[5];
    return res;
}

  s32 icm_read_gyro(int16_t *gx, int16_t *gy, int16_t *gz) {
    uint8_t buf[6];
    s32 res=0;
    res=icm_read(REG_GYRO_XOUT_H, buf, 6);
    *gx = (buf[0] << 8) | buf[1];
    *gy = (buf[2] << 8) | buf[3];
    *gz = (buf[4] << 8) | buf[5];
    return res;
}