
#ifndef __ICM42670_H__
#define __ICM42670_H__

#include <stdint.h>
#include "ql_type.h"
// -------------------------
// ICM-42670 Constants
// -------------------------
#define ICM42670_SLAVE_ADDR_WRITE     0xD1    // 0b11010001 (R/W bit set for write)
#define ICM42670_CHNNL_NO             0

#define REG_BANK_SEL                  0x76
#define REG_WHO_AM_I                  0x75
#define REG_PWR_MGMT0                 0x1F
#define REG_PWR_MGMT1                 0x20
#define REG_ACCEL_CONFIG0             0x21
#define REG_GYRO_CONFIG0              0x4E
#define REG_ACCEL_XOUT_H               0x0B
#define REG_GYRO_XOUT_H                0x11
#define REG_TEMPERATURE                0x09

// Power modes
#define PWR_LN_GYRO_ACCEL              0x0F

// Gyro/Accel config
#define GYRO_CONFIG0_VALUE             0x6C
#define ACCEL_CONFIG0_VALUE            0x69


// -------------------------
// Low-level I2C helpers
// -------------------------
  s32 icm_write(uint8_t reg, uint8_t val);
  void icm_write_bank(uint8_t bank);
  s32 icm_read(uint8_t reg, uint8_t *buf, uint8_t len);

// -------------------------
// Device-specific routines
// -------------------------
  s32 icm_init(void);
  uint8_t icm_who_am_i(void);
  int16_t icm_read_temp(void);
  s32 icm_read_accel(int16_t *ax, int16_t *ay, int16_t *az);
  s32 icm_read_gyro(int16_t *gx, int16_t *gy, int16_t *gz);

#endif // __ICM42670_H__
