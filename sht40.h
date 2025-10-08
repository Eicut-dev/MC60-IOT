#ifndef SHT40_H
#define SHT40_H


#include <stdint.h>

#define SHT4X_CHNNL_NO     0
#define SHT4X_SLAVE_ADDR   0x88 // Example: 0x44<<1=0x88

typedef struct {
    int32_t temperature_c; // °C
    int32_t humidity_pct;  // % RH
} sht4x_data_t;

int sht4x_init(uint8_t channel, uint8_t slave_addr);
uint8_t sht4x_read_serial(uint8_t channel, uint8_t slave_addr);
int sht4x_measure(uint8_t channel, uint8_t slave_addr, sht4x_data_t *data);

#endif //SHT40_H