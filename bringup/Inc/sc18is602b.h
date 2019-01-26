/*
 * SC18IS602B driver
 * 
 * Konrad Beckmann <konrad.beckmann@gmail.com>
 * Public domain 2019-
 * 
 * One goal with this driver is to be platform independent,
 * portable and reusable. Feel free to use it anywhere.
 * 
 */

#pragma once

#include <stdint.h>

#define SC18IS602B_SPI_CONFIG     (0xF0)
#define SC18IS602B_INT_CLEAR      (0xF1)
#define SC18IS602B_IDLE           (0xF2)
#define SC18IS602B_GPIO_WRITE     (0xF4)
#define SC18IS602B_GPIO_READ      (0xF5)
#define SC18IS602B_GPIO_ENABLE    (0xF6)
#define SC18IS602B_GPIO_CONFIG    (0xF7)

#define SC18IS602B_SS0            (0x01)
#define SC18IS602B_SS1            (0x02)
#define SC18IS602B_SS2            (0x04)
#define SC18IS602B_SS3            (0x08)

typedef int (*SC18IS602B_i2c_write)(void *handle, uint16_t address, const uint8_t *data, uint32_t count, uint32_t timeout);
typedef int (*SC18IS602B_i2c_read)(void *handle, uint16_t address, uint8_t *data, uint32_t count, uint32_t timeout);

struct SC18IS602B_config {
    /* configurable address bits 0b000 - 0b111 */
    uint8_t user_address;

    /* User-provided handle passed to the i2c write/read callbacks */
    void *handle;
    uint32_t timeout;

    /* i2c write/read implementations */
    SC18IS602B_i2c_write i2c_write;
    SC18IS602B_i2c_read i2c_read;
};

int SC18IS602B_init(const struct SC18IS602B_config *config);
int SC18IS602B_write_bytes(uint8_t *data, uint32_t count);
int SC18IS602B_write_cmd(uint8_t command);
int SC18IS602B_write_cmd_val(uint8_t command, uint8_t value);
int SC18IS602B_read_cmd_val(uint8_t command, uint8_t value, uint8_t *read_value);

#define SC18IS602B_CONFIG_SPI     (0xF0)
#define SC18IS602B_CLEAR_INT      (0xF1)
#define SC18IS602B_IDLE           (0xF2)
#define SC18IS602B_GPIO_WRITE     (0xF4)
#define SC18IS602B_GPIO_READ      (0xF5)
#define SC18IS602B_GPIO_ENABLE    (0xF6)
#define SC18IS602B_GPIO_CONFIG    (0xF7)

#define SC18IS602B_spi_config(config)   SC18IS602B_write_cmd_val(SC18IS602B_SPI_CONFIG,  (config))
#define SC18IS602B_int_clear()          SC18IS602B_write_cmd    (SC18IS602B_INT_CLEAR)
#define SC18IS602B_idle()               SC18IS602B_write_cmd    (SC18IS602B_IDLE)
#define SC18IS602B_gpio_write(pins)     SC18IS602B_write_cmd_val(SC18IS602B_GPIO_WRITE,  (pins))
#define SC18IS602B_gpio_read(pins, val) SC18IS602B_read_cmd_val (SC18IS602B_GPIO_READ,   (pins), (val))
#define SC18IS602B_gpio_enable(pins)    SC18IS602B_write_cmd_val(SC18IS602B_GPIO_ENABLE, (pins))
#define SC18IS602B_gpio_config(config)  SC18IS602B_write_cmd_val(SC18IS602B_GPIO_CONFIG, (config))
