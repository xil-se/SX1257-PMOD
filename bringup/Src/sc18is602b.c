/*
 * SC18IS602B driver
 * 
 * Konrad Beckmann <konrad.beckmann@gmail.com>
 * Public domain 2019-
 * 
 */

#include "sc18is602b.h"

#define SC18IS602B_ADDR_USER      (config->user_address & 0x07)
#define SC18IS602B_ADDR           (0b0101000 | SC18IS602B_ADDR_USER)
#define SC18IS602B_ADDR_W         ((SC18IS602B_ADDR << 1) | 0x01)
#define SC18IS602B_ADDR_R         ((SC18IS602B_ADDR << 1) | 0x00)

int SC18IS602B_write_cmd(const struct SC18IS602B_config *config,
    uint8_t command)
{
    const uint8_t data[] = {
        command,
    };

    return config->i2c_write(config->handle, SC18IS602B_ADDR_W,
        data, sizeof(data), config->timeout);
}

int SC18IS602B_write_cmd_val(const struct SC18IS602B_config *config,
    uint8_t command, uint8_t value)
{
    const uint8_t data[] = {
        command,
        value,
    };

    return config->i2c_write(config->handle, SC18IS602B_ADDR_W,
        data, sizeof(data), config->timeout);
}

int SC18IS602B_read_cmd_val(const struct SC18IS602B_config *config,
    uint8_t command, uint8_t value, uint8_t *read_value)
{
    int ret;
    uint8_t tx[] = {
        command,
        value,
    };

    ret = config->i2c_write(config->handle, SC18IS602B_ADDR_W,
        tx, sizeof(tx), config->timeout);
    if (ret != 0)
        return ret;

    ret = config->i2c_read(config->handle, SC18IS602B_ADDR_W,
        read_value, 1, config->timeout);
    return ret;
}
