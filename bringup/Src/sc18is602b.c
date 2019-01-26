/*
 * SC18IS602B driver
 * 
 * Konrad Beckmann <konrad.beckmann@gmail.com>
 * Public domain 2019-
 * 
 */

#include "sc18is602b.h"

#define SC18IS602B_ADDR_USER      (self.config.user_address)
#define SC18IS602B_ADDR           (0b0101000 | SC18IS602B_ADDR_USER)
#define SC18IS602B_ADDR_W         ((SC18IS602B_ADDR << 1) | 0x01)
#define SC18IS602B_ADDR_R         ((SC18IS602B_ADDR << 1) | 0x00)

static struct {
    struct SC18IS602B_config config;
} self;

int SC18IS602B_init(const struct SC18IS602B_config *config)
{
    self.config = *config;
    return 0;
}

int SC18IS602B_write_cmd(uint8_t command)
{
    const uint8_t data[] = {
        command,
    };

    return self.config.i2c_write(self.config.handle, SC18IS602B_ADDR_W,
        data, sizeof(data), self.config.timeout);
}

int SC18IS602B_write_cmd_val(uint8_t command, uint8_t value)
{
    const uint8_t data[] = {
        command,
        value,
    };

    return self.config.i2c_write(self.config.handle, SC18IS602B_ADDR_W,
        data, sizeof(data), self.config.timeout);
}

int SC18IS602B_read_cmd_val(uint8_t command, uint8_t value, uint8_t *read_value)
{
    int ret;
    uint8_t tx[] = {
        command,
        value,
    };

    ret = self.config.i2c_write(self.config.handle, SC18IS602B_ADDR_W,
        tx, sizeof(tx), self.config.timeout);
    if (ret != 0)
        return ret;

    ret = self.config.i2c_read(self.config.handle, SC18IS602B_ADDR_W,
        read_value, 1, self.config.timeout);
    return ret;
}
