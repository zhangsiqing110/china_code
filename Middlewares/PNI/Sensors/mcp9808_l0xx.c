#include <stdint.h>
#include "mcp9808.h"
#include "mcp9808_l0xx.h"
#include "stm32l0xx_hal.h"

static int reg_read(void *self, uint8_t reg, uint16_t *value)
{
    MCP9808_Iface *iface = self;
    MCP9808_L0XXParams *params = iface->params;
    uint8_t buffer[sizeof(*value)] = { reg, 0x00 };

    /*
     * MCP9808 I2C timing is very sensitive, so we manually do the register
     * reading instead of HAL register reading function.
     */
    if (HAL_I2C_Master_Transmit(params->hi2c, params->addr, buffer,
            1, params->timeout) != HAL_OK) {
        return MCP9808_RET_ERROR;
    }

    if (HAL_I2C_Master_Receive(params->hi2c, params->addr, buffer,
            sizeof(buffer), params->timeout) != HAL_OK) {
        return MCP9808_RET_ERROR;
    }

    *value = (buffer[0] << 8) | buffer[1];
    return MCP9808_RET_OK;
}

static int reg_write(void *self, uint8_t reg, uint16_t value)
{
    MCP9808_Iface *iface = self;
    MCP9808_L0XXParams *params = iface->params;
    uint8_t buffer[sizeof(value) + 1] = { reg, (value >> 8), (value & 0xFF) };

    if (HAL_I2C_Master_Transmit(params->hi2c, params->addr, buffer,
            sizeof(buffer), params->timeout) != HAL_OK) {
        return MCP9808_RET_ERROR;
    }

    return MCP9808_RET_OK;
}

static int reg_write_byte(void *self, uint8_t reg, uint8_t value)
{
    MCP9808_Iface *iface = self;
    MCP9808_L0XXParams *params = iface->params;
    uint8_t buffer[sizeof(value) + 1] = { reg, value };

    if (HAL_I2C_Master_Transmit(params->hi2c, params->addr, buffer,
            sizeof(buffer), params->timeout) != HAL_OK) {
        return MCP9808_RET_ERROR;
    }

    return MCP9808_RET_OK;
}

void mcp9808_l0xx_init(MCP9808_Iface *iface, MCP9808_L0XXParams *params)
{
    iface->reg_read = &reg_read;
    iface->reg_write = &reg_write;
    iface->reg_write_byte = &reg_write_byte;
    iface->params = params;
}
