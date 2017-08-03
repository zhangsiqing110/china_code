#ifndef MCP9808_L0XX_H
#define MCP9808_L0XX_H

#include <stdint.h>
#include "mcp9808.h"
#include "stm32l0xx_hal.h"

typedef struct mcp9808_l0xx_params {
    /*
     * I2C handle
     */
    I2C_HandleTypeDef *hi2c;

    /*
     * 8-bit shifted I2C address
     */
    uint8_t addr;

    /*
     * I2C transfer timeout
     */
    uint32_t timeout;
} MCP9808_L0XXParams;

void mcp9808_l0xx_init(MCP9808_Iface *iface, MCP9808_L0XXParams *params);

#endif /* MCP9808_L0XX_H */
