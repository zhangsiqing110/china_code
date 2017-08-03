#include <stdint.h>
#include "mcp9808.h"

int mcp9808_set_cfg_flag(MCP9808_Iface *iface, uint16_t flag, uint8_t enabled)
{
    uint16_t value = 0x0000;

    if (iface->reg_read(iface, MCP9808_REG_CFG, &value) != MCP9808_RET_OK) {
        return MCP9808_RET_ERROR;
    }

    if (enabled) {
        value |= flag;
    } else {
        value &= ~(flag);
    }

    if (iface->reg_write(iface, MCP9808_REG_CFG, value) != MCP9808_RET_OK) {
        return MCP9808_RET_ERROR;
    }

    return MCP9808_RET_OK;
}

int mcp9808_sleep(MCP9808_Iface *iface)
{
    return mcp9808_set_cfg_flag(iface, MCP9808_CFG_FLAG_SHDN_ENABLED, 1);
}

int mcp9808_wake(MCP9808_Iface *iface)
{
    return mcp9808_set_cfg_flag(iface, MCP9808_CFG_FLAG_SHDN_ENABLED, 0);
}

int mcp9808_get_temperature(MCP9808_Iface *iface, float *temperature)
{
    uint16_t v = 0;
    float t = 0.0f;
 
    if (iface->reg_read(iface, MCP9808_REG_TEMP, &v) != MCP9808_RET_OK) {
        return MCP9808_RET_ERROR;
    }

    t = v & 0x0FFF;
    t /= 16.0f;
    if (v & 0x1000) {
        t -= 256.0f;
    }

    *temperature = t;

    return MCP9808_RET_OK;
}

int mcp9808_set_res(MCP9808_Iface *iface, MCP9808_Resolution res)
{
    uint8_t value = ((uint8_t)res & MCP9808_RES_MASK);

    if (iface->reg_write_byte(iface, MCP9808_REG_RES, value)
            != MCP9808_RET_OK) {
        return MCP9808_RET_ERROR;
    }

    return MCP9808_RET_OK;
}

int mcp9808_is_detected(MCP9808_Iface *iface)
{
    uint16_t value = 0x0000;

    /* get mfg id */
    if (iface->reg_read(iface, MCP9808_REG_MFG_ID, &value) != MCP9808_RET_OK) {
        return MCP9808_RET_ERROR;
    }

    /* check mfg id */
    if (value != MCP9808_MFG_ID) {
        return MCP9808_RET_ERROR;
    }

    /* get dev id */
    if (iface->reg_read(iface, MCP9808_REG_DEV_ID, &value) != MCP9808_RET_OK) {
        return MCP9808_RET_ERROR;
    }

    /* check dev id */
    if (value != MCP9808_DEV_ID) {
        return MCP9808_RET_ERROR;
    }

    return MCP9808_RET_OK;
}
