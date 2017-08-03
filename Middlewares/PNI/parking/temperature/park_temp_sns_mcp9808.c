#include <stdint.h>
#include "ble_clock.h"
#include "mcp9808.h"
#include "park_temp_sns.h"
#include "park_temp_sns_mcp9808.h"

static int is_ready(void *self)
{
    PNI_ParkingTemperatureSensorIface *iface = self;
    MCP9808_Iface *mcp9808 = iface->params;

    /* check sensor presence */
    if (mcp9808_is_detected(mcp9808) != MCP9808_RET_OK) {
        return PARK_TEMP_SNS_RET_ERROR;
    }

    /* set resolultion */
    if (mcp9808_set_res(mcp9808, PARK_TEMP_SNS_MCP9808_RES)
            != MCP9808_RET_OK) {
        return PARK_TEMP_SNS_RET_ERROR;
    }

    return PARK_TEMP_SNS_RET_OK;
}

static int get_temperature(void *self, float *temperature)
{
    PNI_ParkingTemperatureSensorIface *iface = self;
    MCP9808_Iface *mcp9808 = iface->params;

    /* wake up sensor */
    if (mcp9808_wake(mcp9808) != MCP9808_RET_OK) {
        return PARK_TEMP_SNS_RET_ERROR;
    }

    /* wait for measurement */
    Clock_Wait(PARK_TEMP_SNS_MCP9808_T_CONV_MS);

    /* get measurement */
    if (mcp9808_get_temperature(mcp9808, temperature) != MCP9808_RET_OK) {
        return PARK_TEMP_SNS_RET_ERROR;
    }

    /* put sensor to sleep */
    if (mcp9808_sleep(mcp9808) != MCP9808_RET_OK) {
        return PARK_TEMP_SNS_RET_ERROR;
    }

    return PARK_TEMP_SNS_RET_OK;
}

void park_tmp_sns_mcp9808_init(PNI_ParkingTemperatureSensorIface *iface,
        MCP9808_Iface *mcp9808)
{
    iface->is_ready = &is_ready;
    iface->get_temperature = &get_temperature;
    iface->params = mcp9808;
}
