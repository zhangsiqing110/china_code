#ifndef PARK_TEMP_SNS_MCP9808_H
#define PARK_TEMP_SNS_MCP9808_H

#include "mcp9808.h"
#include "park_temp_sns.h"

#define PARK_TEMP_SNS_MCP9808_RES (MCP9808_RES_0_5)
#define PARK_TEMP_SNS_MCP9808_T_CONV_MS (MCP9808_TCONV_MS_0_5 + 10)

void park_tmp_sns_mcp9808_init(PNI_ParkingTemperatureSensorIface *iface,
        MCP9808_Iface *mcp9808);

#endif /* PARK_TEMP_SNS_MCP9808_H */
