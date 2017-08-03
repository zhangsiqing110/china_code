#ifndef PNI_PARK_TEMP_SNS_H
#define PNI_PARK_TEMP_SNS_H

/*
 * Return codes
 */
typedef enum park_temp_sns_ret {
    PARK_TEMP_SNS_RET_ERROR = -1,
    PARK_TEMP_SNS_RET_OK = 0,
} Parking_TemperatureSensorRet;

typedef struct park_temp_sns_iface {
    /*
     * Check if device is present and ready
     */
    int (*is_ready)(void *self);

    /*
     * Get current temperature in °C
     */
    int (*get_temperature)(void *self, float *temperature);

    /*
     * Interface implementation parameters
     */
    void *params;
} PNI_ParkingTemperatureSensorIface;

#endif /* PNI_PARK_TEMP_SNS_H */
