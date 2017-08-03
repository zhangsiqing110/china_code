#ifndef PNI_PARKING_SENSOR_RTI_H
#define PNI_PARKING_SENSOR_RTI_H

#include "parking_sensor.h"
#include "SENtralA2_types.h"

typedef enum pni_park_rti_sns_type {
    PARK_RTI_SNS_TYPE_CAR_DETECTOR = 0x15,
    PARK_RTI_SNS_TYPE_RAW_MAGNETIC_FIELD = 0x1D,
    PARK_RTI_SNS_TYPE_CAR_DETECTOR_DATA = 0x20,
    PARK_RTI_SNS_TYPE_PARAMETER_REPORT = 0x21,
} PNI_ParkingRtiSensorType;

typedef enum pni_park_rti_pio_page {
    PARK_RTI_PIO_PAGE_INFO = 0x09,
    PARK_RTI_PIO_PAGE_USER = 0x0E,
} PNI_ParkingRtiParameterPage;

typedef enum pni_park_rti_pio_info {
    PARK_RTI_PIO_INFO_FW_VERSION = 0x01,
    PARK_RTI_PIO_INFO_FW_TIMESTAMP = 0x02, // uint64_t
} PNI_ParkingRtiInfoParameter;

#pragma pack(push, 1)
typedef struct pni_park_rti_car_detector_cfg {
    uint8_t continuous_mode:1;
    uint8_t intermediate_report:1;
} PNI_ParkingRtiCarDetectorCfg;
#pragma pack(pop)

typedef enum {
    PARK_RTI_PIO_USER_CTX_INPUT = 0x01, // uint8_t
    PARK_RTI_PIO_USER_CAR_DETECTOR_OUTPUT_TYPE = 0x02, // uint8_t
    PARK_RTI_PIO_USER_HS_TRIGGER_THRD = 0x03, // float
    PARK_RTI_PIO_USER_CAL_TIMEOUT = 0x04, // uint16_t
    // NOT USED = 0x05,
    PARK_RTI_PIO_USER_STD = 0x06, // float
    // NOT USED = 0x07,
    PARK_RTI_PIO_USER_STD_THRD = 0x08, // float
    PARK_RTI_PIO_USER_MAG_THRD = 0x09, // float
    PARK_RTI_PIO_USER_MAG_THRD_UP = 0x0A, // float
    PARK_RTI_PIO_USER_MAG_THRD_UP_2 = 0x0B, // float
    PARK_RTI_PIO_USER_CNT_THRD = 0x0C, // uint16_t
    PARK_RTI_PIO_USER_STATE_CNT_THRD = 0x0D, // uint16_t
    PARK_RTI_PIO_USER_STATE_CNT_THRD_TIMEOUT = 0x0E, // uint16_t
    PARK_RTI_PIO_USER_AVG2_0 = 0x0F, // float
    PARK_RTI_PIO_USER_AVG2_1 = 0x10, // float
    PARK_RTI_PIO_USER_AVG2_2 = 0x11, // float
    PARK_RTI_PIO_USER_AVG2_INIT_0 = 0x12, // float
    PARK_RTI_PIO_USER_AVG2_INIT_1 = 0x13, // float
    PARK_RTI_PIO_USER_AVG2_INIT_2 = 0x14, // float
    PARK_RTI_PIO_USER_DETECT_THRD = 0x15, // float
    PARK_RTI_PIO_USER_MOV_AVG = 0x16, // float
    PARK_RTI_PIO_USER_FILTER_ALPHA = 0x17, // float
    PARK_RTI_PIO_USER_FILTER_BETA = 0x18, // float
    // NOT USED = 0x19,
    PARK_RTI_PIO_USER_MAG_THRD_2 = 0x1A, // float
    PARK_RTI_PIO_USER_TIMEOUT_TRANSITION = 0x1B, // uint16_t
    PARK_RTI_PIO_USER_G_ALARM_MODE = 0x1C, // uint8_t
    PARK_RTI_PIO_USER_G_LS_RATE = 0x1D, // uint8_t
    PARK_RTI_PIO_USER_G_HS_RATE = 0x1E, // uint8_t
    PARK_RTI_PIO_USER_G_PARK_STRUCT_INIT_FLAG = 0x1F, // uint8_t
    PARK_RTI_PIO_USER_LS_START_VALUE_1_0 = 0x20, // float(R)
    PARK_RTI_PIO_USER_LS_START_VALUE_1_1 = 0x21, // float(R)
    PARK_RTI_PIO_USER_LS_START_VALUE_1_2 = 0x22, // float(R)
    PARK_RTI_PIO_USER_LS_START_VALUE_3_0 = 0x23, // float(R)
    PARK_RTI_PIO_USER_LS_START_VALUE_3_1 = 0x24, // float(R)
    PARK_RTI_PIO_USER_LS_START_VALUE_3_2 = 0x25, // float(R)
    PARK_RTI_PIO_USER_BLE_TRIGGER_THRESH = 0x26, // float
    PARK_RTI_PIO_USER_TRIGGER_FLAG = 0x27, // uint8_t
    PARK_RTI_PIO_USER_BLE_TRIGGER_MODE = 0x28, // uint8_t
    PARK_RTI_PIO_USER_SHIPPING_MODE_FLAG = 0x29, // uint8_t
    PARK_RTI_PIO_USER_HS_TRIGGER_THRESH_SHIP = 0x2A, // float
    PARK_RTI_PIO_USER_SHIPPING_MODE_INIT_FLAG = 0x2B, // uint8_t
    PARK_RTI_PIO_USER_STANDBY_MODE_FLAG = 0x2C, // uint8_t
    PARK_RTI_PIO_USER_STANDBY_MODE_CNT = 0x2D, // uint16_t
    PARK_RTI_PIO_USER_STANDBY_MODE_TIMEOUT = 0x2E, // uint16_t
    PARK_RTI_PIO_USER_TEMPERATURE = 0x2F, // int8_t
    PARK_RTI_PIO_USER_STD_THRESH_UP = 0x30, // float
    PARK_RTI_PIO_USER_STD_THRESH_DOWN = 0x31, // float
    PARK_RTI_PIO_USER_DETECT_THRESH_UP = 0x32, // float
    PARK_RTI_PIO_USER_DETECT_THRESH_DOWN = 0x33, // float
    PARK_RTI_PIO_USER_HS_DATA_N_DIFF_THRESH = 0x34, // float
    PARK_RTI_PIO_USER_BLE_TRIGGER_ENABLE = 0x35, // uint8_t
    PARK_RTI_PIO_USER_LOCAL_BASE_LINE_UPDATE_ENABLE = 0x36, // uint8_t
    PARK_RTI_PIO_USER_HS_DATA_DIFF_Z_THRESH_LEVEL1 = 0x37, // float
    PARK_RTI_PIO_USER_HS_DATA_DIFF_Z_THRESH_LEVEL2 = 0x38, // float
    PARK_RTI_PIO_USER_HS_DATA_DIFF_XY_THRESH_LEVEL2 = 0x39, // float
    PARK_RTI_PIO_USER_DATA_N_DIFF_THRESH = 0x3A, // float
    PARK_RTI_PIO_USER_DATA_N_DIFF_THRESH2 =  0x3B, // float
    PARK_RTI_PIO_USER_MAG_THRESH =  0x3C, // float
    PARK_RTI_PIO_USER_MAG_THRESH2 =  0x3D, // float
    PARK_RTI_PIO_USER_MAG_THRESH3 =  0x3E, // float
    PARK_RTI_PIO_USER_MAG_THRESH_LEVEL1 =  0x3F, // float
    PARK_RTI_PIO_USER_MAG_THRESH_LEVEL2 =  0x40, // float
    PARK_RTI_PIO_USER_MAG_THRESH_Z_AXIS_LEVEL3 =  0x41, // float
    PARK_RTI_PIO_USER_HS_DATA_DIFF_THRESH_LEVEL3 =  0x42, // float
} PNI_ParkingRtiUserParameter;

typedef enum pni_park_rti_user_ctx_input {
    PARK_RTI_USER_CTX_INPUT_NO_CAR = 0x01,
    PARK_RTI_USER_CTX_INPUT_CAR_ENTERING = 0x02,
    PARK_RTI_USER_CTX_INPUT_CAR_PARKED = 0x03,
    PARK_RTI_USER_CTX_INPUT_CAR_LEAVING = 0x04,
    PARK_RTI_USER_CTX_INPUT_REINITIALIZE = 0x63, // 99
    PARK_RTI_USER_CTX_INPUT_RECALIBRATE = PARK_RTI_USER_CTX_INPUT_NO_CAR,
} PNI_ParkingRtiUserCtxInput;

typedef enum pni_park_rti_user_trigger_flag {
    PARK_RTI_USER_TRIG_FLAG_NONE =           0,
    PARK_RTI_USER_TRIG_FLAG_ONE_BIT =        1,
    PARK_RTI_USER_TRIG_FLAG_MAINT =          2, // Pattern A - BLE on
    PARK_RTI_USER_TRIG_FLAG_LORA_ON =        3, // Pattern B - Lora on(not use by Host, done by SENtral)
    PARK_RTI_USER_TRIG_FLAG_HW_RESET =       4, // Pattern C - reset
    PARK_RTI_USER_TRIG_FLAG_SHIPPING =       5, // Pattern D - shipping
    PARK_RTI_USER_TRIG_FLAG_MFG =            6, // Pattern E - manufacturing
} PNI_ParkingRtiUserTriggerFlag;

#pragma pack(push, 1)
typedef struct pni_park_rti_version {
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
    uint8_t other;
    uint32_t build;
} PNI_ParkingRtiVersion;
#pragma pack(pop)

void pni_park_sns_rti_init(PNI_ParkingSensorIface *iface, SENtralA2Iface *rti);

#endif /* PNI_PARKING_SENSOR_RTI_H */
