#ifndef PNI_PARKING_H
#define PNI_PARKING_H

#include <stdint.h>

typedef enum pni_park_ret {
    PNI_PARKING_RET_ERROR = -1,
    PNI_PARKING_RET_OK = 0,
} PNI_ParkingRet;

typedef enum pni_park_sns_flag {
    PARK_SNS_FLAG_CAR_DETECTOR_ENABLED =      (1U << 0),
    PARK_SNS_FLAG_CAR_DETECTOR_DATA_ENABLED = (1U << 1),
    PARK_SNS_FLAG_PARAM_REPORT_ENABLED =      (1U << 2),
} PNI_ParkingSensorFlag;

typedef enum pni_park_cd_flag {
    PARK_CD_FLAG_CONTINUOUS_MODE_ENABLED = (1U << 0),
    PARK_CD_FLAG_INTER_STATE_ENABLED =     (1U << 1),
} PNI_ParkingCarDetectorFlag;

typedef enum pni_park_sns_type {
    PARK_SNS_TYPE_CFG_STATE = 0x30,
    PARK_SNS_TYPE_VERSION =   0x31,
    PARK_SNS_TYPE_PARAMETER = 0x32,
    PARK_SNS_TYPE_PONG =      0x33,
    PARK_SNS_TYPE_FW_VERSION = 0x34,
    PARK_SNS_TYPE_PARAM_IO = 0x35,
} PNI_ParkingSensorType;

typedef enum pni_park_cmd {
    PARK_CMD_RECALIBRATE =                 0x01,
    PARK_CMD_FORCE_OCCUPIED =              0x02,
    PARK_CMD_FORCE_VACANT =                0x03,
    PARK_CMD_SELF_TEST =                   0x04,
    PARK_CMD_INTERMEDIATE_REPORT_ENABLE =  0x05,
    PARK_CMD_INTERMEDIATE_REPORT_DISABLE = 0x06,
    PARK_CMD_SET_WAKEUP_INTERVAL =         0x07,
    PARK_CMD_CAR_DETECTOR_DATA_ENABLE =    0x08,
    PARK_CMD_CAR_DETECTOR_DATA_DISABLE =   0x09,
    PARK_CMD_SET_PARAMETER =               0x0A,
    PARK_CMD_GET_PARAMETER =               0x0B,
    PARK_CMD_FULL_MAG_DATA_REPORT_START =  0x0C,
    PARK_CMD_PING =                        0x0D,
    PARK_CMD_ALARM_A_INTERVAL =            0x0E,  /* temperature wakeup interval */
    PARK_CMD_MFG_WAKE_BLE =                0x10,  /* manufacture wake BLE */
    PARK_CMD_RADIO_TXP =                   0x11,  /* radio transmit power */
    PARK_CMD_RESET =                       0x3F,
} PNI_ParkingCommand;

typedef enum pni_park_car_det_flag {
    CAR_DET_FLAG_CLEARED =                 0x00,
    CAR_DET_FLAG_PASS_TRIGGER_THRESHOLD =  0x01,
    CAR_DET_FLAG_BLE_WAKE_UP =             0x02,
    CAR_DET_FLAG_DISABLE_SHIPPING_MODE =   0x03,
    CAR_DET_FLAG_HARD_RESET =              0x04,
    CAR_DET_FLAG_SHIPPING_MODE =           0x05,
    CAR_DET_FLAG_MFG_MODE =                0x06,
} PNI_ParkingCarDetFlag;
#define DETECT_PATTERN(flag, pattern) (((flag >> 5) & 0x7) == pattern)? 1 : 0

#pragma pack(push, 1)
typedef struct pni_park_data_car_detector {
    uint8_t car_presence;
    uint8_t flags;
    float temperature;
    float battery;
} PNI_ParkingDataCarDetector;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct pni_park_data_car_detector_data {
    float x, y, z;
} PNI_ParkingDataCarDetectorData;
#pragma pack(pop)

/* Legacy version structures */
#pragma pack(push, 1)
typedef struct pni_park_host_version {
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
} PNI_ParkingHostVersion;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct pni_park_sns_version {
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
    uint8_t other;
    uint32_t build;
} PNI_ParkingSensorVersion;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct pni_park_version_info {
    PNI_ParkingHostVersion host;
    PNI_ParkingSensorVersion sensor;
} PNI_ParkingVersionInfo;
#pragma pack(pop)

/* New version */
typedef enum pni_park_version_id {
    PARK_VID_HOST = 0x01,
    PARK_VID_SENSOR = 0x02,
    PARK_VID_BLE = 0x03,
    PARK_VID_LORA = 0x04,
} PNI_ParkingVersionId;

#pragma pack(push, 1)
typedef struct pni_park_version {
    uint8_t vid;
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
    uint32_t build;
} PNI_ParkingVersion;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct pni_park_meta_event {
    uint8_t type;
    uint8_t data[2];
} PNI_ParkingMetaEvent;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct pni_park_pong {
    float rssi;
    float snr;
} PNI_ParkingPong;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct pni_park_param_io {
    uint8_t page;
    uint8_t param;
    float value;
} PNI_ParkingParamIo;
#pragma pack(pop)

#define PARK_PKT_DATA_SIZE (12)

#pragma pack(push, 1)
typedef struct pni_park_pkt {
    uint8_t sid;
    union {
        PNI_ParkingDataCarDetector car_detector;
        PNI_ParkingDataCarDetectorData car_detector_data;
        PNI_ParkingVersion version; /* new */
        PNI_ParkingVersionInfo version_info; /* legacy */
        PNI_ParkingMetaEvent meta_event;
        PNI_ParkingPong pong;
        PNI_ParkingParamIo param_io;
        uint8_t data[PARK_PKT_DATA_SIZE];
    };
    uint32_t timestamp;
} PNI_ParkingPacket;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct pni_park_cfg_state {
    uint8_t sns_flags;
    uint8_t car_detector_flags;
    uint16_t wakeup_interval;
} PNI_ParkingCfgState;
#pragma pack(pop)

/* BLE configuration by manufacture */
#define EEPROM_U16_BLE_FLAGS                  ((uint16_t *) 0x08080000)
#define BLE_FLAG_PUBLIC_MAC                   (1)
#define BLE_FLAG_RESERVED1                    (1 << 1)
#define BLE_FLAG_RESERVED2                    (1 << 2)
#define BLE_FLAG_RESERVED3                    (1 << 3)

#define EEPROM_U8_BLE_PUBLIC_MAC              ((uint8_t *)  0x08080002)
#define EEPROM_U32_BLE_RESERVED1              ((uint32_t *) 0x08080008)
#define EEPROM_U32_BLE_RESERVED2              ((uint32_t *) 0x08080010)
#define EEPROM_U32_BLE_RESERVED3              ((uint32_t *) 0x08080017)

#define CHECK_BLE_FLAG_SET(BleFlagBit) \
                                ((EEPROM_U16_BLE_FLAGS[0]&(BleFlagBit)) ? 1 : 0)

/* XDot configuration by manufacture */
#define EEPROM_U32_XDOT_FLAGS                 ((uint32_t *) 0x08080020)
#define XDOT_FLAG_MFG_EUI                     (1)
#define XDOT_FLAG_PROD_EUI                    (1 << 1)
#define XDOT_FLAG_PROD_NET_KEY                (1 << 2)
#define XDOT_FLAG_PROD_CHANGE_CFG             (1 << 3)

#define EEPROM_U8_XDOT_MFG_EUI                ((uint8_t *) 0x08080028)
#define EEPROM_U8_XDOT_PROD_EUI               ((uint8_t *) 0x08080030)
#define EEPROM_U8_XDOT_PROD_NET_KEY           ((uint8_t *) 0x08080038)
#define EEPROM_U32_XDOT_PROD_CFG_ISM_BAND     ((uint32_t *) 0x08080048)
#define EEPROM_U32_XDOT_PROD_CFG_SUB_BAND     ((uint32_t *) 0x08080050)
#define EEPROM_U32_XDOT_PROD_CFG_SF           ((uint32_t *) 0x08080058)
#define EEPROM_U32_XDOT_PROD_CFG_RESERVE1     ((uint32_t *) 0x08080060)
#define EEPROM_U32_XDOT_PROD_CFG_RESERVE2     ((uint32_t *) 0x08080068)
#define EEPROM_U32_XDOT_PROD_CFG_RESERVE3     ((uint32_t *) 0x08080070)
#define EEPROM_U32_XDOT_PROD_CFG_RESERVE4     ((uint32_t *) 0x08080078)

#define CHECK_XDOT_FLAG_SET(XdotFlagBit) \
                              ((EEPROM_U32_XDOT_FLAGS[0]&(XdotFlagBit)) ? 1 : 0)

#endif /* PNI_PARKING_H */
