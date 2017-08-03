/**
* @file         SENtralA2_types.h
*
* @brief        Types and struct definitions for SENTralA2.
*
* @date         08/09/2015
* @copyright    (C) 2016 PNI Corp
*
*               THIS SOFTWARE IS PROVIDED BY PNI SENSOR CORPORATION "AS IS" AND
*               ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
*               TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
*               PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL PNI SENSOR
*               CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*               SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
*               NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*               LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
*               HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
*               CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
*               OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
*               EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*               DISCLOSURE TO THIRD PARTIES OR REPRODUCTION IN ANY FORM
*               WHATSOEVER, WITHOUT PRIOR WRITTEN CONSENT, IS STRICTLY
*               FORBIDDEN.
*
*/
#ifndef SENTRAL_A2_TYPES_H
#define SENTRAL_A2_TYPES_H

#include <stdint.h>

#include "i2c.h"

// SENtral-A2 interface            change here if different
#define SENTRAL_A2_I2C                hi2c1
#define SENTRAL_A2_DRDY_PIN           SENSOR_IRQ_Pin
#define SENTRAL_A2_DRDY_PORT          SENSOR_IRQ_GPIO_Port
#define SENTRAL_A2_I2C_TIMEOUT        (1000U)

//Macros
#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))
#define PI 3.1415927f

#define MAX_FIFO_SIZE                                   100 * 1024

#define  SENTRAL_A2_ADDRESS                             (0x50)
#define  SENTRAL_A2_PROD_ID                        			0x86    //A2

#define MAX_EEPROM_I2C_WRITE			                      64
#define I2C_MAX_READ			                              64//16//8
#define DEFAULT_I2C_SPEED_KHZ                           400

#define PNI_YES                                         1
#define PNI_NO                                          0

typedef enum sentral_a2_ret {
    SENA2_RET_ERROR = 0,
    SENA2_RET_OK = 1,
} SENtralA2Ret;

// EEPROM
#define EEPROM_ADDRESS                                  0xA0
/**< number of bytes to write to EEPROM at a time; allocated on the stack */
#define EEPROM_BUF_LEN                                  64
/**< number of bytes to read from config file at a time while uploading to RAM; allocated on the stack; must be 4 */
#define CONFIG_FILE_BUF_LEN                             32

#define SENTRAL_UUID_SIZE                               8

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

//CHIP_STATUS_REG bits
#define EEDETECTED			(0x01)
#define EEUPLOADDONE			(0x02)
#define EEUPLOADERR			(0x04)
#define FWIDLE				(0x08)
#define NOEEPROM			(0x10)

// FIRMWARE CONSTANTS
#define FIRMWARE_HEADER_SIZE                            16
#define MAX_FIRMWARE_SIZE								                32768
#define IMAGE_SIGNATURE_LSB                             0x2A
#define IMAGE_SIGNATURE_MSG                             0x65

// REGISTER MAP
#define FIFO_FLUSH_REG                                  0x32 // flush sensor data from fifo
#define CHIP_CONTROL_REG                                0x34 // enable cpu
#define HOST_STATUS_REG                                 0x35 // host status
#define INT_STATUS_REG                                  0x36 // interrupt status
#define CHIP_STATUS_REG                                 0x37 // chip status
#define BYTES_REMANING_REG                              0x38 // LSB of remaing bytes in FIFO
#define PARAM_ACK_REG                                   0x3A
#define PARAM_SAVE_REG                                  0x3B
#define ERR_REG                                         0x50 // error register
#define INT_STATE_REG                                   0x51 // interrupt state
#define DEBUG_VAL_REG                                   0x52 // debug value
#define DEBUG_STATE_REG                                 0x53 // debug state
#define PARAM_PAGE_SELECT_REG                           0x54 // parameter page select (and transfer size)
#define HOST_INTERFACE_CTRL_REG                         0x55
#define PARAM_LOAD_REG                                  0x5C
#define PARAM_REQUEST_REG                               0x64
#define HOST_IRQ_TIME_REG                               0x6C
#define ROM_VERSION_REG                                 0x70
#define PRODUCT_ID_REG                                  0x90 // Product ID
#define REVISION_ID_REG                                 0x91 // Product ID
#define SR_UPLOAD_DATA_REG                              0x96 // Firmware upload address
#define HOST_CRC_REG                                    0x97 // Firmware upload CRC
#define RESET_REQ_REG                                   0x9B // Request system reset
#define PASS_THROUGH_RDY_REG                            0x9E
#define SCL_LOW_CYCLES_REG                              0x9F
#define PASS_THROUGH_CFG_REG                            0xA0

// FIFO FLUSH
#define FIFO_FLUSH_DISCARD_NON_WAKE                     0xFA
#define FIFO_FLUSH_DISCARD_WAKE                         0xFB
#define FIFO_FLUSH_TRANSFER_NON_WAKE                    0xFC
#define FIFO_FLUSH_TRANSFER_WAKE                        0xFD
#define FIFO_FLUSH_DISCARD_ALL                          0xFE
#define FIFO_FLUSH_TRANSFER_ALL                         0xFF

// CHIP CONTROL
#define CHIP_CONTROL_CPU_RUN                            0x01
#define CHIP_CONTROL_HOST_UPLOAD                        0x02

// HOST STATUS
#define HOST_STATUS_RESET                               0X01
#define HOST_STATUS_ALGORITHM_STANDBY                   0x02
#define HOST_STATUS_INTERFACE_BITS                      0x1C
#define HOST_STATUS_INTERFACE_K                         0x00
#define HOST_STATUS_INTERFACE_L                         0x04
#define HOST_STATUS_INTERFACE_L_EXTENDED                0x08
#define HOST_STATUS_ALGORITHM_BITS                      0xE0
#define HOST_STATUS_ALGORITHM_SPACEPOINT                0x20

// HOST INTERFACE CONTROL
#define HOST_IFACE_CTRL_FLAG_ALGORITHM_STANDBY          0x01
#define HOST_IFACE_CTRL_FLAG_ABORT_TRANSFER             0x02
#define HOST_IFACE_CTRL_FLAG_UPDATE_TRANSFER_CNT        0x04
#define HOST_IFACE_CTRL_FLAG_WAKE_FIFO_INT_DISABLE      0x08
#define HOST_IFACE_CTRL_FLAG_NED_COORDINATES            0x10
#define HOST_IFACE_CTRL_FLAG_AP_SUSPEND                 0x20
#define HOST_IFACE_CTRL_FLAG_REQ_SENSOR_SELF_TEST       0x40
#define HOST_IFACE_CTRL_FLAG_NON_WAKE_FIFO_INT_DISABLE  0x80

// PARAM IO PAGES
#define PARAM_PAGE_SYSTEM                               1
#define PARAM_PAGE_WARM_START                           2
#define PARAM_PAGE_SENSOR_INFO                          3
#define PARAM_PAGE_SENSOR_CONF                          5
#define PARAM_PAGE_KNOBS                                13
#define PARAM_PAGE_GPS_DATA                             14
#define PARAM_PAGE_FW_VER                               15

// SYSTEM PARAMETERS
#define PARAM_META_EVENT_CONTROL                        1
#define PARAM_FIFO_CONTROL                              2
#define PARAM_SENSOR_STATUS_BANK_0                      3
#define SENSORS_PER_STATUS_BANK                         16
#define PARAM_WAKE_META_EVENT_CONTROL                   29
#define PARAM_HOST_IRQ_TIMESTAMP                        30
#define PARAM_PHYSICAL_SENSOR_STATUS                    31


// WARM_START PARAMETERS
#define PARAM_RAW_SENSOR_CONTROL                        127

#define RAW_ACCEL_CONTROL_BIT                           0x01
#define RAW_MAG_CONTROL_BIT                             0x02
#define RAW_GYRO_CONTROL_BIT                            0x04

#define SENSOR_TYPE_ACCELEROMETER                       1
#define SENSOR_TYPE_MAGNETIC_FIELD                      2
#define SENSOR_TYPE_ORIENTATION                         3
#define SENSOR_TYPE_GYROSCOPE                           4
#define SENSOR_TYPE_LIGHT                               5
#define SENSOR_TYPE_PRESSURE                            6
#define SENSOR_TYPE_TEMPERATURE                         7
#define SENSOR_TYPE_PROXIMITY                           8
#define SENSOR_TYPE_GRAVITY                             9
#define SENSOR_TYPE_LINEAR_ACCELERATION                 10
#define SENSOR_TYPE_ROTATION_VECTOR                     11
#define SENSOR_TYPE_RELATIVE_HUMIDITY                   12
#define SENSOR_TYPE_AMBIENT_TEMPERATURE                 13
#define SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED         14
#define SENSOR_TYPE_GAME_ROTATION_VECTOR                15
#define SENSOR_TYPE_GYROSCOPE_UNCALIBRATED              16
#define SENSOR_TYPE_SIGNIFICANT_MOTION                  17
#define SENSOR_TYPE_STEP_DETECTOR                       18
#define SENSOR_TYPE_STEP_COUNTER                        19
#define SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR         20
#define SENSOR_TYPE_HEART_RATE                          21

#define SENSOR_TYPE_CAR_DETECTOR                        21
#define SENSOR_TYPE_TILT_DETECTOR                       22
#define SENSOR_TYPE_WAKE_GESTURE                        23
#define SENSOR_TYPE_GLANCE_GESTURE                      24
#define SENSOR_TYPE_PICK_UP_GESTURE                     25
#define SENSOR_TYPE_RAW_MAGNETIC_FIELD					29
#define SENSOR_TYPE_ACTIVITY                            31
#define SENSOR_TYPE_CAR_MAG_DATA						32
#define SENSOR_TYPE_CAR_PARAMETER_REPORT				33
#define SENSOR_TYPE_VISIBLE_END							            63
// SENtrace virtual sensors
#define SENSOR_TYPE_ALTITUDE_FUSION                     21
#define SENSOR_TYPE_ALTITUDE                            22
#define SENSOR_TYPE_PDR                                 23
#define SENSOR_TYPE_PDR_DISTANCE                        24
#define SENSOR_TYPE_PDR_PACE                            25
#define SENSOR_TYPE_PDR_PATH                            26
#define SENSOR_TYPE_PDR_EST_POSITION                    27
#define SENSOR_TYPE_PDR_PRE_POSITION                    28
#define SENSOR_TYPE_GPS_REPORT                          29
#define SENSOR_TYPE_GPS_REPORT_2                        30
// FITNESS
#define SENSOR_TYPE_FITNESS_ACTIVITY                    29

#define SENSOR_TYPE_WAKE_OFFTSET                        64

#define SENSOR_TYPE_ACCELEROMETER_WAKE                  65
#define SENSOR_TYPE_MAGNETIC_FIELD_WAKE                 66
#define SENSOR_TYPE_ORIENTATION_WAKE                    67
#define SENSOR_TYPE_GYROSCOPE_WAKE                      68
#define SENSOR_TYPE_LIGHT_WAKE                          69
#define SENSOR_TYPE_PRESSURE_WAKE                       70
#define SENSOR_TYPE_TEMPERATURE_WAKE                    71
#define SENSOR_TYPE_PROXIMITY_WAKE                      72
#define SENSOR_TYPE_GRAVITY_WAKE                        73
#define SENSOR_TYPE_LINEAR_ACCELERATION_WAKE            74
#define SENSOR_TYPE_ROTATION_VECTOR_WAKE                75
#define SENSOR_TYPE_RELATIVE_HUMIDITY_WAKE              76
#define SENSOR_TYPE_AMBIENT_TEMPERATURE_WAKE            77
#define SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED_WAKE    78
#define SENSOR_TYPE_GAME_ROTATION_VECTOR_WAKE           79
#define SENSOR_TYPE_GYROSCOPE_UNCALIBRATED_WAKE         80
#define SENSOR_TYPE_SIGNIFICANT_MOTION_WAKE             81
#define SENSOR_TYPE_STEP_DETECTOR_WAKE                  82
#define SENSOR_TYPE_STEP_COUNTER_WAKE                   83
#define SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR_WAKE    84
#define SENSOR_TYPE_HEART_RATE_WAKE                     85
#define SENSOR_TYPE_TILT_DETECTOR_WAKE                  86
#define SENSOR_TYPE_WAKE_GESTURE_WAKE                   87
#define SENSOR_TYPE_GLANCE_GESTURE_WAKE                 88
#define SENSOR_TYPE_PICK_UP_GESTURE_WAKE                89
#define SENSOR_TYPE_ACTIVITY_WAKE                       95
// SENtrace virtual sensors
#define SENSOR_TYPE_ALTITUDE_FUSION_WAKE                85
#define SENSOR_TYPE_ALTITUDE_WAKE                       86
#define SENSOR_TYPE_PDR_WAKE                            87
#define SENSOR_TYPE_PDR_DISTANCE_WAKE                   88
#define SENSOR_TYPE_PDR_PACE_WAKE                       89
#define SENSOR_TYPE_PDR_PATH_WAKE                       90
#define SENSOR_TYPE_PDR_EST_POSITION_WAKE               91
#define SENSOR_TYPE_PDR_PRE_POSITION_WAKE               92
#define SENSOR_TYPE_GPS_REPORT_WAKE                     93
#define SENSOR_TYPE_GPS_REPORT_2_WAKE                   94



#define SENSOR_TYPE_DEBUG                               245
#define SENSOR_TYPE_TIMESTAMP_WAKE                      246
#define SENSOR_TYPE_TIMESTAMP_OVERFLOW_WAKE             247
#define SENSOR_TYPE_META_WAKE                           248
#define SENSOR_TYPE_RAW_GYRO                            249
#define SENSOR_TYPE_RAW_MAG                             250
#define SENSOR_TYPE_RAW_ACCEL                           251
#define SENSOR_TYPE_TIMESTAMP                           252
#define SENSOR_TYPE_TIMESTAMP_OVERFLOW                  253
#define SENSOR_TYPE_META                                254
#define SENSOR_TYPE_MAX                                 255

#define META_EVENT_FLUSH_COMPLETE                       1
#define META_EVENT_SAMPLE_RATE_CHANGED                  2
#define META_EVENT_POWER_MODE_CHANGED                   3
#define META_EVENT_ERROR                                4
#define META_EVENT_MAG_TRANSIENT_CHANGED                5
#define META_EVENT_CAL_STATUS_CHANGED                   6
#define META_EVENT_STILLNESS_CHANGED                    7
#define META_EVENT_CALIBRATION_STABLE                   9
#define META_EVENT_ACCEL_CAL                            10
#define META_EVENT_SENSOR_EVENT                         11
#define META_EVENT_FIFO_OVERFLOW                        12
#define META_EVENT_DYNAMIC_RANGE_CHANGED                13
#define META_EVENT_FIFO_WATERMARK                       14
#define META_EVENT_SELF_TEST_RESULT                     15
#define META_EVENT_INITIALIZED                          16
#define META_EVENT_TRANSFER_CAUSE                       17


typedef struct
{
  uint8_t  major;
  uint8_t  minor;
  uint8_t  patch;
  uint8_t  other;
  uint32_t build;
} FwVersion;

typedef struct
{
	uint8_t paramNo;
	uint8_t size;
} ParamInfo;

#pragma pack(push, 1)
typedef struct SENtralA2_meta_event {
    uint8_t id;
    union {
        uint16_t value;
        uint8_t bytes[2];
    };
} SENtralA2_MetaEvent;
#pragma pack(pop)

#define SENA2_FIFO_DATA_SIZE (16)

#pragma pack(push, 1)
typedef struct SENtralA2_fifo_data {
    union {
        uint16_t stime;
        SENtralA2_MetaEvent meta_event;
        uint8_t bytes[SENA2_FIFO_DATA_SIZE];
    };
} SENtralA2_FifoData;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct SENtralA2_sns_packet {
    uint8_t sid;
    uint8_t count;
    SENtralA2_FifoData data;
    uint32_t timestamp;
} SENtralA2_SensorPacket;
#pragma pack(pop)

typedef union{
	/**
	 * \brief Direct access to all flags.
	 */
	uint16_t value;                                                     /**< the 16 bit register value for the EEPROM flags register */
	struct bits
	{
		/** \brief Do not execute the EEPROM image immediately after upload */
		uint16_t   EEPROMNoExec : 1;
		/** \brief Reserved */
		uint16_t   Reserved : 7;
		/** \brief The clock speed to upload the firmware at */
		uint16_t   I2CClockSpeed : 3;
		/** \brief Rom version expected */
		uint16_t   ROMVerExp : 4;
		/** \brief Reserved */
		uint16_t   reserved1 : 1;
	} bits;                                                             /**< the bit fields within the EEPROM flags register */
}FwFlags;

typedef struct
{
	uint8_t imageSignatureLsb;
	uint8_t imageSignatureMsb;
	FwFlags flags;
	uint32_t crc;
	uint32_t reserved;
	uint16_t imageLength;
	uint16_t reserved2;
} FirmwareHeader;

typedef struct sentral_a2_meta_event_cb {
    void (*on_flush_complete)(void *self, uint8_t sid);
    void (*on_sample_rate_changed)(void *self, uint8_t sid);
    void (*on_power_mode_changed)(void *self, uint8_t sid, uint8_t mode);
    void (*on_error)(void *self, uint8_t error_reg, uint8_t debug_st);
    void (*on_sensor_error)(void *self, uint8_t sid, uint8_t status);
    void (*on_fifo_overflow)(void *self, uint16_t count);
    void (*on_dynamic_range_changed)(void *self, uint8_t sid);
    void (*on_fifo_watermark)(void *self, uint16_t bytes);
    void (*on_self_test_results)(void *self, uint8_t sid, uint8_t result);
    void (*on_initialized)(void *self, uint16_t ram_ver);
    void (*on_xfer_cause)(void *self, uint8_t sid);
    void *params;
} SENtralA2_MetaEvent_Callback;

typedef struct sentral_a2_iface {
    int (*set_parameter)(void *self, uint8_t page, uint8_t num, uint8_t *data,
            uint8_t size);
    int (*get_parameter)(void *self, uint8_t page, uint8_t num, uint8_t *data,
            uint8_t size);
    int (*set_sns_rate)(void *self, uint8_t sid, uint16_t rate);
    int (*self_test)(void *self);
    int (*set_meta_event_ctrl)(void *self, uint64_t nonwake, uint64_t wake);
    SENtralA2_MetaEvent_Callback *meta_event_cb;
    uint8_t event_sizes[SENSOR_TYPE_MAX];
    uint32_t stime_wk;
    uint32_t stime_nw;
} SENtralA2Iface;

/**
* Sensor names indexed by ID.
*/
const static char *SENtralA2_sensor_name[256] =
{
	"na",										// 0
	"accelerometer",							// 1
	"magnetic field",							// 2
	"orientation",								// 3
	"gyroscope",								// 4
	"light",									// 5
	"pressure",									// 6
	"temperature",								// 7
	"proximity",								// 8
	"gravity",									// 9
	"linear acceleration",						// 10
	"rotation vector",							// 11
	"relative humidity",						// 12
	"ambient temperature",						// 13
	"magnetic field uncalibrated",				// 14
	"game rotation vector",						// 15
	"gyroscope uncalibrated",					// 16
	"significant motion",						// 17
	"step detector",							// 18
	"step counter",								// 19
	"geomagnetic rotation vector",				// 20
	"parking",								// 21
	"tilt detector",							// 22
	"pdr",								// 23
	"pdr distance",							// 24
	"pdr pace",							// 25
	"pdr path",								// 26
	"custom_27",								// 27
	"custom_28",								// 28
	"custom_29",								// 29
	"custom_30",								// 30
	"activity",									// 31
	"custom_32",								// 32
	"custom_33",								// 33
	"custom_34",								// 34
	"custom_35",								// 35
	"custom_36",								// 36
	"custom_37",								// 37
	"custom_38",								// 38
	"custom_39",								// 39
	"custom_40",								// 40
	"custom_41",								// 41
	"custom_42",								// 42
	"custom_43",								// 43
	"custom_44",								// 44
	"custom_45",								// 45
	"custom_46",								// 46
	"custom_47",								// 47
	"custom_48",								// 48
	"custom_49",								// 49
	"custom_50",								// 50
	"custom_51",								// 51
	"custom_52",								// 52
	"custom_53",								// 53
	"custom_54",								// 54
	"custom_55",								// 55
	"custom_56",								// 56
	"custom_57",								// 57
	"custom_58",								// 58
	"custom_59",								// 59
	"custom_60",								// 60
	"custom_61",								// 61
	"custom_62",								// 62
	"custom_63",								// 63
	"reserved",									// 64
	"accelerometer wake",						// 65
	"magnetic field wake",						// 66
	"orientation wake",							// 67
	"gyroscope wake",							// 68
	"light wake",								// 69
	"pressure wake",							// 70
	"temperature wake",							// 71
	"proximity wake",							// 72
	"gravity wake",								// 73
	"linear acceleration wake",					// 74
	"rotation vector wake",						// 75
	"relative humidity wake",					// 76
	"ambient temperature wake",					// 77
	"magnetic field uncalibrated wake",			// 78
	"game rotation vector wake",				// 79
	"gyroscope uncalibrated wake",				// 80
	"significant motion wake",					// 81
	"step detector wake",						// 82
	"step counter wake",						// 83
	"geomagnetic rotation vector wake",			// 84
	"heart rate wake",							// 85
	"tilt detector wake",						// 86
	"wake gesture wake",						// 87
	"glance gesture wake",						// 88
	"pick up gesture wake",						// 89
	"custom_26 wake",							// 90
	"custom_27 wake",							// 91
	"custom_28 wake",							// 92
	"custom_29 wake",							// 93
	"custom_30 wake",							// 94
	"activity wake",							// 95
	"custom_32 wake",							// 96
	"custom_33 wake",							// 97
	"custom_34 wake",							// 98
	"custom_35 wake",							// 99
	"custom_36 wake",							// 100
	"custom_37 wake",							// 101
	"custom_38 wake",							// 102
	"custom_39 wake",							// 103
	"custom_40 wake",							// 104
	"custom_41 wake",							// 105
	"custom_42 wake",							// 106
	"custom_43 wake",							// 107
	"custom_44 wake",							// 108
	"custom_45 wake",							// 109
	"custom_46 wake",							// 110
	"custom_47 wake",							// 111
	"custom_48 wake",							// 112
	"custom_49 wake",							// 113
	"custom_50 wake",							// 114
	"custom_51 wake",							// 115
	"custom_52 wake",							// 116
	"custom_53 wake",							// 117
	"custom_54 wake",							// 118
	"custom_55 wake",							// 119
	"custom_56 wake",							// 120
	"custom_57 wake",							// 121
	"custom_58 wake",							// 122
	"custom_59 wake",							// 123
	"custom_60 wake",							// 124
	"custom_61 wake",							// 125
	"custom_62 wake",							// 126
	"custom_63 wake",							// 127
	"reserved_0",								// 128
	"reserved_1",								// 129
	"reserved_2",								// 130
	"reserved_3",								// 131
	"reserved_4",								// 132
	"reserved_5",								// 133
	"reserved_6",								// 134
	"reserved_7",								// 135
	"reserved_8",								// 136
	"reserved_9",								// 137
	"reserved_10",								// 138
	"reserved_11",								// 139
	"reserved_12",								// 140
	"reserved_13",								// 141
	"reserved_14",								// 142
	"reserved_15",								// 143
	"reserved_16",								// 144
	"reserved_17",								// 145
	"reserved_18",								// 146
	"reserved_19",								// 147
	"reserved_20",								// 148
	"reserved_21",								// 149
	"reserved_22",								// 150
	"reserved_23",								// 151
	"reserved_24",								// 152
	"reserved_25",								// 153
	"reserved_26",								// 154
	"reserved_27",								// 155
	"reserved_28",								// 156
	"reserved_29",								// 157
	"reserved_30",								// 158
	"reserved_31",								// 159
	"reserved_32",								// 160
	"reserved_33",								// 161
	"reserved_34",								// 162
	"reserved_35",								// 163
	"reserved_36",								// 164
	"reserved_37",								// 165
	"reserved_38",								// 166
	"reserved_39",								// 167
	"reserved_40",								// 168
	"reserved_41",								// 169
	"reserved_42",								// 170
	"reserved_43",								// 171
	"reserved_44",								// 172
	"reserved_45",								// 173
	"reserved_46",								// 174
	"reserved_47",								// 175
	"reserved_48",								// 176
	"reserved_49",								// 177
	"reserved_50",								// 178
	"reserved_51",								// 179
	"reserved_52",								// 180
	"reserved_53",								// 181
	"reserved_54",								// 182
	"reserved_55",								// 183
	"reserved_56",								// 184
	"reserved_57",								// 185
	"reserved_58",								// 186
	"reserved_59",								// 187
	"reserved_60",								// 188
	"reserved_61",								// 189
	"reserved_62",								// 190
	"reserved_63",								// 191
	"reserved_64",								// 192
	"reserved_65",								// 193
	"reserved_66",								// 194
	"reserved_67",								// 195
	"reserved_68",								// 196
	"reserved_69",								// 197
	"reserved_70",								// 198
	"reserved_71",								// 199
	"reserved_72",								// 200
	"reserved_73",								// 201
	"reserved_74",								// 202
	"reserved_75",								// 203
	"reserved_76",								// 204
	"reserved_77",								// 205
	"reserved_78",								// 206
	"reserved_79",								// 207
	"reserved_80",								// 208
	"reserved_81",								// 209
	"reserved_82",								// 210
	"reserved_83",								// 211
	"reserved_84",								// 212
	"reserved_85",								// 213
	"reserved_86",								// 214
	"reserved_87",								// 215
	"reserved_88",								// 216
	"reserved_89",								// 217
	"reserved_90",								// 218
	"reserved_91",								// 219
	"reserved_92",								// 220
	"reserved_93",								// 221
	"reserved_94",								// 222
	"reserved_95",								// 223
	"reserved_96",								// 224
	"reserved_97",								// 225
	"reserved_98",								// 226
	"reserved_99",								// 227
	"reserved_100",								// 228
	"reserved_101",								// 229
	"reserved_102",								// 230
	"reserved_103",								// 231
	"reserved_104",								// 232
	"reserved_105",								// 233
	"reserved_106",								// 234
	"reserved_107",								// 235
	"reserved_108",								// 236
	"reserved_109",								// 237
	"reserved_110",								// 238
	"reserved_111",								// 239
	"reserved_112",								// 240
	"reserved_113",								// 241
	"reserved_114",								// 242
	"reserved_115",								// 243
	"reserved_116",								// 244
	"debug",									// 245
	"timestamp lsw wake",						// 246
	"timestamp msw wake",						// 247
	"meta_event wake",							// 248
	"gyroscope raw",							// 249
	"magnetic field raw",						// 250
	"accelerometer raw",						// 251
	"timestamp lsw",							// 252
	"timestamp msw",							// 253
	"meta event",								// 254
	"reserved_127",								// 255
};
#endif // SENTRAL_A2_TYPES_H
