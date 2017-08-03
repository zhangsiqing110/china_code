#ifndef MCP9808_H
#define MCP9808_H

#include <stdint.h>

/*
 * Registers
 */
typedef enum mcp9808_register {
    MCP9808_REG_RFU =              0x00,
    MCP9808_REG_CFG =              0x01,
    MCP9808_REG_ALERT_TEMP_UPPER = 0x02,
    MCP9808_REG_ALERT_TEMP_LOWER = 0x03,
    MCP9808_REG_CRIT_TEMP =        0x04,
    MCP9808_REG_TEMP =             0x05,
    MCP9808_REG_MFG_ID =           0x06,
    MCP9808_REG_DEV_ID =           0x07,
    MCP9808_REG_RES =              0x08,
} MCP9808_Register;

/*
 * Config register flags
 */
typedef enum mcp9808_cfg_flag {
    MCP9808_CFG_FLAG_ALERT_MODE_INT =      (1 << 0),
    MCP9808_CFG_FLAG_ALERT_POL_HIGH =      (1 << 1),
    MCP9808_CFG_FLAG_ALERT_CRIT_ONLY =     (1 << 2),
    MCP9808_CFG_FLAG_ALERT_ENABLE =        (1 << 3),
    MCP9808_CFG_FLAG_ALERT_ASSERT_ENABLE = (1 << 4),
    MCP9808_CFG_FLAG_INT_CLR =             (1 << 5),
    MCP9808_CFG_FLAG_WIN_LOCKED =          (1 << 6),
    MCP9808_CFG_FLAG_CRIT_LOCKED =         (1 << 7),
    MCP9808_CFG_FLAG_SHDN_ENABLED =        (1 << 8),
} MCP9808_ConfigFlag;

/*
 * Resolution
 */
typedef enum mcp9808_res {
    MCP9808_RES_0_5 = 0x00,
    MCP9808_RES_0_25 = 0x01,
    MCP9808_RES_0_125 = 0x02,
    MCP9808_RES_0_0625 = 0x03, /* power-on default */
    MCP9808_RES_MASK = MCP9808_RES_0_0625,
} MCP9808_Resolution;

/*
 * Temperature conversion time (ms)
 */
typedef enum mcp9808_tconv_ms {
    MCP9808_TCONV_MS_0_5 = 30,
    MCP9808_TCONV_MS_0_25 = 65,
    MCP9808_TCONV_MS_0_125 = 130,
    MCP9808_TCONV_MS_0_0625 = 250,
} MCP9808_TconvMs;

/*
 * Manufacturer ID from MCP9808_REG_MFG_ID register
 */
#define MCP9808_MFG_ID (0x0054)

/*
 * Device ID from MCP9808_REG_DEV_ID register
 */
#define MCP9808_DEV_ID (0x0400)

/*
 * Return codes
 */
typedef enum mcp9808_ret {
    MCP9808_RET_ERROR = -1,
    MCP9808_RET_OK = 0,
} MCP9808_Ret;

/*
 * I/O interface
 */
typedef struct mcp9808_iface {
    /*
     * Read 16-bit register value
     */
    int (*reg_read)(void *self, uint8_t reg, uint16_t *value);

    /*
     * Write 16-bit register value
     */
    int (*reg_write)(void *self, uint8_t reg, uint16_t value);

    /*
     * Write 8-bit register value
     */
    int (*reg_write_byte)(void *self, uint8_t reg, uint8_t value);

    /*
     * Interface implementation parameters
     */
    void *params;
} MCP9808_Iface;

/*
 * Set config register flag
 *
 * @param iface I/O iface
 * @param flag flag to set/clear
 * @param enabled 0 = clear, 1 = set
 * @return 0 on success, -1 on error
 */
int mcp9808_set_cfg_flag(MCP9808_Iface *iface, uint16_t flag, uint8_t enabled);

/*
 * Put MCP9808 to into sleep mode
 *
 * @param iface I/O iface
 * @return 0 on success, -1 on error
 */
int mcp9808_sleep(MCP9808_Iface *iface);

/*
 * Wake MCP9808 from sleep mode
 *
 * @param iface I/O iface
 * @return 0 on success, -1 on error
 */
int mcp9808_wake(MCP9808_Iface *iface);

/*
 * Get current temperature in °C
 *
 * @param iface I/O iface
 * @param temperature pointer to store temperature into
 * @return 0 on success, -1 on error
 */
int mcp9808_get_temperature(MCP9808_Iface *iface, float *temperature);

/*
 * Set resolution
 *
 * @param iface I/O iface
 * @param res resolution to set
 * @return 0 on success, -1 on error
 */
int mcp9808_set_res(MCP9808_Iface *iface, MCP9808_Resolution res);

/*
 * Check that MCP9808 is detected and reporting correct device id
 *
 * @param iface I/O iface
 * @return 0 on success, -1 on error
 */
int mcp9808_is_detected(MCP9808_Iface *iface);

#endif /* MCP9808_H */
