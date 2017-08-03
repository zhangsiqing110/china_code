#include <stdint.h>
#include "parking.h"
#include "parking_sensor.h"
#include "parking_sensor_rti.h"
#include "SENtralA2_types.h"

static int park_sns_rti_set_user_ctx_input(SENtralA2Iface *rti, uint8_t value)
{
    if (rti->set_parameter(rti, PARK_RTI_PIO_PAGE_USER,
            PARK_RTI_PIO_USER_CTX_INPUT, &value, sizeof(value))
            != SENA2_RET_OK) {
        return PNI_PARKING_RET_ERROR;
    }

    return PNI_PARKING_RET_OK;
}

static int park_sns_rti_get_car_detector_cfg(SENtralA2Iface *rti,
        PNI_ParkingRtiCarDetectorCfg *cfg)
{
    if (rti->get_parameter(rti, PARK_RTI_PIO_PAGE_USER,
            PARK_RTI_PIO_USER_CAR_DETECTOR_OUTPUT_TYPE, (uint8_t *)cfg,
            sizeof(*cfg)) != SENA2_RET_OK) {
        return PNI_PARKING_RET_ERROR;
    }

    return PNI_PARKING_RET_OK;
}

static int park_sns_rti_set_car_detector_cfg(SENtralA2Iface *rti,
        PNI_ParkingRtiCarDetectorCfg *cfg)
{
    if (rti->set_parameter(rti, PARK_RTI_PIO_PAGE_USER,
            PARK_RTI_PIO_USER_CAR_DETECTOR_OUTPUT_TYPE, (uint8_t *)cfg,
            sizeof(*cfg)) != SENA2_RET_OK) {
        return PNI_PARKING_RET_ERROR;
    }

    return PNI_PARKING_RET_OK;
}

static int park_sns_rti_set_car_detector_enabled(void *self, int enabled)
{
    PNI_ParkingSensorIface *iface = self;
    SENtralA2Iface *rti = iface->params;

    if (rti->set_sns_rate(rti, PARK_RTI_SNS_TYPE_CAR_DETECTOR, !!enabled)
            != SENA2_RET_OK) {
        return PNI_PARKING_RET_ERROR;
    }

    return PNI_PARKING_RET_OK;
}

static int park_sns_rti_set_car_detector_data_enabled(void *self, int enabled)
{
    PNI_ParkingSensorIface *iface = self;
    SENtralA2Iface *rti = iface->params;

    PNI_PRINTF("[DBG] cat detect mag data(0x%.02X), value(%d)\r\n",
      PARK_RTI_SNS_TYPE_CAR_DETECTOR_DATA, enabled);

    if (rti->set_sns_rate(rti, PARK_RTI_SNS_TYPE_CAR_DETECTOR_DATA,
            !!enabled) != SENA2_RET_OK) {
        return PNI_PARKING_RET_ERROR;
    }

    return PNI_PARKING_RET_OK;
}

static int park_sns_rti_set_inter_state_enabled(void *self, int enabled)
{
    PNI_ParkingSensorIface *iface = self;
    SENtralA2Iface *rti = iface->params;
    PNI_ParkingRtiCarDetectorCfg cfg = { 0 };

    if (park_sns_rti_get_car_detector_cfg(rti, &cfg) != PNI_PARKING_RET_OK) {
        return PNI_PARKING_RET_ERROR;
    }

    cfg.intermediate_report = !!enabled;

    return park_sns_rti_set_car_detector_cfg(rti, &cfg);
}

static int park_sns_rti_force_occupied(void *self)
{
    PNI_ParkingSensorIface *iface = self;
    SENtralA2Iface *rti = iface->params;

    return park_sns_rti_set_user_ctx_input(rti,
            PARK_RTI_USER_CTX_INPUT_CAR_PARKED);
}

static int park_sns_rti_force_vacant(void *self)
{
    PNI_ParkingSensorIface *iface = self;
    SENtralA2Iface *rti = iface->params;

    return park_sns_rti_set_user_ctx_input(rti,
            PARK_RTI_USER_CTX_INPUT_NO_CAR);
}

static int park_sns_rti_recalibrate(void *self)
{
    PNI_ParkingSensorIface *iface = self;
    SENtralA2Iface *rti = iface->params;

    return park_sns_rti_set_user_ctx_input(rti,
            PARK_RTI_USER_CTX_INPUT_RECALIBRATE);
}

static int park_sns_rti_self_test(void *self)
{
    PNI_ParkingSensorIface *iface = self;
    SENtralA2Iface *rti = iface->params;

    if (rti->self_test(rti) != SENA2_RET_OK) {
        return PNI_PARKING_RET_ERROR;
    }
    return PNI_PARKING_RET_OK;
}

static int park_sns_rti_get_version(void *self, PNI_ParkingVersion *version)
{
    PNI_ParkingSensorIface *iface = self;
    SENtralA2Iface *rti = iface->params;
    PNI_ParkingRtiVersion v = { 0 };

    if (rti->get_parameter(rti, PARK_RTI_PIO_PAGE_INFO,
            PARK_RTI_PIO_INFO_FW_VERSION, (uint8_t *)&v, sizeof(v))
            != SENA2_RET_OK) {
        return PNI_PARKING_RET_ERROR;
    }

    version->major = v.major;
    version->minor = v.minor;
    version->patch = v.patch;
    version->build = v.build;

    return PNI_PARKING_RET_OK;
}

static int park_sns_rti_set_hw_alarm_mode_enabled(void *self, int enabled)
{
    PNI_ParkingSensorIface *iface = self;
    SENtralA2Iface *rti = iface->params;
    uint8_t value = !!enabled;

    PNI_PRINTF("[DBG] set gAlarmMode(0x%.02X), value(%d)\r\n",
      PARK_RTI_PIO_USER_G_ALARM_MODE, value);

    if (rti->set_parameter(rti, PARK_RTI_PIO_PAGE_USER,
            PARK_RTI_PIO_USER_G_ALARM_MODE, &value, sizeof(value))
            != SENA2_RET_OK) {
        PNI_PRINTF("[ERROR] disable/enable G ALARM MODE\r\n");
        return PNI_PARKING_RET_ERROR;
    }

    return PNI_PARKING_RET_OK;
}

static int park_sns_rti_set_temperature(void *self, int8_t temperature)
{
    PNI_ParkingSensorIface *iface = self;
    SENtralA2Iface *rti = iface->params;

    if (rti->set_parameter(rti, PARK_RTI_PIO_PAGE_USER,
            PARK_RTI_PIO_USER_TEMPERATURE, (uint8_t *)&temperature,
            sizeof(temperature)) != SENA2_RET_OK) {
        return PNI_PARKING_RET_ERROR;
    }

    return PNI_PARKING_RET_OK;
}

/** Check if pattern A the maintenance mode(BLE on) is triggered
  * @param void * self - Parking sensor interface
  * @retval int8_t Return value  ( -1/0/1 == Error/No/Yes(triggered) )
  */
static int park_sns_rti_is_maint_triggered(void *self)
{
    PNI_ParkingSensorIface *iface = self;
    SENtralA2Iface *rti = iface->params;
    uint8_t value = 0;
    int result = 0;

    if (rti->get_parameter(rti, PARK_RTI_PIO_PAGE_USER,
            PARK_RTI_PIO_USER_TRIGGER_FLAG, &value, sizeof(value))
            != SENA2_RET_OK) {
        return PNI_PARKING_RET_ERROR;
    }

    if (value == PARK_RTI_USER_TRIG_FLAG_MAINT)
      result = 1;
    else
      result =  0;

    return result;
}

/** Check manufacture mode Flag(pattern E) is enabled or not
  * @param void * self - Parking sensor interface
  * @retval int8_t Return value  ( -1/0/1 == Error/No/Yes(triggered) )
  */
static int park_sns_rti_is_mfg_triggered(void *self)
{
    PNI_ParkingSensorIface *iface = self;
    SENtralA2Iface *rti = iface->params;
    uint8_t value = 0;
    int result = 0;

    if (rti->get_parameter(rti, PARK_RTI_PIO_PAGE_USER,
            PARK_RTI_PIO_USER_TRIGGER_FLAG, &value, sizeof(value))
            != SENA2_RET_OK) {
        return PNI_PARKING_RET_ERROR;
    }

    if (value == PARK_RTI_USER_TRIG_FLAG_MFG)
      result = 1;
    else
      result =  0;

    return result;
}

/** Check shipping mode Flag is enabled or not
  * @param void * self - Parking sensor interface
  * @retval int8_t Return value  ( -1/0/1 == Error/Not Enabled/Enabled )
  */
static int park_sns_rti_is_shipping_mode_enabled(void *self)
{
    PNI_ParkingSensorIface *iface = self;
    SENtralA2Iface *rti = iface->params;
    uint8_t value = 0;

    if (rti->get_parameter(rti, PARK_RTI_PIO_PAGE_USER,
            PARK_RTI_PIO_USER_SHIPPING_MODE_FLAG, &value, sizeof(value))
            != SENA2_RET_OK) {
        return PNI_PARKING_RET_ERROR;
    }

#if 0
    PNI_PRINTF("[DBG] shipping mode: value = %u\r\n", value);

    /* should not use Trigger Flag, value change for different bit patterns */
    uint8_t value2 = 0;
    if (rti->get_parameter(rti, PARK_RTI_PIO_PAGE_USER,
                PARK_RTI_PIO_USER_TRIGGER_FLAG, &value2, sizeof(value2))
                != SENA2_RET_OK) {
            return PNI_PARKING_RET_ERROR;
    }
    PNI_PRINTF("[DBG] shipping mode: trigger value = %u\r\n", value2);
#endif
    return value;
}

static int park_sns_rti_set_shipping_mode_enabled(void *self, uint8_t enabled)
{
    PNI_ParkingSensorIface *iface = self;
    SENtralA2Iface *rti = iface->params;
    uint8_t value = !!enabled;

    if (rti->set_parameter(rti, PARK_RTI_PIO_PAGE_USER,
            PARK_RTI_PIO_USER_SHIPPING_MODE_FLAG, &value, sizeof(value))
            != SENA2_RET_OK) {
        return PNI_PARKING_RET_ERROR;
    }

    return PNI_PARKING_RET_OK;
}

static int park_sns_rti_set_parameter(void *self, uint8_t page, uint8_t num,
            uint8_t *data, uint8_t size)
{
    PNI_ParkingSensorIface *iface = self;
    SENtralA2Iface *rti = iface->params;

    if (rti->set_parameter(rti, page, num, data, size) != SENA2_RET_OK) {
        return PNI_PARKING_RET_ERROR;
    }

    return PNI_PARKING_RET_OK;
}

static int park_sns_rti_get_parameter(void *self, uint8_t page, uint8_t num,
            uint8_t *data, uint8_t size)
{
    PNI_ParkingSensorIface *iface = self;
    SENtralA2Iface *rti = iface->params;

    if (rti->get_parameter(rti, page, num, data, size) != SENA2_RET_OK) {
        return PNI_PARKING_RET_ERROR;
    }

    return PNI_PARKING_RET_OK;
}

void pni_park_sns_rti_init(PNI_ParkingSensorIface *iface, SENtralA2Iface *rti)
{
    iface->set_car_detector_enabled = &park_sns_rti_set_car_detector_enabled;
    iface->set_car_detector_data_enabled
            = &park_sns_rti_set_car_detector_data_enabled;
    iface->set_inter_state_enabled = &park_sns_rti_set_inter_state_enabled;
    iface->force_occupied = &park_sns_rti_force_occupied;
    iface->force_vacant = &park_sns_rti_force_vacant;
    iface->recalibrate = &park_sns_rti_recalibrate;
    iface->self_test = &park_sns_rti_self_test;
    iface->get_version = &park_sns_rti_get_version;
    iface->set_hw_alarm_mode_enabled = &park_sns_rti_set_hw_alarm_mode_enabled;
    iface->set_temperature = &park_sns_rti_set_temperature;
    iface->is_maint_triggered = &park_sns_rti_is_maint_triggered;
    iface->is_shipping_mode_enabled = &park_sns_rti_is_shipping_mode_enabled;
    iface->is_mfg_triggered = &park_sns_rti_is_mfg_triggered;
    iface->set_shipping_mode_enabled = &park_sns_rti_set_shipping_mode_enabled;
    iface->set_parameter = &park_sns_rti_set_parameter;
    iface->get_parameter = &park_sns_rti_get_parameter;

    iface->params = rti;

    /* add custom RTI parking sensor stuff */
    rti->event_sizes[PARK_RTI_SNS_TYPE_CAR_DETECTOR] = 3;
    rti->event_sizes[PARK_RTI_SNS_TYPE_CAR_DETECTOR_DATA] = 13;
}
