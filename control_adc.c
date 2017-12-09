#define NRF_LOG_MODULE_NAME control_adc
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();


#include <math.h>

#include "nrf.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"

#include "app_scheduler.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_saadc.h"


#include "boards.h"
#include "app_timer.h"

#include "control_adc.h"

#include "macros_common.h"


#define CHANNELS            2   /* 0=speed, 1=battery */
#define SAMPLES_PER_CHANNEL 5
#define SAMPLES_IN_BUFFER (CHANNELS * SAMPLES_PER_CHANNEL)
volatile uint8_t state = 1;

static const nrf_drv_timer_t m_timer = NRF_DRV_TIMER_INSTANCE(0);
static nrf_saadc_value_t m_buffer_pool[2][SAMPLES_IN_BUFFER];
static nrf_ppi_channel_t m_ppi_channel;
static uint32_t m_adc_evt_counter;

int last_speed = 0;
int last_battery_level = 0;
int last_battery_voltage = 0;

int battery_level()
{
    return last_battery_level;
}


static long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    long value = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    if (value < out_min) {
        value = out_min;
    }
    if (value > out_max) {
        value = out_max;
    }
    return value;
}



static int clamp(int value, int min_val, int max_val)
{
    if (value < min_val) {
        value = min_val;
    }
    if (value > max_val) {
        value = max_val;
    }
    return value;
}



static int avg(nrf_saadc_value_t *a, int count, int offset)
{
    int sum = 0;
    for (int i = 0; i < count; i++) {
        int idx = offset + (i*2);
        sum += a[idx];
    }
    int avg = sum / count;
    return avg;
}


static void calculate_speed(int reading)
{
    static int last_reading = -1;

    #define MIN_THRESHOLD 115

    reading = clamp(reading, 0, 4095);

    if ((reading < (last_reading - 5)) || (reading > (last_reading + 5))) {
        last_reading = reading;
    } else {
        reading = last_reading;
    }

    if (reading < MIN_THRESHOLD) {
        last_speed = 0;
    }
    else {
        last_speed = map(reading, 100, 4096, 0, 1000);
    }
}



static void calculate_voltage(uint32_t reading)
{
    //matches the constant used in the SAADC configuration for this channel
    float adc_gain = 1.0 / 5.0;


    // 0.6f is the NRF52 internal reference voltage used for the comparison
    // 4096 is the resolution of the ADC(12 bits)
    float tmp = reading / ((adc_gain / (0.6f)) * 4096);

    // compensate for the hardware voltage divider (maps back to the 4.2V maximum)
    // matches the voltage divider on the hardware
    float divider = 2.0 / (0.8 + 2.0);
    tmp /= divider;

    // convert to millivolts
    int voltage = ((tmp * 1000 + 5) / 10) * 10;


    last_battery_voltage = voltage;

    // convert to the percentage value expected in the BLE characteristic schema (uint8 0-100)
    const int MIN_BATTERY_VOLTAGE = 2700;
    const int MAX_BATTERY_VOLTAGE = 4175;
    last_battery_level = map(voltage, MIN_BATTERY_VOLTAGE, MAX_BATTERY_VOLTAGE, 0, 100);

    NRF_LOG_INFO("battery reading:%d  voltage:%d  level:%d", reading, last_battery_voltage, last_battery_level);
}



static void control_adc_callback(nrf_drv_saadc_evt_t const *p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE) {
        ret_code_t err_code;

#if 0
        for (int i = 0; i < SAMPLES_IN_BUFFER; i++) {
            NRF_LOG_INFO("sample %d: %d", i, p_event->data.done.p_buffer[i]);
        }
#endif

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

        int speed_avg = avg((p_event->data.done.p_buffer), SAMPLES_PER_CHANNEL, 0);
        int battery_avg = avg(p_event->data.done.p_buffer, SAMPLES_PER_CHANNEL, 1);

        calculate_speed(speed_avg);
    //NRF_LOG_INFO("speed avg:%03d, reading: %03d", speed_avg, last_speed);

        calculate_voltage(battery_avg);
    //NRF_LOG_INFO("battery reading: %d", last_battery_level);
    } else {
        NRF_LOG_INFO("unhandled p_event->type %d", p_event->type);
    }
    m_adc_evt_counter++;
}




void timer_handler(nrf_timer_event_t event_type, void *p_context)
{
}


static bool control_adc_calibration_in_progress = false;


static ret_code_t control_adc_calibrate()
{
    control_adc_calibration_in_progress = true;

    return NRF_SUCCESS;
}



ret_code_t control_adc_init(void)
{
    ret_code_t err_code;

    nrf_drv_saadc_config_t saadc_config = NRF_DRV_SAADC_DEFAULT_CONFIG;
    saadc_config.resolution = NRF_SAADC_RESOLUTION_12BIT;

    nrf_saadc_channel_config_t speed_channel_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN6);
    speed_channel_config.burst = NRF_SAADC_BURST_ENABLED;
    speed_channel_config.gain = NRF_SAADC_GAIN1_4;
    speed_channel_config.acq_time = NRF_SAADC_ACQTIME_40US;
    speed_channel_config.reference = NRF_SAADC_REFERENCE_VDD4;

    nrf_saadc_channel_config_t battery_channel_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN7);
    battery_channel_config.burst = NRF_SAADC_BURST_ENABLED;
    battery_channel_config.gain = NRF_SAADC_GAIN1_5;
    battery_channel_config.acq_time = NRF_SAADC_ACQTIME_40US;

    err_code = nrf_drv_saadc_init(&saadc_config, control_adc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = control_adc_calibrate();
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &speed_channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(1, &battery_channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("initialized");

    return NRF_SUCCESS;
}



/* set the battery check period (check every N ms) */
ret_code_t control_adc_set_rate(uint32_t ms)
{
    ret_code_t err_code;

    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    err_code = nrf_drv_timer_init(&m_timer, &timer_cfg, timer_handler);
    APP_ERROR_CHECK(err_code);

    /* setup m_timer for compare event every 400ms */
    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_timer, ms);
    nrf_drv_timer_extended_compare(&m_timer,
                                   NRF_TIMER_CC_CHANNEL1,
                                   ticks,
                                   NRF_TIMER_SHORT_COMPARE1_CLEAR_MASK,
                                   false);
    nrf_drv_timer_enable(&m_timer);

    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer,
                                                                                NRF_TIMER_CC_CHANNEL1);
    uint32_t saadc_sample_task_addr = nrf_drv_saadc_sample_task_get();

    /*
     * setup ppi channel so that timer compare event is triggering sample
     * task in SAADC
     */
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel,
                                          timer_compare_event_addr, saadc_sample_task_addr);
    APP_ERROR_CHECK(err_code);


    err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("set rate: %d ms", ms);

    return NRF_SUCCESS;

}
