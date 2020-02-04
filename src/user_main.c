#include "osapi.h"
#include "user_interface.h"
#include "pwm.h"


#define PWM_CHANNELS 1
#define PWM_PERIOD 5000       //5000 ticks * 200ns ~= 1kHz

#define ADC_TASK_PRIO 0
#define ADC_TASK_QUEUE_LEN 5
#define ADC_READS 500

#define PEDAL_MIN 130
#define PEDAL_MAX 645
#define POT_MIN 90
#define POT_MAX 1023


// -- GLOBAL VARS --
static os_timer_t print_timer;

uint32 pwm_duty_init[PWM_CHANNELS] = { 0 };
uint32 pwm_io[PWM_CHANNELS][3] = {
	// MUX, FUNC, PIN
	{PERIPHS_IO_MUX_MTDI_U, FUNC_GPIO12, 12},
};
uint16_t pwm_duty = 0;

volatile uint16_t adc_pedal = 0;
volatile uint16_t adc_pot = 0;
uint32_t adc_value = 0;
os_event_t adc_task_queue[ADC_TASK_QUEUE_LEN];
enum {
    ADC_TASK_POT = 0,
    ADC_TASK_PEDAL,
    ADC_TASK_SWITCH,
};
// -----------------


uint32 ICACHE_FLASH_ATTR user_rf_cal_sector_set(void)
{
    enum flash_size_map size_map = system_get_flash_size_map();
    uint32 rf_cal_sec = 0;

    switch (size_map) {
        case FLASH_SIZE_4M_MAP_256_256:
            rf_cal_sec = 128 - 5;
            break;

        case FLASH_SIZE_8M_MAP_512_512:
            rf_cal_sec = 256 - 5;
            break;

        case FLASH_SIZE_16M_MAP_512_512:
        case FLASH_SIZE_16M_MAP_1024_1024:
            rf_cal_sec = 512 - 5;
            break;

        case FLASH_SIZE_32M_MAP_512_512:
        case FLASH_SIZE_32M_MAP_1024_1024:
            rf_cal_sec = 1024 - 5;
            break;

        case FLASH_SIZE_64M_MAP_1024_1024:
            rf_cal_sec = 2048 - 5;
            break;
        case FLASH_SIZE_128M_MAP_1024_1024:
            rf_cal_sec = 4096 - 5;
            break;
        default:
            rf_cal_sec = 0;
            break;
    }
    return rf_cal_sec;
}

// GPIO16 (speaker) - Helpers
// GPIO16 is connected to RTC, so manipulating it differs than others
void ICACHE_FLASH_ATTR gpio16_output_conf(void)
{
    WRITE_PERI_REG(
        PAD_XPD_DCDC_CONF,
        (READ_PERI_REG(PAD_XPD_DCDC_CONF) & 0xffffffbc) | (uint32)0x1
    ); 	// mux configuration for XPD_DCDC to output rtc_gpio0

    WRITE_PERI_REG(
        RTC_GPIO_CONF,
        (READ_PERI_REG(RTC_GPIO_CONF) & (uint32)0xfffffffe) | (uint32)0x0
    );	//mux configuration for out enable

    WRITE_PERI_REG(
        RTC_GPIO_ENABLE,
        (READ_PERI_REG(RTC_GPIO_ENABLE) & (uint32)0xfffffffe) | (uint32)0x1
    );	//out enable
}

void ICACHE_FLASH_ATTR  gpio16_output_set(uint8 value)
{
    WRITE_PERI_REG(
        RTC_GPIO_OUT,
        (READ_PERI_REG(RTC_GPIO_OUT) & (uint32)0xfffffffe) | (uint32)(value & 1)
    );
}

void enableArc()
{
    GPIO_OUTPUT_SET(4,1);
}

void disableArc()
{
    GPIO_OUTPUT_SET(4,0);
}

void updatePwm()
{
    if (adc_pedal == 0) {
        disableArc();
        pwm_duty = adc_pot / (1000.0/PWM_PERIOD);
    } else {
        pwm_duty = (adc_pot * adc_pedal) / (1000000.0/PWM_PERIOD);
        enableArc();
    }
    pwm_set_duty(pwm_duty, 0);
    pwm_start();
}

// map value to a 0..1000 range triming both ends
int prepareAdcValue(int value, int min, int max)
{
    value -= min;
    value *= 1024.0/(max-min);
    if (value < 0) {
        value = 0;
    }
    if (value > 1000) {
        value = 1000;
    }
    return value;
}

void setAdcPedal(int value)
{
    value = 1024 - value;
    adc_pedal = prepareAdcValue(value, PEDAL_MIN, PEDAL_MAX);
    updatePwm();
}

void setAdcPot(int value)
{
    adc_pot = prepareAdcValue(value, POT_MIN, POT_MAX);
    updatePwm();
}

// State machine for adc reading and channel switching using
// os_task and message queue. This wont stave cpu as each rutine is short,
// queues another via task queue.
static void ICACHE_FLASH_ATTR adc_task(os_event_t *events) 
{
    switch (events->sig) {
        case ADC_TASK_SWITCH:
            switch (events->par) {
                case ADC_TASK_POT:
                    gpio_output_set(0, BIT13, BIT13, BIT14);
                    setAdcPedal(adc_value / ADC_READS);

                    break;
                case ADC_TASK_PEDAL:
                    gpio_output_set(0, BIT14, BIT14, BIT13);
                    setAdcPot(adc_value / ADC_READS);

                    break;
            }

            adc_value = 0;
            system_os_post(ADC_TASK_PRIO, events->par, 0);

            break;
        default:
            adc_value += system_adc_read();
            if (events->par >= ADC_READS - 1) {
                switch (events->sig) {
                    case ADC_TASK_POT:
                        system_os_post(ADC_TASK_PRIO, ADC_TASK_SWITCH, ADC_TASK_PEDAL);
                        break;
                    case ADC_TASK_PEDAL:
                        system_os_post(ADC_TASK_PRIO, ADC_TASK_SWITCH, ADC_TASK_POT);
                        break;
                }
            } else {
                system_os_post(ADC_TASK_PRIO, events->sig, events->par + 1);
            }

            break;
    }
}

// Basic method for debuging
void ICACHE_FLASH_ATTR print_status (void *arg)
{
    os_printf("POT: %d  PEDAL: %d  PWM: %d  ARC: %d\n", adc_pot, adc_pedal, pwm_duty,GPIO_INPUT_GET(4));

    if (adc_pedal >= 1000) {
        gpio16_output_set(1);
    } else {
        gpio16_output_set(0);
    }
}

// MAIN entry point
void ICACHE_FLASH_ATTR user_init(void)
{
    // Please stop that spekear imediatly!
    gpio_init();
    gpio16_output_conf();

    // Select GPIO function for ADC control pins (GPIO14 and GPIO13)
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, FUNC_GPIO14);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_GPIO13);

    uart_init(115200, 115200);
    os_delay_us(1000);


    os_printf("\n\n\n\nSDK version:%s\n", system_get_sdk_version());

    // Disable WiFi
    wifi_set_opmode(NULL_MODE);

    // Init pwm driver
    pwm_init(PWM_PERIOD, pwm_duty_init, PWM_CHANNELS, pwm_io);
    pwm_start();

    // Prepare to launch print_status periodically each 50ms
    os_timer_disarm(&print_timer);
    os_timer_setfn(&print_timer, (os_timer_func_t *)print_status, NULL);
    os_timer_arm(&print_timer, 50, 1);
    
    // Register ADC control task, and queue first switch
    system_os_task(adc_task, ADC_TASK_PRIO, adc_task_queue, ADC_TASK_QUEUE_LEN);
    system_os_post(ADC_TASK_PRIO, ADC_TASK_SWITCH, ADC_TASK_POT);
}
