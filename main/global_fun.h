#include "stdio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "math.h"
#include "driver/adc.h"

/// battery, buzzer functions


// #define BUZZER_PIN GPIO_NUM_23
// gpio_num_t BUZZER_PIN = GPIO_NUM_8;
gpio_num_t BUZZER_PIN = GPIO_NUM_23;

TimerHandle_t buzzer_timer_h;
static volatile bool buzz_active = true;

static void IRAM_ATTR buzz_isr_h(TimerHandle_t xTimer){
    xTimerStop(xTimer, 0);
    gpio_set_level(BUZZER_PIN, 0);
    buzz_active = false;
}

void buzz_beep(uint16_t tms){
    if(buzz_active)return;

    buzz_active = true;
    gpio_set_level(BUZZER_PIN, 1);
    xTimerChangePeriod(buzzer_timer_h, pdMS_TO_TICKS(tms), 0);
    xTimerStart(buzzer_timer_h, 0);
}

void buz_pin_init(gpio_num_t pin){
    BUZZER_PIN = pin;
    
    gpio_config_t buz_pin_conf{
            .pin_bit_mask = (1ULL << BUZZER_PIN),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
    gpio_config(&buz_pin_conf);

    gpio_set_level(BUZZER_PIN, 1);
    
    buzzer_timer_h = xTimerCreate("buzzer_timer_h", pdMS_TO_TICKS(100), pdTRUE, NULL, buzz_isr_h);
    if(buzzer_timer_h != NULL) xTimerStart(buzzer_timer_h, 0);
            
    // xTimerDelete(buzzer_timer_h, 0);
}


/// @brief inlcude driver/adc.h
class BATTERY_LEVEL_CALCULATOR{
    float r1, r2;
    float max_bat, min_bat;
    float V_t;

    /// @brief get voltage at VCC
    /// @return return VCC in float
    void get_voltage_v(){
        V_t = adc1_get_raw(FC_BATT_V_SEN_PIN);
        V_t = V_t * 4.0f / 4095.0f; //voltage in divider
        V_t = V_t * (r1 + r2) / r2;
    }
public:
    /// @brief set physical values
    /// @param _r1 resistor value in kohm near vcc
    /// @param _r2  resistor value in kohm near gnd
    /// @param _max_bat max battery v float
    /// @param _min_bat min battery v float
    BATTERY_LEVEL_CALCULATOR(float _r1, float _r2, float _max_bat, float _min_bat):
        r1(_r1), r2(_r2), max_bat(_max_bat), min_bat(_min_bat){}
    
    void init(){
        esp_err_t err = ESP_OK;
        gpio_num_t pin;
        adc1_pad_get_io_num(FC_BATT_V_SEN_PIN, &pin);

        gpio_config_t pinCfg = {
            .pin_bit_mask = (1ULL << pin),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&pinCfg);
        
        err |= adc1_config_width(ADC_WIDTH_BIT_12);
        err |= adc1_config_channel_atten(FC_BATT_V_SEN_PIN, ADC_ATTEN_DB_12);
        if (err != ESP_OK) ESP_LOGW("BAT", "BAT LVL INIT ERROR");
    }
    /// @brief first need to call get_voltage
    /// @return precentage of batt
    uint8_t get_precentage(){
        get_voltage_v();
        float pre = 100 * (V_t - min_bat) / (max_bat - min_bat);
        pre = (pre <= 0) ? 0 : (pre >= 100) ? 100 : pre;
        return roundf(pre);
    }
    float get_voltage(){
        return V_t;
    }
};
