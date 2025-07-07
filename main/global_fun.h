#include "stdio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/gpio.h"


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
