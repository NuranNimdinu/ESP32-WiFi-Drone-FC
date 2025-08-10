#include <stdio.h>
#include <stdint.h>
#include "string.h"

#include "esp_system.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "ultrasonic_rmt_drv.h"

struct ULTRASONIC_RMT_DRV_STRUCT {
    TaskHandle_t task_han;
    TickType_t xMeasDelayTicks;

    rmt_channel_handle_t tx_ch;
    rmt_channel_handle_t rx_ch;

    rmt_encoder_handle_t copy_encoder;
    rmt_copy_encoder_config_t encoder_cfg;

    rmt_symbol_word_t trigger_word;
    rmt_symbol_word_t echo_word;
    rmt_transmit_config_t trigger_cfg;
    rmt_receive_config_t echo_cfg;

    float distance_cm;
    bool new_distance;
};

struct ULTRASONIC_RMT_DRV_STRUCT ultrasonic_rmt_d;

/// @brief get messured ultrasonic distance
/// @return distance in float if valid distance detected else return -1
float get_ultrasonic_distance(){
    if(ultrasonic_rmt_d.new_distance){
        ultrasonic_rmt_d.new_distance = false;
        return ultrasonic_rmt_d.distance_cm;
    }else {
        return -1;
    }
}

static bool ultrasonic_rmt_rx_cb(rmt_channel_handle_t ch, const rmt_rx_done_event_data_t *edata, void *usr_ctx){
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(ultrasonic_rmt_d.task_han, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    return xHigherPriorityTaskWoken;
}

static void ultrasonic_rmt_distance(){
    rmt_transmit(ultrasonic_rmt_d.tx_ch, ultrasonic_rmt_d.copy_encoder, &ultrasonic_rmt_d.trigger_word, sizeof(ultrasonic_rmt_d.trigger_word), &ultrasonic_rmt_d.trigger_cfg);
    rmt_tx_wait_all_done(ultrasonic_rmt_d.tx_ch, portMAX_DELAY);
    rmt_receive(ultrasonic_rmt_d.rx_ch, &ultrasonic_rmt_d.echo_word, sizeof(ultrasonic_rmt_d.echo_word), &ultrasonic_rmt_d.echo_cfg);

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    uint32_t echo_us = ultrasonic_rmt_d.echo_word.duration0;
    if(echo_us > 0){
        ultrasonic_rmt_d.distance_cm = (echo_us * 0.0343f) / 2.0f;
        ultrasonic_rmt_d.new_distance = true;
    } else {
        ultrasonic_rmt_d.new_distance = false;
    }
}

void ultrasonic_rmt_task(void *arg){
    TickType_t xLastMeasureTime;
    xLastMeasureTime = xTaskGetTickCount();
    while(1){
        ultrasonic_rmt_distance();
        vTaskDelay(pdMS_TO_TICKS(1));
        xTaskDelayUntil(&xLastMeasureTime, ultrasonic_rmt_d.xMeasDelayTicks);
    }
}

/// @brief initiate function of ultrasonic sensor driver with esp rmt driver
/// @param trig_pin ultrasonic trigger pin
/// @param echo_pin ultrasonic echo pin
/// @param max_distance set maxdistance in cm
/// @param frq set how frequent distance measurement need to done
/// @return return ESP_OK when all goes good :)
esp_err_t ultrasonic_rmt_init(uint8_t trig_pin, uint8_t echo_pin, int16_t max_distance, int16_t tskfrq){
    esp_err_t err = ESP_OK;

    rmt_tx_channel_config_t tx_cfg = {
        .gpio_num = (gpio_num_t)trig_pin,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000, // 1MHz = 1us tick
        .mem_block_symbols = 64,
        .trans_queue_depth = 1,
    };
    err = rmt_new_tx_channel(&tx_cfg, &ultrasonic_rmt_d.tx_ch);
    err = rmt_enable(ultrasonic_rmt_d.tx_ch);

    rmt_rx_channel_config_t rx_cfg = {
        .gpio_num = (gpio_num_t)echo_pin,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000,
        .mem_block_symbols = 64,
    };
    err = rmt_new_rx_channel(&rx_cfg, &ultrasonic_rmt_d.rx_ch);
    err = rmt_enable(ultrasonic_rmt_d.rx_ch);

    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = ultrasonic_rmt_rx_cb,
    };
    err = rmt_rx_register_event_callbacks(ultrasonic_rmt_d.rx_ch, &cbs, NULL);

    err = rmt_new_copy_encoder(&ultrasonic_rmt_d.encoder_cfg, &ultrasonic_rmt_d.copy_encoder);

    ultrasonic_rmt_d.trigger_word = (rmt_symbol_word_t){
        .duration0 = 10, .level0 = 1,  // 10us pulse
        .duration1 = 0, .level1 = 0, 
    };

    ultrasonic_rmt_d.trigger_cfg.loop_count = 0;

    ultrasonic_rmt_d.echo_cfg.signal_range_min_ns = 1000;
    ultrasonic_rmt_d.echo_cfg.signal_range_max_ns = max_distance*58*1000;

    uint32_t tckkk = ((1000.0f / tskfrq) > (max_distance*58/1000)) ? (1000.0f / tskfrq) : (max_distance*58/1000);
    ultrasonic_rmt_d.xMeasDelayTicks = pdMS_TO_TICKS(tckkk);

    xTaskCreate(ultrasonic_rmt_task, "ultrasonic_rmt_task", 2048, NULL, 10, &ultrasonic_rmt_d.task_han);

    return err;
}

