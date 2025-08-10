
extern "C" {
    #include <stdio.h>
    #include <stdint.h>
    #include "string.h"

    #include "esp_log.h"
    #include "esp_system.h"
    #include "esp_timer.h"
    #include "driver/gpio.h"

    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "freertos/timers.h"

    #include "bmp_interface.h"
    #include "bmp3.h"
}
#define ESP32_MODEL_NAME DEV // DEV - esp32 , C3 - esp32 c3

void ws_ctrl_data(uint8_t *data);
void buzz_beep(uint16_t tms);
uint8_t running_task = 0;

#include "module_gpios.h"
#include "global_fun.h"
#include "drone_motor_ctrl_esc.h"
#include "nvs_data_store.h"
#include "wifi_ws_handler.h"
#include "i2c_driver_class.h"
#include "mpu6500_handler.cpp"
#include "function_comp.cpp"

TaskHandle_t drone_motor_task_h, web_service_h, vl53l0x_sensor_h;
QueueHandle_t web_service_q;

#define FC_I2C_PORT_NUM I2C_NUM_1

NVS_DATA_STORE nvs_f;
ESP_WIFI_DEV WiFi_drv(&nvs_f);
DRONE_MOTOR_CTRL dmotor(FC_ESC_PIN_1, FC_ESC_PIN_2, FC_ESC_PIN_3, FC_ESC_PIN_4);
ESP_I2C_IDF i2cDrv(FC_I2C_SDA_PIN,FC_I2C_SCL_PIN, 400, FC_I2C_PORT_NUM);
BATTERY_LEVEL_CALCULATOR xbattery(10.0f, 2.0f, 12.6f, 10.6f);

// #define delay(x) vTaskDelay(pdMS_TO_TICKS(x))

int16_t drone_ati = 0, drone_roll = 0, drone_pitch = 0;
void ws_ctrl_data(uint8_t *data){
    drone_roll = data[1] << 8 | data[0];
    drone_ati = data[3] << 8 | data[2];
    drone_pitch = data[5] << 8 | data[4];  
}

#define MAX_ROLL_PITCH_ANG 20
#define STICK_DEADZONE 5

float desired_roll = 0, desired_pitch = 0, desired_yaw = 0;

PID_Controller pidRateROLL;
PID_Controller pidRatePITCH;
PID_Controller pidRateYAW;

PID_Controller pidAngROLL; // 2,0,0
PID_Controller pidAngPITCH;

PID_Controller pidVerhei;
PID_Controller pidVerVelo;

int16_t mpu_offsets[6] = {0};

bool Drone_control_off = false;

void enable_drone(){
    Drone_control_off = false;
    mpu.setIntDMPEnabled(true);
    // ESP_ERROR_CHECK(mpu6050.enable());
}
void disable_drone(){
    // ESP_ERROR_CHECK(mpu6050.disable());
    mpu.setIntDMPEnabled(false);
    Drone_control_off = true;
    drone_ati = 0;
    dmotor.break_motors();
}
struct GLOBAL_DATA_ST{
    uint16_t altitude_mm = 0;
};
GLOBAL_DATA_ST gData;

struct WEB_SERVICE_TXT_PKT{
    uint8_t type[4];
    uint8_t* data;
    size_t size;
};
template <typename Text, typename... TextArg>
esp_err_t web_service_txt(char type[4], Text str, TextArg... strArg){
    if(!WiFi_drv.WIFI_CONNECTED_SUC) return ESP_FAIL;
    
    WEB_SERVICE_TXT_PKT wsDtPkt = {0};

    memcpy(wsDtPkt.type, type, 4);
    wsDtPkt.data = (uint8_t*)malloc(64);
    snprintf((char*)wsDtPkt.data, 64, str, strArg...);
    wsDtPkt.size = strlen((char*)wsDtPkt.data);

    if(xQueueSend(web_service_q, &wsDtPkt, 0) != pdPASS){
        // ESP_LOGE("DD", "WS SEND TO QUEUE FAILED !!");
        free(wsDtPkt.data);
        return ESP_FAIL;
    }
    return ESP_OK;
}
void web_service(void *arg){

    WEB_SERVICE_TXT_PKT web_pkt = {0};

    uint64_t second_5_timer_fun = 0;

    while(1){
        if(WiFi_drv.WIFI_CONNECTED_SUC){
            if(xQueueReceive(web_service_q, &web_pkt, 0) == pdTRUE){
                WiFi_drv.ws_dt_Str_send((char*)web_pkt.type, "%s", web_pkt.data);
                free(web_pkt.data);
            }
            if(esp_timer_get_time() >= second_5_timer_fun){
                second_5_timer_fun = esp_timer_get_time() + 1000000;
                web_service_txt("BAT", "%d %0.2fV", xbattery.get_precentage(), xbattery.get_voltage());
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}


class BMP3_BARROMETER_CLASS{
    bmp3_dev dev;
    bmp3_data rx_data = {0};
    bmp3_settings settings = {0};
    uint16_t settings_sel = 0;
    bmp3_status rx_status = {{0}};

    double ground_pressure = 0;

public:
    BMP3_BARROMETER_CLASS(){
        settings.int_settings.drdy_en = BMP3_ENABLE;
        settings.press_en = BMP3_ENABLE;
        settings.temp_en = BMP3_ENABLE;

        settings.odr_filter.press_os = BMP3_OVERSAMPLING_2X;
        settings.odr_filter.temp_os = BMP3_OVERSAMPLING_2X;
        settings.odr_filter.odr = BMP3_ODR_100_HZ;
        
        settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_ODR | BMP3_SEL_DRDY_EN;
    }
    esp_err_t init(){
        int8_t bmp_err = 0;

        bmp_err = bmp3_interface_init(&this->dev, BMP3_I2C_INTF);
        bmp3_check_rslt("bmp3_interface_init", bmp_err);

        bmp_err = bmp3_init(&dev);
        bmp3_check_rslt("bmp3_init", bmp_err);

        bmp_err = bmp3_set_sensor_settings(settings_sel, &settings, &dev);
        bmp3_check_rslt("bmp3_set_sensor_settings", bmp_err);

        settings.op_mode = BMP3_MODE_NORMAL;
        bmp_err = bmp3_set_op_mode(&settings, &dev);
        bmp3_check_rslt("bmp3_set_op_mode", bmp_err);

        if(bmp_err == 0) return ESP_OK;
        return ESP_FAIL;
    }

    double get_data(){
        int8_t bmp_err = 0;

        bmp_err = bmp3_get_status(&this->rx_status, &dev);
        bmp3_check_rslt("bmp3_get_status", bmp_err);

        if(bmp_err == BMP3_OK && rx_status.intr.drdy == BMP3_ENABLE){
            /*
             * First parameter indicates the type of data to be read
             * BMP3_PRESS_TEMP : To read pressure and temperature data
             * BMP3_TEMP       : To read only temperature data
             * BMP3_PRESS      : To read only pressure data
             */
            bmp_err = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &rx_data, &dev);
            bmp3_check_rslt("bmp3_get_sensor_data", bmp_err);

            /* NOTE : Read status register again to clear data ready interrupt status */
            bmp_err = bmp3_get_status(&rx_status, &dev);
            bmp3_check_rslt("bmp3_get_status", bmp_err);
            
            // float pres = pressureX.get_filtered(rx_data.pressure);
            // printf(">T:%.2f,P: %.2f,f:%.2f,m:%.4f\n", (rx_data.temperature), (rx_data.pressure), pres, get_altitude_from_pressure(pres));
            return rx_data.pressure;
        }
        return 0;
    }

    double get_rel_pressure(){
        double temp = this->get_data() - this->ground_pressure;
        return temp;
    }

    void calibrate_ground(){
        double temp_ground = 0;
        uint16_t count = 0;
        
        while(count < 200){ // discard 200 samples
            if(this->get_data() > 0) count ++;
            vTaskDelay(pdMS_TO_TICKS(4));
        }

        while(count < 100){ // get 100 samples
            double temp = this->get_data();
            if(temp > 0){
                count ++;
                temp_ground += temp;
            }
            vTaskDelay(pdMS_TO_TICKS(4));
        }

        temp_ground /= count;
        this->ground_pressure = temp_ground;

        ESP_LOGI("BMP", "Ground pressure : %0.4f\n", temp_ground);
    }

};

BMP3_BARROMETER_CLASS baro;

void drone_motor_task(void* arg){
    // LOW_PASS_FILTER altLowFil;
    
    uint16_t mpufps = 0;
    uint64_t mpunextFps = 0;

    float input_roll = 0, input_pitch = 0, input_yaw = 0;
    float drate_roll = 0, drate_pitch = 0;
    // float kal_ang_roll = 0, kal_ang_pitch = 0;

    bool loop_reset = false;
    
    float mpu_dmp_angles[3] = {0};
    float mpu_gy_rates[3] = {0};
    float mpu_acc_rates[3] = {0};

    // float drone_motor
    // float drone_xaltitude = 0;
    // float drone_vvel = 0;

    // float drone_alt_rate = 0;
    // float drone_alt_val = 0;

    while (1){
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(400));
        // looptime = esp_timer_get_time();

        if(!Drone_control_off && drone_ati > 50 && mpu6050_task(mpu_dmp_angles, mpu_gy_rates, mpu_acc_rates)) {
            loop_reset = false;
            mpufps++;

            //// vertical
            // float altmm = altLowFil.get_filtered(gData.altitude_mm);
            // float altrate = altRateCal.velocity(altmm);
            web_service_txt("DEB1", "acc :\n x: %.4f\n y: %.4f\n z: %.4f", mpu_acc_rates[0], mpu_acc_rates[1], mpu_acc_rates[2]);

            // float temp_alt = get_ultrasonic_distance();
            // if(temp_alt != -1){
            //     drone_xaltitude = temp_alt;
            //     drone_vvel = alt_vel.velocity(drone_xaltitude);
            //     web_service_txt("DEB1", "altitude : %0.2f\n velocity : %0.2f", drone_xaltitude, drone_vvel);
                
            //     drone_alt_val += pidVerVelo.calculate_pid(drone_vvel, drone_ati/100);
            // }
            
            // printf("altitude : %0.2f\n", drone_xaltitude);
            // drone_alt_rate = pidVerhei.calculate_pid(drone_xaltitude, drone_ati);
            // drone_alt_val = pidVerVelo.calculate_pid(drone_vvel, drone_alt_rate);

            ////
            // printf(">r:%0.4f,p:%0.4f,y:%0.4f,gx:%0.4f,gy:%0.4f,gz:%0.4f\r\n", 
            //             mpu_dmp_angles[0],mpu_dmp_angles[1],mpu_dmp_angles[2],
            //             mpu_gy_rates[0],mpu_gy_rates[1],mpu_gy_rates[2]);

            /// desired drone angles in deg
            desired_roll = (drone_roll > STICK_DEADZONE || drone_roll < -STICK_DEADZONE) ? (float)drone_roll * MAX_ROLL_PITCH_ANG / 150 : 0;
            desired_pitch = (drone_pitch > STICK_DEADZONE || drone_pitch < -STICK_DEADZONE) ? (float)drone_pitch * MAX_ROLL_PITCH_ANG / 150 : 0;
            desired_yaw = 0;

            /// outer loop, angle pid and output desired rate
            drate_roll = pidAngROLL.calculate_pid(mpu_dmp_angles[0], desired_roll);
            drate_pitch = pidAngROLL.calculate_pid(mpu_dmp_angles[1], desired_pitch);
            
            /// inner loop, gyro rate and desired rate to output motor
            input_roll = pidRateROLL.calculate_pid(mpu_gy_rates[0], drate_roll);
            input_pitch = pidRatePITCH.calculate_pid(mpu_gy_rates[1], -drate_pitch);
            input_yaw = pidRateYAW.calculate_pid(mpu_gy_rates[2], desired_yaw);

            drone_ati = (drone_ati > 900) ? 900 : drone_ati;

            // dmotor.mot_spd[0] = drone_alt_val * 2 - input_roll - input_pitch + input_yaw + 2000;
            // dmotor.mot_spd[1] = drone_alt_val * 2 + input_roll - input_pitch - input_yaw + 2000;
            // dmotor.mot_spd[2] = drone_alt_val * 2 + input_roll + input_pitch + input_yaw + 2000;
            // dmotor.mot_spd[3] = drone_alt_val * 2 - input_roll + input_pitch - input_yaw + 2000;

            dmotor.mot_spd[0] = drone_ati * 2 - input_roll - input_pitch + input_yaw + 2000;
            dmotor.mot_spd[1] = drone_ati * 2 + input_roll - input_pitch - input_yaw + 2000;
            dmotor.mot_spd[2] = drone_ati * 2 + input_roll + input_pitch + input_yaw + 2000;
            dmotor.mot_spd[3] = drone_ati * 2 - input_roll + input_pitch - input_yaw + 2000;

            dmotor.set_motors();

            // printf(">r:%0.4f,p:%0.4f\r\n", mpu6050.comAng.roll, mpu6050.comAng.pitch);
        } else {
            if(!loop_reset){
                loop_reset = true;
                pidRateROLL.reset_I();
                pidRatePITCH.reset_I();
                pidRateYAW.reset_I();
                pidAngPITCH.reset_I();
                pidAngROLL.reset_I();
                pidVerhei.reset_I();
                pidVerVelo.reset_I();
                dmotor.break_motors();
            }
        }
        if(!loop_reset && esp_timer_get_time() >= mpunextFps){
            mpunextFps = esp_timer_get_time() + 1000*1000;
            web_service_txt("mpf","F:%d",mpufps);
            mpufps = 0;
        }
    }
}

void mpu6050_intr_cfg(){
    gpio_config_t gCfg = {
        .pin_bit_mask = (1ULL << FC_GYRO_INT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&gCfg);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(FC_GYRO_INT_PIN, [](void *arg){
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            vTaskNotifyGiveFromISR(drone_motor_task_h, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }, NULL);
        
    ESP_LOGI(TAG, "GPIO Interrupt Configured on GPIO");
}

void drone_init(){

    dmotor.init();
    dmotor.break_motors();

    buz_pin_init(FC_BUZZER_PIN);
    xbattery.init();

    nvs_f.init();
    nvs_f.reg_var_nvs("motm", &dmotor.max_mot, sizeof(dmotor.max_mot));
    nvs_f.reg_var_nvs("motc", &dmotor.calib_mot, sizeof(dmotor.calib_mot));

    nvs_f.reg_var_nvs("Rpid", pidRateROLL.get_kpid_ptr(), sizeof(float) * 3);
    nvs_f.reg_var_nvs("Ppid", pidRatePITCH.get_kpid_ptr(), sizeof(float) * 3);
    nvs_f.reg_var_nvs("Ypid", pidRateYAW.get_kpid_ptr(), sizeof(float) * 3);

    nvs_f.reg_var_nvs("ARid", pidAngROLL.get_kpid_ptr(), sizeof(float) * 3);
    nvs_f.reg_var_nvs("APid", pidAngPITCH.get_kpid_ptr(), sizeof(float) * 3);

    nvs_f.reg_var_nvs("VerR", pidVerVelo.get_kpid_ptr(), sizeof(float) * 3);
    nvs_f.reg_var_nvs("VerV", pidVerhei.get_kpid_ptr(), sizeof(float) * 3);
    
    nvs_f.reg_var_nvs("mpug", &mpu6050_gyro_cal, sizeof(mpu6050_gyro_cal));
    nvs_f.reg_var_nvs("mpua", &mpu6050_acc_cal, sizeof(mpu6050_acc_cal));
    nvs_f.reg_var_nvs("mpuc", &mpu_offsets, sizeof(mpu_offsets));
    
    nvs_f.recoverData();

    WiFi_drv.init();
    
    // ESP_ERROR_CHECK(ultrasonic_rmt_init(FC_EX_PIN_26, FC_EX_PIN_33, 300, 10));

    i2cDrv.install_i2c_driver();
    i2cDrv.i2c_scan();
    
    web_service_q = xQueueCreate(10, sizeof(WEB_SERVICE_TXT_PKT));
    if(web_service_q == NULL){
        ESP_LOGE("QUEUE", "Failed to create queue for ws frame");
    }
    xTaskCreate(web_service, "web_service", 4096, NULL, 3, &web_service_h);

    mpu6050_init_task(buzz_beep, mpu_offsets);
    xTaskCreatePinnedToCore(&drone_motor_task, "drone_motor_task", 1024*8, NULL, 20, &drone_motor_task_h, 1);
    mpu6050_intr_cfg();
    // xTaskCreatePinnedToCore(&vl53l0x_sensor, "vl53l0x_sensor", 4096, NULL, 10, &vl53l0x_sensor_h, 1);
    // ESP_ERROR_CHECK(mpu6050.init(&drone_motor_task_h));

    vTaskDelay(pdMS_TO_TICKS(100));

    baro.init();
    baro.calibrate_ground();

    vTaskDelay(pdMS_TO_TICKS(100));

    // vTaskSuspend(drone_motor_task_h);
}

void indicatorMotor(){
    dmotor.set_motors(MIN_MOTOR_SPEED+150, 0, 0, 0);
    vTaskDelay(500);
    dmotor.set_motors(0, MIN_MOTOR_SPEED+150, 0, 0);
    vTaskDelay(500);
    dmotor.set_motors(0, 0, MIN_MOTOR_SPEED+150, 0);
    vTaskDelay(500);
    dmotor.set_motors(0, 0, 0, MIN_MOTOR_SPEED+150);
    vTaskDelay(500);
    dmotor.break_motors();
    vTaskDelay(500);
}


extern "C" void app_main(void){
    printf("BOOTING...\n");
    printf("SETUP IN PROG...\n");
    vTaskDelay(pdMS_TO_TICKS(100));

    drone_init();

    uint8_t pre_running_task = running_task;


    printf("done\n");
    buzz_beep(1000);
    indicatorMotor();

    // uint64_t ws_fps_nxt_up = 0;

    while (1) {
        if(running_task != pre_running_task){
            pre_running_task = running_task;
            buzz_beep(100);
        }
        // char serial_data[16];
        // fgets(serial_data, sizeof(serial_data), stdin);
        
        switch(running_task){
            case 0: // stop
                // stop_drone_motor_task();
                disable_drone();
                dmotor.break_motors();
                running_task = 1;
                break;

            case 1: // idle
                vTaskDelay(pdMS_TO_TICKS(20));
                break;

            case 2: // run
                running_task = 1;
                enable_drone();
                vTaskDelay(pdMS_TO_TICKS(20));
                

                break;

            case 3: {// calibrate
                printf( "calibration starting...\n");
                WiFi_drv.ws_dt_Str_send("DEBV", "calibration starting...\n");

                disable_drone();
                dmotor.break_motors();

                vTaskDelay(pdMS_TO_TICKS(500));

                printf("calibrating..\n");
                mpu6050_calibrate(mpu_offsets);
                printf("calibration values :\nacc : %d %d %d \ngy: %d %d %d\n",
                    mpu_offsets[0],mpu_offsets[1],mpu_offsets[2],
                    mpu_offsets[3],mpu_offsets[4],mpu_offsets[5]);
                    
                WiFi_drv.ws_dt_Str_send("DEBV", "calibrated saving data...\n");

                vTaskDelay(pdMS_TO_TICKS(100));
                nvs_f.writeToNVS("mpuc");
                nvs_f.writeToNVS("mpua");
                nvs_f.writeToNVS("mpug");
                buzz_beep(100);
                vTaskDelay(pdMS_TO_TICKS(100));

                WiFi_drv.ws_dt_Str_send("DEBV", "calibration completed...\n");
                printf("calibration completed...\n");
                indicatorMotor();
                running_task = 1;
                }
                break;
            case 4: {// calibrate motors
                uint16_t maxmot = 4000;
                uint16_t minmot = 2100;
                buzz_beep(60);
                printf("Starting motor calibration...\n");
                printf("conntect motor power\n");
                buzz_beep(60);
                vTaskDelay(pdMS_TO_TICKS(1000));

                printf("set motor high\n");
                dmotor.mot_spd[0] = maxmot;
                dmotor.mot_spd[1] = maxmot;
                dmotor.mot_spd[2] = maxmot;
                dmotor.mot_spd[3] = maxmot;
                dmotor.set_motors();
                for(uint8_t i = 5; i > 0; i--){
                    printf("end in : %d s\n",i);
                    buzz_beep(80);
                    vTaskDelay(pdMS_TO_TICKS(1000));
                }

                printf("set motor low\n");
                dmotor.mot_spd[0] = minmot;
                dmotor.mot_spd[1] = minmot;
                dmotor.mot_spd[2] = minmot;
                dmotor.mot_spd[3] = minmot;
                dmotor.set_motors();
                vTaskDelay(pdMS_TO_TICKS(1000));

                printf("done!!\n");
                buzz_beep(200);
                vTaskDelay(pdMS_TO_TICKS(1000));
                buzz_beep(200);
                running_task = 0;
                }
                break;
            case 5:
                // i2c_driver.i2c_scan();
                running_task = 0;
                break;

            case 6: // get calibrate data
                
                break;

            case 7: {
                float *temp = (float*)pidRateROLL.get_kpid_ptr();
                WiFi_drv.ws_dt_Str_send("DEBV", "Rpid kp: %f ki: %f kd: %f\n", temp[0], temp[1], temp[2]);
                temp = (float*)pidRatePITCH.get_kpid_ptr();
                WiFi_drv.ws_dt_Str_send("DEBV", "Ppid kp: %f ki: %f kd: %f\n", temp[0], temp[1], temp[2]);
                temp = (float*)pidRateYAW.get_kpid_ptr();
                WiFi_drv.ws_dt_Str_send("DEBV", "Ypid kp: %f ki: %f kd: %f\n", temp[0], temp[1], temp[2]);
                temp = (float*)pidAngROLL.get_kpid_ptr();
                WiFi_drv.ws_dt_Str_send("DEBV", "Apid kp: %f ki: %f kd: %f\n", temp[0], temp[1], temp[2]);
                }
                running_task = 1;
                break;
            
            

            default:
                running_task = 0;
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
}



// bool isRunningMpuTask = false;
// void stop_drone_motor_task(bool stop){
//     // if(drone_motor_task_h != NULL) vTaskSuspend(drone_motor_task_h);
//     // if(ESP_MOTOR_TASK_H != NULL) xTimerStop(ESP_MOTOR_TASK_H, 0);
//     // mpu6.reset_pid();

//     isRunningMpuTask = !stop;

//     if(stop)
//         gpio_intr_disable(GPIO_NUM_1);
//     else
//         gpio_intr_enable(GPIO_NUM_1);
// }

// void setupMpuInterrupt(){
//     gpio_config_t gCfg = {
//         .pin_bit_mask = (1ULL << GPIO_NUM_1),
//         .mode = GPIO_MODE_INPUT,
//         .pull_up_en = GPIO_PULLUP_ENABLE,
//         .pull_down_en = GPIO_PULLDOWN_ENABLE,
//         .intr_type = GPIO_INTR_NEGEDGE,
//     };
//     gpio_config(&gCfg);

//     gpio_install_isr_service(0);
//     gpio_isr_handler_add(GPIO_NUM_1, mpuInterruptTask, NULL);

//     ESP_LOGI(TAG, "GPIO Interrupt Configured on GPIO");

//     isRunningMpuTask = true;
// }


// void vl53l0x_sensor(void *arg){
//     vl53l0x_t *vll = NULL;
//     vll = vl53l0x_config(FC_I2C_PORT_NUM, -1, 0x29, 0);
//     if (vll == NULL){
//         ESP_LOGE("LL", "VL53l0x init error");
//         vTaskDelete(NULL);
//     }
//     const char *ret = vl53l0x_init(vll);
//     if(ret != NULL){
//         ESP_LOGW("LL", "LL : %s", ret);
//     }

//     vl53l0x_startContinuous(vll, 5);
    
//     ESP_LOGI("LL", "VL53l0x configured");

//     while(1){
//         if(!Drone_control_off){
//             uint16_t dis = vl53l0x_readRangeContinuousMillimeters(vll);
//             gData.altitude_mm = dis;
//             // web_service_txt("DEB2", "altitude : %d", dis);
//         }
//         vTaskDelay(pdMS_TO_TICKS(5));
//     }
// }

// struct VERTICAL_CTRL{
//     float prv_alt = 0;
//     uint64_t prv_ti = 0;
//     float velocity(float curr_alt){
//         float df = esp_timer_get_time() - prv_ti;
//         prv_ti = esp_timer_get_time();
//         float Vel = (curr_alt - prv_alt) / (df / 1000000.0f); // ds / dt = v
//         prv_alt = curr_alt;
//         return Vel;
//     }
// };