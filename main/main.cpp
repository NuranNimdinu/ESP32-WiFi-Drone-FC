
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

TaskHandle_t drone_motor_task_h, web_service_h;
QueueHandle_t web_service_q;

NVS_DATA_STORE nvs_f;
ESP_WIFI_DEV WiFi_drv(&nvs_f);
DRONE_MOTOR_CTRL dmotor(FC_ESC_PIN_1, FC_ESC_PIN_2, FC_ESC_PIN_3, FC_ESC_PIN_4);
ESP_I2C_IDF i2cDrv(FC_I2C_SDA_PIN,FC_I2C_SCL_PIN, 400, I2C_NUM_1);
// MPU6050_HANDLER mpu6050(I2C_NUM_1, FC_GYRO_INT_PIN, MPU6050_I2C_ADDRESS);

#define delay(x) vTaskDelay(pdMS_TO_TICKS(x))
    uint8_t deb_info_num = 0;

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

PID_Controller pidAngROLL(2,0,0); // 2,0,0
PID_Controller pidAngPITCH(2,0,0);
int16_t mpu_offsets[6] = {0};

void enable_drone(){
    mpu.setIntDMPEnabled(true);
    // ESP_ERROR_CHECK(mpu6050.enable());
}
void disable_drone(){
    // ESP_ERROR_CHECK(mpu6050.disable());
    mpu.setIntDMPEnabled(false);
    dmotor.break_motors();
}

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
        ESP_LOGE("DD", "WS SEND TO QUEUE FAILED !!");
        free(wsDtPkt.data);
        return ESP_FAIL;
    }
    return ESP_OK;
}
void web_service(void *arg){

    WEB_SERVICE_TXT_PKT web_pkt = {0};

    while(1){
        if(WiFi_drv.WIFI_CONNECTED_SUC){
            if(xQueueReceive(web_service_q, &web_pkt, 0) == pdTRUE){
                WiFi_drv.ws_dt_Str_send((char*)web_pkt.type, "%s", web_pkt.data);
                free(web_pkt.data);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void drone_motor_task(void* arg){
    KALMAN_1D_FILTER rollK;
    KALMAN_1D_FILTER pitchK;
    
    uint16_t mpufps = 0;
    uint64_t mpunextFps = 0;

    float input_roll = 0, input_pitch = 0, input_yaw = 0;
    float drate_roll = 0, drate_pitch = 0;
    float kal_ang_roll = 0, kal_ang_pitch = 0;

    bool loop_reset = false;
    
    float mpu_dmp_angles[3] = {0};
    float mpu_gy_rates[3] = {0};

    while (1){
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(400));
        // looptime = esp_timer_get_time();

        if(drone_ati > 50 && mpu6050_task(mpu_dmp_angles, mpu_gy_rates)) {
            loop_reset = false;
            mpufps++;
            // printf(">r:%0.4f,p:%0.4f,y:%0.4f,gx:%0.4f,gy:%0.4f,gz:%0.4f\r\n", 
            //             mpu_dmp_angles[0],mpu_dmp_angles[1],mpu_dmp_angles[2],
            //             mpu_gy_rates[0],mpu_gy_rates[1],mpu_gy_rates[2]);
            // continue;
            // mpu.dmpGetGyro
            
            /// actual angles after filter
            // kal_ang_roll = rollK.get_angle(mpu6050.gyVal.gyro_x, mpu6050.comAng.roll);
            // kal_ang_pitch = pitchK.get_angle(mpu6050.gyVal.gyro_y, mpu6050.comAng.pitch);
            // kal_ang_roll = mpu6050.comAng.roll;
            // kal_ang_pitch = mpu6050.comAng.pitch;

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

            drone_ati = (drone_ati > 800) ? 800 : drone_ati;

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

    nvs_f.init();
    nvs_f.reg_var_nvs("motm", &dmotor.max_mot, sizeof(dmotor.max_mot));
    nvs_f.reg_var_nvs("motc", &dmotor.calib_mot, sizeof(dmotor.calib_mot));
    nvs_f.reg_var_nvs("Rpid", pidRateROLL.get_kpid_ptr(), sizeof(float) * 3);
    nvs_f.reg_var_nvs("Ppid", pidRatePITCH.get_kpid_ptr(), sizeof(float) * 3);
    nvs_f.reg_var_nvs("Ypid", pidRateYAW.get_kpid_ptr(), sizeof(float) * 3);
    nvs_f.reg_var_nvs("ARid", pidAngROLL.get_kpid_ptr(), sizeof(float) * 3);
    nvs_f.reg_var_nvs("APid", pidAngPITCH.get_kpid_ptr(), sizeof(float) * 3);
    // nvs_f.reg_var_nvs("mpua", &mpu6050.acCalVal, sizeof(mpu6050.acCalVal));
    // nvs_f.reg_var_nvs("mpug", &mpu6050.gyCalVal, sizeof(mpu6050.gyCalVal));
    nvs_f.reg_var_nvs("mpua", &mpu6050_gyro_cal, sizeof(mpu6050_gyro_cal));
    nvs_f.reg_var_nvs("mpuc", &mpu_offsets, sizeof(mpu_offsets));
    
    nvs_f.recoverData();

    WiFi_drv.init();
    
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
    // ESP_ERROR_CHECK(mpu6050.init(&drone_motor_task_h));

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

        // if(esp_timer_get_time() >= ws_fps_nxt_up){
        //     ws_fps_nxt_up = esp_timer_get_time() + 1000000;
        //     WiFi_drv.ws_dt_Str_send("DEB", "fps m: %d g: %d\n", motor_fps, gyro_fps);
        //     motor_fps = 0;
        //     gyro_fps = 0;
        // }
        
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
                WiFi_drv.ws_dt_Str_send("DEB", "calibration starting...\n");

                disable_drone();
                dmotor.break_motors();

                vTaskDelay(pdMS_TO_TICKS(500));

                printf("calibrating..\n");
                mpu6050_calibrate(mpu_offsets);
                printf("calibration values :\nacc : %d %d %d \ngy: %d %d %d\n",
                    mpu_offsets[0],mpu_offsets[1],mpu_offsets[2],
                    mpu_offsets[3],mpu_offsets[4],mpu_offsets[5]);
                    
                WiFi_drv.ws_dt_Str_send("DEB", "calibrated saving data...\n");

                vTaskDelay(pdMS_TO_TICKS(100));
                nvs_f.writeToNVS("mpuc");
                nvs_f.writeToNVS("mpua");
                buzz_beep(100);
                vTaskDelay(pdMS_TO_TICKS(100));

                WiFi_drv.ws_dt_Str_send("DEB", "calibration completed...\n");
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
                WiFi_drv.ws_dt_Str_send("DEB", "Rpid kp: %f ki: %f kd: %f\n", temp[0], temp[1], temp[2]);
                temp = (float*)pidRatePITCH.get_kpid_ptr();
                WiFi_drv.ws_dt_Str_send("DEB", "Ppid kp: %f ki: %f kd: %f\n", temp[0], temp[1], temp[2]);
                temp = (float*)pidRateYAW.get_kpid_ptr();
                WiFi_drv.ws_dt_Str_send("DEB", "Ypid kp: %f ki: %f kd: %f\n", temp[0], temp[1], temp[2]);
                temp = (float*)pidAngROLL.get_kpid_ptr();
                WiFi_drv.ws_dt_Str_send("DEB", "Apid kp: %f ki: %f kd: %f\n", temp[0], temp[1], temp[2]);
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

