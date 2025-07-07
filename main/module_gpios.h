#include "driver/gpio.h"

// GY_INT = 2 | Pixel_led - 5 | rx2 = 16 | tx2 = 17 | buz = 23
// m1 = 18 | m2 = 19 | m3 = 21 | m4 = 22 | 12v_ref = sen_vp
// scl = 4 | sda = 15
// hspi_miso = 12 | hspi_mosi = 13 | hspi_sck = 14

#if ESP32_MODEL_NAME == DEV
    #define FC_ESC_PIN_1 GPIO_NUM_18
    #define FC_ESC_PIN_2 GPIO_NUM_19
    #define FC_ESC_PIN_3 GPIO_NUM_22
    #define FC_ESC_PIN_4 GPIO_NUM_21

    #define FC_I2C_SDA_PIN 15
    #define FC_I2C_SCL_PIN 4

    #define FC_UART_TX_PIN 17
    #define FC_UART_RX_PIN 16

    #define FC_HSPI_MISO_PIN 12
    #define FC_HSPI_MOSI_PIN 13
    #define FC_HSPI_SCK_PIN 14
    #define FC_HSPI_CS_PIN 27

    #define FC_GYRO_INT_PIN GPIO_NUM_2
    #define FC_BATT_V_SEN_PIN 36
    #define FC_PIXLED_PIN 5
    #define FC_BUZZER_PIN GPIO_NUM_23

    #define FC_EX_PIN_25 25
    #define FC_EX_PIN_26 26
    #define FC_EX_PIN_32 32
    #define FC_EX_PIN_33 33

#elif ESP32_MODEL_NAME == C3
    #define FC_ESC_PIN_1 GPIO_NUM_7
    #define FC_ESC_PIN_2 GPIO_NUM_5
    #define FC_ESC_PIN_3 GPIO_NUM_2
    #define FC_ESC_PIN_4 GPIO_NUM_10
    
    #define FC_I2C_SDA_PIN 3
    #define FC_I2C_SCL_PIN 4

    #define FC_BUZZER_PIN GPIO_NUM_8

#else
    #error "ESP32 model not specified"
#endif

// I (26604) I2C: Starting I2C scan...
// I (26614) I2C: Device found at address 0x68
// I (26614) I2C: Device found at address 0x76
// I (26614) I2C: I2C scan complete