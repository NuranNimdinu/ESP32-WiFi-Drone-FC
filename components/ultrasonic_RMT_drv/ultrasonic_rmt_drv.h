
#ifndef ULRASONIC_RMT_DRV_H
#define ULRASONIC_RMT_DRV_H

#ifdef __cplusplus
extern "C" {
#endif


#include "esp_log.h"


// THIS IS EASY TO USE ULTRASONIC DISTANCE MEASUREMENT DRIVER BASED ON ESP32 RMT DRIVER
// USES DEDICATED FREERTOS TASK FOR PERIODIC MEASUREMET
// INITIATE ultrasonic_rmt_init(); FUNCTION WITH PARAMETERS FOR ONCE
// TRIG PIN MUST BE OUTPUT PIN ECHO PIN CAN BE ANY INPUT PIN
// SET FREQUENCY OF MESSUREMENT SHOULD HAPPEN 
// IF MEASURING LOW DISTANCES FRQ CAN BE HIGHER THAN LONG DISTANCES
// CALL get_ultrasonic_distance(); FOR GET DISTANCE IN CM

/// @brief initiate function of ultrasonic sensor driver with esp rmt driver
/// @param trig_pin ultrasonic trigger pin
/// @param echo_pin ultrasonic echo pin
/// @param max_distance set maxdistance in cm
/// @param tskfrq set how frequent distance measurement need to done in Hz
/// @return return ESP_OK when all goes good :)
esp_err_t ultrasonic_rmt_init(uint8_t trig_pin, uint8_t echo_pin, int16_t max_distance, int16_t tskfrq);

/// @brief get messured ultrasonic distance
/// @return distance in float if valid distance detected else return -1
float get_ultrasonic_distance();

#ifdef __cplusplus
}
#endif

#endif
