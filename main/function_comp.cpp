extern "C" {
    #include "stdint.h"
    #include "stdio.h"
    #include "string.h"

    #include "esp_log.h"
    #include "esp_timer.h"
}

#define PID_CTRL_I_TERM_MAX 400

struct LOW_PASS_FILTER {
    float prv_value = 0;
    float coeff = 0.6f;
    float get_filtered(float value){
        value = this->coeff * this->prv_value + (1 - this->coeff) * value;
        this->prv_value = value;
        return value;
    }
};

/// @brief uses ESP32 libraries !!
class PID_Controller {
private:
    float Kpid[3] = {0};
    float prv_I = 0;
    float prv_perr = 0;
    int64_t prv_time = 0;

public:
    PID_Controller(float _Kp = 0, float _Ki = 0, float _Kd = 0){
        this->Kpid[0] = _Kp;
        this->Kpid[1] = _Ki;
        this->Kpid[2] = _Kd;
    }
    
    void* get_kpid_ptr(){
        return &this->Kpid;
    }
    void reset_I(){
        this->prv_I = 0;
    }
    float calculate_pid(float input, float desired){
        float dt = this->get_dt();
        float P = 0, I = 0, D = 0, perr = 0;
        
        perr = desired - input;
        P = this->Kpid[0] * perr;

        I = this->prv_I +  this->Kpid[1] * (perr + prv_perr) * dt / 2;
        I = (I > PID_CTRL_I_TERM_MAX) ? PID_CTRL_I_TERM_MAX : 
            (I < -PID_CTRL_I_TERM_MAX) ? -PID_CTRL_I_TERM_MAX : I;
            
        D = this->Kpid[2] * (perr - prv_perr)/dt;

        this->prv_perr = perr;
        this->prv_I = I;
        
        float pid_value = P + I + D;
        pid_value = (pid_value > PID_CTRL_I_TERM_MAX) ? PID_CTRL_I_TERM_MAX : 
            (pid_value < -PID_CTRL_I_TERM_MAX) ? -PID_CTRL_I_TERM_MAX : pid_value;
            
        return pid_value;
    }
private:
    float get_dt(){
        float dt = (esp_timer_get_time() - this->prv_time) / 1e6f;
        this->prv_time = esp_timer_get_time();
        return dt;
    }
};

class KALMAN_1D_FILTER{
    float kalman_angle = 0;
    float kalman_un_angle = 2*2;

public:
    KALMAN_1D_FILTER(){

    }
    /// @brief get the calculated kalman angle
    /// @param kalmanInput gyro rate input
    /// @param kalmanMeasurement accelerometer calculated angle
    /// @return kalman angle 
    float get_angle(float kalmanInput, float kalmanMeasurement){
        this->kalman_angle = this->kalman_angle + 0.004f * kalmanInput;
        this->kalman_un_angle = this->kalman_un_angle + 0.000256f; // this->kalman_un_angle = this->kalman_un_angle + 0.004*0.004*4*4;

        float kalman_gain = this->kalman_un_angle / (this->kalman_un_angle + 9.0f); // float kalman_gain = this->kalman_un_angle*1/(1*this->kalman_un_angle + 3*3);

        this->kalman_angle = this->kalman_angle + kalman_gain*(kalmanMeasurement - this->kalman_angle);
        this->kalman_un_angle = (1 - kalman_gain) * kalman_un_angle;

        return this->kalman_angle;
    }

};