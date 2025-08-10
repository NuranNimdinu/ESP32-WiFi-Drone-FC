#include "math.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "i2c_driver_class.h"

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
// #include "MPU6050.h"

#define RAD_TO_DEG 57.29578  //(180.0/M_PI)
#define DEG_TO_RAD 0.0174533

#define micros() (esp_timer_get_time())

const char* TAG = "MPU";

MPU6050 mpu;

// float mpuIMU_roll, mpuIMU_pitch, mpuIMU_yaw; /// final vals

uint16_t mpuIMU_fifoCount;		// count of all bytes currently in FIFO
uint8_t mpuIMU_fifoBuf[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;			// [w, x, y, z]			quaternion container
VectorInt16 aa;			// [x, y, z]			accel sensor measurements
VectorInt16 aaReal;		// [x, y, z]			gravity-free accel sensor measurements
VectorInt16 aaWorld;	// [x, y, z]			world-frame accel sensor measurements
VectorFloat gravity;	// [x, y, z]			gravity vector
float euler[3];			// [psi, theta, phi]	Euler angle container
float ypr[3];			// [yaw, pitch, roll]	yaw/pitch/roll container and gravity vector


// display quaternion values in easy matrix form: w x y z
void getQuaternion() {
	mpu.dmpGetQuaternion(&q, mpuIMU_fifoBuf);
	// printf("quat x:%6.2f y:%6.2f z:%6.2f w:%6.2f\n", q.x, q.y, q.z, q.w);
}

// display Euler angles in degrees
void getEuler() {
	mpu.dmpGetQuaternion(&q, mpuIMU_fifoBuf);
	mpu.dmpGetEuler(euler, &q);
	// printf("euler psi:%6.2f theta:%6.2f phi:%6.2f\n", euler[0] * RAD_TO_DEG, euler[1] * RAD_TO_DEG, euler[2] * RAD_TO_DEG);
}

// display Euler angles in degrees
void getYawPitchRoll(float *angles) {
	mpu.dmpGetQuaternion(&q, mpuIMU_fifoBuf);
	mpu.dmpGetGravity(&gravity, &q);
	mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

	angles[0] = ypr[2] * RAD_TO_DEG;
	angles[1] = ypr[1] * RAD_TO_DEG;
	angles[2] = ypr[0] * RAD_TO_DEG;

	//printf("ypr roll:%3.1f pitch:%3.1f yaw:%3.1f\n",ypr[2] * RAD_TO_DEG, ypr[1] * RAD_TO_DEG, ypr[0] * RAD_TO_DEG);
	// ESP_LOGI(TAG, "roll:%f pitch:%f yaw:%f",ypr[2] * RAD_TO_DEG, ypr[1] * RAD_TO_DEG, ypr[0] * RAD_TO_DEG);
	// printf(">roll:%f,pitch:%f,yaw:%f\r\n",ypr[2] * RAD_TO_DEG, ypr[1] * RAD_TO_DEG, ypr[0] * RAD_TO_DEG);
}

// display real acceleration, adjusted to remove gravity
void getRealAccel() {
	mpu.dmpGetQuaternion(&q, mpuIMU_fifoBuf);
	mpu.dmpGetAccel(&aa, mpuIMU_fifoBuf);
	mpu.dmpGetGravity(&gravity, &q);
	mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
	// printf("areal x=%d y:%d z:%d\n", aaReal.x, aaReal.y, aaReal.z);
}

// display initial world-frame acceleration, adjusted to remove gravity
// and rotated based on known orientation from quaternion
void getWorldAccel() {
	mpu.dmpGetQuaternion(&q, mpuIMU_fifoBuf);
	mpu.dmpGetAccel(&aa, mpuIMU_fifoBuf);
	mpu.dmpGetGravity(&gravity, &q);
	mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
	mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
	// printf("aworld x:%d y:%d z:%d\n", aaWorld.x, aaWorld.y, aaWorld.z);
}

int16_t mpu6050_gyro_cal[3] = {0};
int16_t mpu6050_acc_cal[3] = {0};

void mpu6050_calibrate(int16_t *offsets){
	
	mpu.setDMPEnabled(false);
	mpu.resetFIFO();
	vTaskDelay(pdMS_TO_TICKS(100));

	// Calibration Time: generate offsets and calibrate our MPU6050
	mpu.setXAccelOffset(0);
	mpu.setYAccelOffset(0);
	mpu.setZAccelOffset(0);
	mpu.setXGyroOffset(0);
	mpu.setYGyroOffset(0);
	mpu.setZGyroOffset(0);
	vTaskDelay(pdMS_TO_TICKS(100));

	mpu.CalibrateAccel(6);
	vTaskDelay(pdMS_TO_TICKS(100));
	mpu.CalibrateGyro(6);
	vTaskDelay(pdMS_TO_TICKS(100));

	int16_t* offmp = mpu.GetActiveOffsets();
	memcpy(offsets, offmp, sizeof(int16_t)*6);

	ESP_LOGI("MPU", "calibrating gryo");
	int32_t cal_temp[3] = {0};
	int16_t temp[3] = {0};
	for(uint8_t i = 0; i < 200; i++){
		mpu.getRotation(&temp[0], &temp[1], &temp[2]);
		cal_temp[0] += temp[0];
		cal_temp[1] += temp[1];
		cal_temp[2] += temp[2];
		printf(".");
		vTaskDelay(pdMS_TO_TICKS(5));
	}
	printf(".\n");

	mpu6050_gyro_cal[0] = (float)cal_temp[0] / 200;
	mpu6050_gyro_cal[1] = (float)cal_temp[1] / 200;
	mpu6050_gyro_cal[2] = (float)cal_temp[2] / 200;

	ESP_LOGI("MPU", "calibrating gryo done");

	ESP_LOGI("MPU", "calibrating accel");
	cal_temp[3] = {0};
	temp[3] = {0};
	for(uint8_t i = 0; i < 200; i++){
		mpu.getAcceleration(&temp[0], &temp[1], &temp[2]);
		cal_temp[0] += temp[0];
		cal_temp[1] += temp[1];
		cal_temp[2] += temp[2];
		printf(".");
		vTaskDelay(pdMS_TO_TICKS(5));
	}
	printf(".\n");

	mpu6050_acc_cal[0] = (float)cal_temp[0] / 200;
	mpu6050_acc_cal[1] = (float)cal_temp[1] / 200;
	mpu6050_acc_cal[2] = (float)cal_temp[2] / 200;

	ESP_LOGI("MPU", "calibrating accel done");

	mpu.setDMPEnabled(true);
}

void mpu6050_init_task(void buz(uint16_t gg), int16_t* offsets){
	mpu.initialize();

	// initilize dmp success = 0
	if(mpu.dmpInitialize() != 0){
		ESP_LOGE(TAG, "DMP initialization failed ...");
		while(1){
			buz(20);
			vTaskDelay(40);
		}
	}
	// calibrate
	// mpu6050_calibrate(offsets);

	mpu.setXAccelOffset(offsets[0]);
	mpu.setYAccelOffset(offsets[1]);
	mpu.setZAccelOffset(offsets[2]);
	mpu.setXGyroOffset(offsets[3]);
	mpu.setYGyroOffset(offsets[4]);
	mpu.setZGyroOffset(offsets[5]);

	mpu.setInterruptLatchClear(true);
	mpu.setIntDMPEnabled(true);

	printf("int latch = %d\n", mpu.getInterruptMode());
	printf("int latch = %d\n", mpu.getInterruptLatchClear());
	printf("int latch = %d\n", mpu.getIntDMPStatus());
	printf("int latch = %d\n", mpu.getIntDMPEnabled());

	mpu.setDMPEnabled(true);

	mpuIMU_fifoCount = mpu.dmpGetFIFOPacketSize();
}

uint8_t printin_tig = 0;
uint8_t debug_output_type = 0;

float acc_sensitivity_factor = 0.0f;
void mpu6050_get_accel(float *accel){
	int16_t temp[3] = {0};
	mpu.getAcceleration(&temp[0], &temp[1], &temp[2]);

	if(acc_sensitivity_factor == 0.0f){
		uint8_t ff = mpu.getFullScaleAccelRange();
		ESP_LOGI("MPU", "ACC SENS %d", ff);
		acc_sensitivity_factor = (ff == 0) ? 16384.f : 
								 (ff == 1) ? 8192.f :
								 (ff == 2) ? 4096.f :
								 (ff == 3) ? 2048.f : 0.0f;
	}

	accel[0] = (float)(temp[0] - mpu6050_acc_cal[0]) / acc_sensitivity_factor;
	accel[1] = (float)(temp[1] - mpu6050_acc_cal[1]) / acc_sensitivity_factor;
	accel[2] = ((float)(temp[2] - mpu6050_acc_cal[2]) / acc_sensitivity_factor) + 1;
}

float gyro_sensitivity_factor = 0.0f;
void mpu6050_get_gyro(float *gyRate){
	int16_t temp[3] = {0};
	mpu.getRotation(&temp[0], &temp[1], &temp[2]);

	if(gyro_sensitivity_factor == 0.0f){
		uint8_t ff = mpu.getFullScaleGyroRange();
		ESP_LOGI("MPU", "GYRO SENS %d", ff);
		gyro_sensitivity_factor = (ff == 0) ? 131.0f : 
								  (ff == 1) ? 65.5f :
								  (ff == 2) ? 32.8f :
								  (ff == 3) ? 16.4f : 0.0f;
	}

	gyRate[0] = (float)(temp[0] - mpu6050_gyro_cal[0]) / gyro_sensitivity_factor;
	gyRate[1] = (float)(temp[1] - mpu6050_gyro_cal[1]) / gyro_sensitivity_factor;
	gyRate[2] = (float)(temp[2] - mpu6050_gyro_cal[2]) / gyro_sensitivity_factor;
}

bool mpu6050_task(float *angles, float *gyRate = NULL, float *accel = NULL){
	uint16_t fifobufsz = mpu.getFIFOCount();

	if(fifobufsz >= mpuIMU_fifoCount){
		if(!mpu.dmpGetCurrentFIFOPacket(mpuIMU_fifoBuf))return false;
		getYawPitchRoll(angles);
		if(gyRate != NULL)mpu6050_get_gyro(gyRate);
		if(accel != NULL)mpu6050_get_accel(accel);
		return true;
	}

	return false;


	// bool gotdata = false;
    // if (mpu.dmpGetCurrentFIFOPacket(mpuIMU_fifoBuf)) { // Get the Latest packet
	// 		getYawPitchRoll();
	// 		gotdata = true;
	// }
	// else{
	// 		gotdata = false;
	// }
		// printin_tig++;
        // if(printin_tig >= 20) {
			// printf(">accX: %f,accY:%f,accZ:%f,gyX:%f,gyY:%f,gyZ:%f\n", ax, ay, az, gx, gy, gz);
			// printf( ">roll:%f,pitch:%f,yaw:%f\n", mpuIMU_roll, mpuIMU_pitch, mpuIMU_yaw);
       	 	// if(debug_output_type == 1) WiFi_drv.ws_dt_Str_send("DEB", "acc: %f %f %f - gy: %f %f %f\n", ax, ay, az, gx, gy, gz);
            // else if(debug_output_type == 2) WiFi_drv.ws_dt_Str_send("DEB", "roll:%f pitch=%f yaw=%f\n", mpuIMU_roll, mpuIMU_pitch, mpuIMU_yaw);
   		// 	printin_tig = 0;
        // }

	// return gotdata;
}
