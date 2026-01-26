#ifndef ACCEL_GYRO_CUSTOM_DOOR_H
#define ACCEL_GYRO_CUSTOM_DOOR_H

#include <stdint.h>
#include "bmi270.h"




// tune these values as needed to lower power consumption
#define BMI2_ACC_ODR_SPEED_HZ   BMI2_ACC_ODR_50HZ   // 50 Hz
#define BMI2_GYR_ODR_SPEED_HZ   BMI2_GYR_ODR_50HZ   // 50 Hz

#define BMI2_GYR_RANGE_DPS      BMI2_GYR_RANGE_250  // 250 dps
#define BMI2_GYR_RANGE_DPS_VAL  BMI2_GYR_RANGE_250_VAL  // 250 dps

#define BMI2_MPS2_RANGE         2.0f               // +/- 2g




/*! Buffer size allocated to store raw FIFO data. */
#define BMI2_FIFO_RAW_DATA_BUFFER_SIZE  UINT16_C(2048)

/*! Length of data to be read from FIFO. */
#define BMI2_FIFO_RAW_DATA_USER_LENGTH  UINT16_C(2048)

/*! Number of accel frames to be extracted from FIFO. */

/*! Calculation for frame count: Total frame count = Fifo buffer size(2048)/ Total frames(6 Accel, 6 Gyro and 1 header,
 * totaling to 13) which equals to 157.
 *
 * Extra frames to parse sensortime da  ta
 */
#define BMI2_FIFO_ACCEL_FRAME_COUNT     UINT8_C(185)

/*! Number of gyro frames to be extracted from FIFO. */
#define BMI2_FIFO_GYRO_FRAME_COUNT      UINT8_C(185)

/*! Macro to read sensortime byte in FIFO. */
#define SENSORTIME_OVERHEAD_BYTE        UINT8_C(220)




#define TO_MPS2(axis)  lsb_to_mps2(axis, BMI2_MPS2_RANGE,          get_bmi2_device_ptr()->resolution);
#define TO_DPS(axis)   lsb_to_dps (axis, BMI2_GYR_RANGE_DPS_VAL,   get_bmi2_device_ptr()->resolution);




typedef struct {
    float acc_x;
    float acc_y;
    float acc_z;
    float gyr_x;
    float gyr_y;
    float gyr_z;
} custom_accel_gyro_data_t;


typedef void (*fifo_cb_t)(custom_accel_gyro_data_t *sample, void *ctx);

//int  main_accel_gyro(void);
struct bmi2_dev* get_bmi2_device_ptr(void);
int  custom_door___accel_gyro_init(void);
int  custom_door___accel_gyro_read(custom_accel_gyro_data_t* data);
void custom_door___accel_gyro_print(uint32_t indx, const custom_accel_gyro_data_t *data);

int custom_door___accel_gyro_enable(void);
int custom_door___accel_gyro_disable(void);


//int custom_door___accel_gyro_read_fifo( void (*sample_cb)(custom_accel_gyro_data_t *sample));
//int custom_door___accel_gyro_read_fifo(fifo_cb_t cb, void *ctx);
int custom_door___accel_gyro_read_fifo(uint16_t* fifoDepth);
void custom_door___accel_gyro_fifo_flush(void);


float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width);
float lsb_to_dps(int16_t val, float dps, uint8_t bit);


#endif // ACCEL_GYRO_CUSTOM_DOOR_H