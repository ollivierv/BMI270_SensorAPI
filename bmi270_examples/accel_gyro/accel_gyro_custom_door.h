#ifndef ACCEL_GYRO_CUSTOM_DOOR_H
#define ACCEL_GYRO_CUSTOM_DOOR_H

#include <stdint.h>


typedef struct {
    float acc_x;
    float acc_y;
    float acc_z;
    float gyr_x;
    float gyr_y;
    float gyr_z;
} custom_accel_gyro_data_t;


typedef void (*fifo_cb_t)(custom_accel_gyro_data_t *sample, void *ctx);

int  main_accel_gyro(void);
int  custom_door___accel_gyro_init(void);
int  custom_door___accel_gyro_read(custom_accel_gyro_data_t* data);
void custom_door___accel_gyro_print(uint32_t indx, const custom_accel_gyro_data_t *data);

int custom_door___accel_gyro_enable(void);
int custom_door___accel_gyro_disable(void);


//int custom_door___accel_gyro_read_fifo( void (*sample_cb)(custom_accel_gyro_data_t *sample));
int custom_door___accel_gyro_read_fifo(fifo_cb_t cb, void *ctx);
void custom_door___accel_gyro_fifo_flush(void);


#endif // ACCEL_GYRO_CUSTOM_DOOR_H