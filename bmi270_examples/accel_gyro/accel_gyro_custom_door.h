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


int  main_accel_gyro(void);
int  custom_door___accel_gyro_init(void);
int  custom_door___accel_gyro_read(custom_accel_gyro_data_t* data);
void custom_door___accel_gyro_print(uint32_t indx, const custom_accel_gyro_data_t *data);

int custom_door___accel_gyro_enable(void);
int custom_door___accel_gyro_disable(void);

#endif // ACCEL_GYRO_CUSTOM_DOOR_H