/**\
 * Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

/******************************************************************************/
/*!                 Header Files                                              */
#include <stdio.h>
#include <math.h>
#include "accel_gyro_custom_door.h"
#include "../common/zephyr_common.h"


/******************************************************************************/
/*!                Macro definition                                           */

// Earth's gravity in m/s^2
#define GRAVITY_EARTH           (9.80665f)

// Macros to select the sensors
#define ACCEL                   UINT8_C(0x00)
#define GYRO                    UINT8_C(0x01)

// Enable FIFO in imu
#define WITH_FIFO               1




// static
static const uint8_t sensor_list[2] = { BMI2_ACCEL, BMI2_GYRO };
//static struct bmi2_sens_config config;



/******************************************************************************/
/*!           Static Function Declaration                                     */

/*!
 *  @brief This internal API is used to set configurations for accel.
 *
 *  @param[in] bmi       : Structure instance of bmi2_dev.
 *
 *  @return Status of execution.
 */
static int8_t set_accel_gyro_config(struct bmi2_dev *bmi);

/*!
 *  @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 *  range 2G, 4G, 8G or 16G.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] g_range   : Gravity range.
 *  @param[in] bit_width : Resolution for accel.
 *
 *  @return Accel values in meter per second squared.
 */
//static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width);

/*!
 *  @brief This function converts lsb to degree per second for 16 bit gyro at
 *  range 125, 250, 500, 1000 or 2000dps.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] dps       : Degree per second.
 *  @param[in] bit_width : Resolution for gyro.
 *
 *  @return Degree per second.
 */
//static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width);




/**
 * custom custom code
 */

static struct bmi2_dev bmi;

struct bmi2_dev* get_bmi2_device_ptr(void){
    return &bmi;
}


int custom_door___accel_gyro_init(void) {

   /* Interface reference is given as a parameter
     * For I2C : BMI2_I2C_INTF
     * For SPI : BMI2_SPI_INTF
     */
    BMI2_CHECK(bmi2_interface_init(&bmi, BMI2_I2C_INTF)); // custom uses I2C

    /* Initialize bmi270. */
    if(bmi270_init(&bmi) == BMI2_OK) {
        custom_door___config_acc_gyro_default();
        custom_door___accel_gyro_disable();
        return 0;
    }

    return -1;
}


int custom_door___accel_gyro_enable(void){
    return BMI2_CHECK(bmi2_sensor_enable(sensor_list, 2, &bmi));
}


int custom_door___accel_gyro_disable(void){
    BMI2_CHECK(bmi2_sensor_disable(sensor_list, 2, &bmi));
    BMI2_CHECK(bmi2_set_adv_power_save(BMI2_ENABLE, &bmi));
    return 0;
}


#if 0
int custom_door___gyro_enable(void){

    int8_t rslt;
    uint8_t sensor = BMI2_GYRO;

    rslt = bmi2_sensor_enable(&sensor, 1, &bmi);
    bmi2_error_codes_print_result(rslt);

    return (rslt == BMI2_OK) ? 0 : -1;
}


int custom_door___gyro_disable(void){

    int8_t rslt;
    uint8_t sensor = BMI2_GYRO;

    rslt = bmi2_sensor_disable(&sensor, 1, &bmi);
    bmi2_error_codes_print_result(rslt);

    return (rslt == BMI2_OK) ? 0 : -1;
}
#endif


// Fonction qui lit les données accel/gyro UNE FOIS et les place dans la structure
int custom_door___accel_gyro_read(custom_accel_gyro_data_t *data) {

    struct bmi2_sens_data sensor_data = { { 0 } };

    if((BMI2_CHECK(bmi2_get_sensor_data(&sensor_data, &bmi)) == BMI2_OK) && (sensor_data.status & BMI2_DRDY_ACC) && (sensor_data.status & BMI2_DRDY_GYR)) {
        data->acc_x = lsb_to_mps2(sensor_data.acc.x,  2.0f, bmi.resolution);
        data->acc_y = lsb_to_mps2(sensor_data.acc.y,  2.0f, bmi.resolution);
        data->acc_z = lsb_to_mps2(sensor_data.acc.z,  2.0f, bmi.resolution);

        data->gyr_x = lsb_to_dps(sensor_data.gyr.x, BMI2_GYR_RANGE_DPS_VAL, bmi.resolution);
        data->gyr_y = lsb_to_dps(sensor_data.gyr.y, BMI2_GYR_RANGE_DPS_VAL, bmi.resolution);
        data->gyr_z = lsb_to_dps(sensor_data.gyr.z, BMI2_GYR_RANGE_DPS_VAL, bmi.resolution);
        return 0;
    }
    return -1;
}



//
int custom_door___override_acc_gyro_odr(uint8_t odr, uint8_t gyro_noise_perf) {

    struct bmi2_sens_config configs[2];
    uint8_t sensors[2] = { BMI2_ACCEL, BMI2_GYRO };

    // 1. On récupère les configs des DEUX
    configs[0].type = BMI2_ACCEL;
    configs[1].type = BMI2_GYRO;
    BMI2_CHECK(bmi2_get_sensor_config(configs, 2, &bmi));

    // 2. On désactive les deux pour la reconfiguration
    BMI2_CHECK(bmi2_sensor_disable(sensors, 2, &bmi));

    // 3. On aligne les ODR et on force le mode High Performance
    configs[0].cfg.acc.odr = odr; // Accel suit le gyro
    configs[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

    configs[1].cfg.gyr.odr = odr;
    configs[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    configs[1].cfg.gyr.noise_perf  = gyro_noise_perf;
    BMI2_CHECK(bmi2_set_sensor_config(configs, 2, &bmi));

    // 4. On réactive
    BMI2_CHECK(bmi2_sensor_enable(sensors, 2, &bmi));

    return 0;
}



#if 0

int custom_door___set_gyro_ONLY_odr(uint8_t odr, uint8_t gyro_noise_perf){

      int8_t rslt;
    struct bmi2_sens_config config;
    uint8_t sensor = { BMI2_GYRO };

    // 1. On récupère les configs des DEUX
    config.type = BMI2_GYRO;
    rslt = bmi2_get_sensor_config(&config, 1, &bmi);

    // 2. On désactive les deux pour la reconfiguration
    rslt = bmi2_sensor_disable(&sensor, 1, &bmi);
    bmi2_error_codes_print_result(rslt);

    config.cfg.gyr.odr = odr;
    config.cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    config.cfg.gyr.noise_perf  = gyro_noise_perf;
    rslt = bmi2_set_sensor_config(&config, 1, &bmi);
    bmi2_error_codes_print_result(rslt);

    // 4. On réactive
    rslt = bmi2_sensor_enable(&sensor, 1, &bmi);
    bmi2_error_codes_print_result(rslt);

    return (rslt == BMI2_OK) ? 0 : -1;
}


int custom_door___set_gyro_ONLY_FIFO() {
    int8_t rslt;

    /* Disable advanced power save before FIFO */
    bmi2_set_adv_power_save(BMI2_DISABLE, &bmi);

    /* Disable all FIFO configurations first */
    bmi2_set_fifo_config(BMI2_FIFO_ALL_EN, BMI2_DISABLE, &bmi);


    ///* Enable FIFO for accel and gyro (header mode is default) */
    rslt = bmi2_set_fifo_config(BMI2_FIFO_GYR_EN, BMI2_ENABLE, &bmi);
    bmi2_error_codes_print_result(rslt);

    /* To enable headerless mode, disable the header. */
    rslt = bmi2_set_fifo_config(BMI2_FIFO_HEADER_EN, BMI2_DISABLE, &bmi);
    bmi2_error_codes_print_result(rslt);
}
#endif

int custom_door___config_both_FIFO() {

    /* Disable advanced power save before FIFO */
    BMI2_CHECK(bmi2_set_adv_power_save(BMI2_DISABLE, &bmi));

    /* Disable all FIFO configurations first */
    BMI2_CHECK(bmi2_set_fifo_config(BMI2_FIFO_ALL_EN, BMI2_DISABLE, &bmi));

    ///* Enable FIFO for accel and gyro (header mode is default) */
    BMI2_CHECK(bmi2_set_fifo_config(BMI2_FIFO_ACC_EN | BMI2_FIFO_GYR_EN, BMI2_ENABLE, &bmi));

    /* To enable headerless mode, disable the header. */
    BMI2_CHECK(bmi2_set_fifo_config(BMI2_FIFO_HEADER_EN, BMI2_DISABLE, &bmi));

    return 0;
}



int custom_door___config_acc_gyro_default(){

    BMI2_CHECK(set_accel_gyro_config(&bmi));

    custom_door___config_both_FIFO();

    return 0;
}



int custom_door___config_acc_gyro_high_ODR(){

    BMI2_CHECK(set_accel_gyro_config(&bmi));

    custom_door___override_acc_gyro_odr(BMI2_GYR_ODR_HIGH, BMI2_GYR_NOISE_PERF_HIGH);

    custom_door___config_both_FIFO();

    return 0;
}






#if WITH_FIFO




#if 0

/* To read sensortime, extra 3 bytes are added to fifo buffer. */
uint16_t fifo_buffer_size = BMI2_FIFO_RAW_DATA_BUFFER_SIZE + SENSORTIME_OVERHEAD_BYTE;

/* Number of bytes of FIFO data
 * NOTE : Dummy byte (for SPI Interface) required for FIFO data read must be given as part of array size
 * Array size same as fifo_buffer_size
 */
uint8_t fifo_data[BMI2_FIFO_RAW_DATA_BUFFER_SIZE + SENSORTIME_OVERHEAD_BYTE];


/* Array of accelerometer frames -> Total bytes =
* 157 * (6 axes + 1 header bytes) = 1099 bytes */
struct bmi2_sens_axes_data fifo_accel_data[BMI2_FIFO_ACCEL_FRAME_COUNT] = { { 0 } };

/* Array of gyro frames -> Total bytes =
 * 157 * (6 axes + 1 header bytes) = 1099 bytes */
struct bmi2_sens_axes_data fifo_gyro_data[BMI2_FIFO_GYRO_FRAME_COUNT] = { { 0 } };

#endif

#if 1


/* Number of bytes of FIFO data
 * NOTE : Dummy byte (for SPI Interface) required for FIFO data read must be given as part of array size
 */
uint8_t fifo_data[BMI2_FIFO_RAW_DATA_BUFFER_SIZE] = { 0 };

/* Array of accelerometer frames -> Total bytes =
 * 170 * (6 axes bytes) = 1020 bytes */
struct bmi2_sens_axes_data fifo_accel_data[BMI2_FIFO_ACCEL_FRAME_COUNT] = { { 0 } };

/* Array of gyro frames -> Total bytes =
 * 170 * (6 axes bytes) = 1020 bytes */
struct bmi2_sens_axes_data fifo_gyro_data[BMI2_FIFO_GYRO_FRAME_COUNT] = { { 0 } };

#endif



/**
 * @brief This function reads accel and gyro data from FIFO and fills the
 *        global fifo_accel_data and fifo_gyro_data arrays.
 */
#warning "verifier pourquoi la fonction est si lente"
int custom_door___accel_gyro_read_fifo(uint16_t* outFifoDepth) //, uint32_t* outSensortime, uint16_t* outSkippedFrameCount)
{

    uint16_t fifo_length = 0;
    *outFifoDepth = 0;

    // initialize FIFO frame structure
    struct bmi2_fifo_frame fifoframe = {0};

    uint16_t accel_frame_length = BMI2_FIFO_ACCEL_FRAME_COUNT;
    uint16_t gyro_frame_length  = BMI2_FIFO_GYRO_FRAME_COUNT;


    /* Get FIFO length */
    if(BMI2_CHECK(bmi2_get_fifo_length(&fifo_length, &bmi)) != BMI2_OK) {
        return -2;
    }
    if(fifo_length == 0){
        return 0; // pas d'erreur, mais pas de données non plus
    }

    /* Setup FIFO frame */
    fifoframe.data   = fifo_data;
    fifoframe.length = fifo_length + bmi.dummy_byte;

    /* Read FIFO */
    if(BMI2_CHECK(bmi2_read_fifo_data(&fifoframe, &bmi)) != BMI2_OK) {
        return -3;
    }


    /* Extract accel */
    bmi2_extract_accel(fifo_accel_data, &accel_frame_length, &fifoframe, &bmi);

    /* Extract gyro */
    bmi2_extract_gyro(fifo_gyro_data, &gyro_frame_length, &fifoframe, &bmi);

    /* Pair accel + gyro by index */
    *outFifoDepth = (accel_frame_length < gyro_frame_length) ? accel_frame_length : gyro_frame_length;


    return 0;
}




#if 0
int custom_door___gyro_ONLY_read_fifo(uint16_t* outFifoDepth)
{
    int8_t rslt;
    uint16_t fifo_length = 0;
    *outFifoDepth = 0;

    // initialize FIFO frame structure
    struct bmi2_fifo_frame fifoframe = {0};
    uint16_t gyro_frame_length  = BMI2_FIFO_GYRO_FRAME_COUNT;


    /* Get FIFO length */
    rslt = bmi2_get_fifo_length(&fifo_length, &bmi);
    bmi2_error_codes_print_result(rslt);
    if (rslt != BMI2_OK) {
        return -2;
    }
    if(fifo_length == 0){
        return 0; // pas d'erreur, mais pas de données non plus
    }

    /* Setup FIFO frame */
    fifoframe.data   = fifo_data;
    fifoframe.length = fifo_length + bmi.dummy_byte;

    /* Read FIFO */
    rslt = bmi2_read_fifo_data(&fifoframe, &bmi);
    bmi2_error_codes_print_result(rslt);
    if (rslt != BMI2_OK) {
        return -3;
    }

    /* Extract gyro */
    bmi2_extract_gyro(fifo_gyro_data, &gyro_frame_length, &fifoframe, &bmi);

    /* Pair accel + gyro by index */
    *outFifoDepth = gyro_frame_length;



    return 0;
}
#endif




#endif // WITH_FIFO










/*!
 * @brief This internal API is used to set configurations for accel and gyro.
 */
static int8_t set_accel_gyro_config(struct bmi2_dev *bmi)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Structure to define accelerometer and gyro configuration. */
    struct bmi2_sens_config config[2];

    /* Configure the type of feature. */
    config[ACCEL].type = BMI2_ACCEL;
    config[GYRO].type = BMI2_GYRO;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi2_get_sensor_config(config, 2, bmi);
    bmi2_error_codes_print_result(rslt);

    /* Map data ready interrupt to interrupt pin. */
    rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, bmi);
    bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK)
    {
        /* NOTE: The user can change the following configuration parameters according to their requirement. */
        /* Set Output Data Rate */
        config[ACCEL].cfg.acc.odr = BMI2_ACC_ODR_DEFAULT;

        /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G). */
        config[ACCEL].cfg.acc.range = BMI2_ACC_RANGE_2G;

        /* The bandwidth parameter is used to configure the number of sensor samples that are averaged
         * if it is set to 2, then 2^(bandwidth parameter) samples
         * are averaged, resulting in 4 averaged samples.
         * Note1 : For more information, refer the datasheet.
         * Note2 : A higher number of averaged samples will result in a lower noise level of the signal, but
         * this has an adverse effect on the power consumed.
         */
        config[ACCEL].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;

        /* Enable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)
         * For more info refer datasheet.
         */
        config[ACCEL].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

        /* The user can change the following configuration parameters according to their requirement. */
        /* Set Output Data Rate */
        config[GYRO].cfg.gyr.odr = BMI2_GYR_ODR_DEFAULT;

        /* Gyroscope Angular Rate Measurement Range.By default the range is 2000dps. */
        config[GYRO].cfg.gyr.range = BMI2_GYR_RANGE_DPS;

        /* Gyroscope bandwidth parameters. By default the gyro bandwidth is in normal mode. */
        config[GYRO].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;

        /* Enable/Disable the noise performance mode for precision yaw rate sensing
         * There are two modes
         *  0 -> Ultra low power mode(Default)
         *  1 -> High performance mode
         */
        config[GYRO].cfg.gyr.noise_perf = BMI2_GYR_NOISE_PERF_DEFAULT;

        /* Enable/Disable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)
         */
        config[GYRO].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

        /* Set the accel and gyro configurations. */
        rslt = bmi2_set_sensor_config(config, 2, bmi);
        bmi2_error_codes_print_result(rslt);
    }

    return rslt;
}

/*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
/*
float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width)
{
    // half_scale = 2^(bit_width-1) computed in float to use the single-precision FPU
    float half_scale = ldexpf(1.0f, (int)bit_width - 1);

    return (GRAVITY_EARTH * val * g_range) / half_scale;
}*/

// even faster version
float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width)
{
    // multiplication by 2^(1 - bit_width) to avoid division
    return ldexpf(GRAVITY_EARTH * (float)val * g_range, 1 - (int)bit_width);
}


/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    // half_scale = 2^(bit_width-1) computed in float to use the single-precision FPU
    float half_scale = ldexpf(1.0f, (int)bit_width - 1);

    return (dps / (half_scale)) * (val);
}
