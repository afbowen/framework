#pragma once

#include <modules/spi_device/spi_device.h>

enum invensense_imu_type_t {
    INVENSENSE_IMU_TYPE_MPU6000,
    INVENSENSE_IMU_TYPE_MPU6500,
    INVENSENSE_IMU_TYPE_MPU9250,
    INVENSENSE_IMU_TYPE_MPU9255,
    INVENSENSE_IMU_TYPE_ICM20608,
    INVENSENSE_IMU_TYPE_ICM20602,
};

struct invensense_instance_s {
    struct spi_device_s spi_dev;
    enum invensense_imu_type_t imu_type;
};

typedef struct{
    float x;
    float y;
    float z;
}   accel_data_t;

typedef enum{
    ACCEL_FS_2g,
    ACCEL_FS_4g,
    ACCEL_FS_8g,
    ACCEL_FS_16g
}   accel_scale_t;

typedef struct{
    float x;
    float y;
    float z;
}   gyro_data_t;


typedef enum{
    GYRO_FS_250dps,
    GYRO_FS_500dps,
    GYRO_FS_1000dps,
    GYRO_FS_2000dps
}   gyro_scale_t;

typedef enum{
    ACCEL_FIFO_EN,
    GYRO_FIFO_EN,
    ACCEL_GYRO_FIFO_EN
}   fifo_setting_t;

typedef struct{
    float accel_x;
    float accel_y;
    float accel_z;
    float temp;
    float gyro_x;
    float gyro_y;
    float gyro_z;
}   fifo_data_t;


void invensense_init(struct invensense_instance_s* instance, uint8_t spi_idx, uint32_t select_line, enum invensense_imu_type_t imu_type, accel_scale_t accel_scale_g, gyro_scale_t gyro_scale_dps);
