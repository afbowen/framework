#include "driver_invensense.h"
#include "invensense_internal.h"

#include <modules/uavcan_debug/uavcan_debug.h>

static void invensense_read(struct invensense_instance_s* instance, uint8_t reg, size_t len, void* buf);

void invensense_init(struct invensense_instance_s* instance, uint8_t spi_idx, uint32_t select_line, enum invensense_imu_type_t imu_type) {
    if (!instance) {
        return;
    }
    
    spi_device_init(&instance->spi_dev, spi_idx, select_line, 10000, 8, SPI_DEVICE_FLAG_CPHA|SPI_DEVICE_FLAG_CPOL);
    
    uint8_t whoami = 0;
    invensense_read(instance, INVENSENSE_REG_WHOAMI, sizeof(uint8_t), &whoami);
    
    uavcan_send_debug_msg(LOG_LEVEL_DEBUG, "invensense", "whoami=0x%02X", whoami);
    
    spi_device_set_max_speed_hz(&instance->spi_dev, 10000000);
}

static void invensense_read(struct invensense_instance_s* instance, uint8_t reg, size_t len, void* buf) {
    reg |= 0x80;
    
    spi_device_begin(&instance->spi_dev);
    spi_device_send(&instance->spi_dev, sizeof(reg), &reg);
    spi_device_receive(&instance->spi_dev, len, buf);
    spi_device_end(&instance->spi_dev);
}
