#pragma once

#include "driver_invensense.h"

#define INVENSENSE_REG_WHOAMI 0x75

uint8_t invensense_get_whoami(enum invensense_imu_type_t imu_type);
