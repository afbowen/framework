/*
 *    ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio
 *
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *        http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 */

#include <ch.h>
#include <modules/uavcan_debug/uavcan_debug.h>
#include <modules/param/param.h>
#include <common/helpers.h>
#include <string.h>
#include <modules/driver_invensense/driver_invensense.h>

int main(void) {
    chThdSleep(S2ST(3));
    struct invensense_instance_s invensense;
    invensense_init(&invensense, 3, BOARD_PAL_LINE_SPI3_ICM_CS, INVENSENSE_IMU_TYPE_ICM20602, ACCEL_FS_2g, GYRO_FS_250dps);

    chThdSleep(TIME_INFINITE);

    return 0;
}
