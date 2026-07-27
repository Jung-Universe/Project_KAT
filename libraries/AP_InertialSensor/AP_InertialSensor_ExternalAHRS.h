/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <AP_ExternalAHRS/AP_ExternalAHRS.h>

#if HAL_EXTERNAL_AHRS_ENABLED

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

class AP_InertialSensor_ExternalAHRS : public AP_InertialSensor_Backend
{
public:
    AP_InertialSensor_ExternalAHRS(
        AP_InertialSensor &imu,
        uint8_t serial_port);

    bool update() override;
    void start() override;
    void accumulate() override;

    void handle_external(
        const AP_ExternalAHRS::ins_data_message_t &pkt) override;

    bool get_output_banner(
        char *banner,
        uint8_t banner_len) override;

private:
    const uint8_t serial_port;

    // Never leave this uninitialised: startup data acceptance must be deterministic.
    bool started = false;
};

#endif // HAL_EXTERNAL_AHRS_ENABLED
