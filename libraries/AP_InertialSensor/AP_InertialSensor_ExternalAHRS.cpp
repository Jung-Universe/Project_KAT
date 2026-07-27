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

#include <AP_HAL/AP_HAL.h>
#include "AP_InertialSensor_ExternalAHRS.h"
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>

#include <cmath>
#include <stdio.h>

#if HAL_EXTERNAL_AHRS_ENABLED

const extern AP_HAL::HAL& hal;

AP_InertialSensor_ExternalAHRS::AP_InertialSensor_ExternalAHRS(
    AP_InertialSensor &imu,
    uint8_t _serial_port) :
    AP_InertialSensor_Backend(imu),
    serial_port(_serial_port)
{
}

void AP_InertialSensor_ExternalAHRS::handle_external(
    const AP_ExternalAHRS::ins_data_message_t &pkt)
{
    if (!started) {
        return;
    }

    Vector3f accel = pkt.accel;
    Vector3f gyro = pkt.gyro;

    /*
     * 하나의 ExternalAHRS IMU packet에 포함된 gyro와 accel에
     * 동일한 FC 수신 시각을 적용한다.
     */
    const uint64_t sample_us = AP_HAL::micros64();

    _rotate_and_correct_accel(accel_instance, accel);

    _notify_new_accel_raw_sample(
        accel_instance,
        accel,
        sample_us);

    /*
     * MTCAN은 온도를 제공하지 않을 때 -300도를 전달한다.
     * 비정상 온도가 IMU 온도 및 heater 처리로 들어가는 것을 방지한다.
     */
    if (std::isfinite(pkt.temperature) &&
        pkt.temperature > -100.0f &&
        pkt.temperature < 150.0f) {
        _publish_temperature(
            accel_instance,
            pkt.temperature);
    }

    _notify_new_gyro_sensor_rate_sample(
        gyro_instance,
        gyro);

    _rotate_and_correct_gyro(
        gyro_instance,
        gyro);

    _notify_new_gyro_raw_sample(
        gyro_instance,
        gyro,
        sample_us);
}

bool AP_InertialSensor_ExternalAHRS::update(void)
{
    if (started) {
        update_accel(accel_instance);
        update_gyro(gyro_instance);
    }

    return started;
}

void AP_InertialSensor_ExternalAHRS::start()
{
    const float rate =
        AP::externalAHRS().get_IMU_rate();

    if (_imu.register_gyro(
            gyro_instance,
            rate,
            AP_HAL::Device::make_bus_id(
                AP_HAL::Device::BUS_TYPE_SERIAL,
                serial_port,
                1,
                DEVTYPE_SERIAL)) &&
        _imu.register_accel(
            accel_instance,
            rate,
            AP_HAL::Device::make_bus_id(
                AP_HAL::Device::BUS_TYPE_SERIAL,
                serial_port,
                2,
                DEVTYPE_SERIAL))) {
        started = true;
    }
}

void AP_InertialSensor_ExternalAHRS::accumulate()
{
    AP::externalAHRS().update();
}

bool AP_InertialSensor_ExternalAHRS::get_output_banner(
    char* banner,
    uint8_t banner_len)
{
    const char* name =
        AP::externalAHRS().get_name();

    snprintf(
        banner,
        banner_len,
        "IMU%u: External: %s %0.0fHz",
        gyro_instance,
        (name != nullptr) ? name : "",
        AP::externalAHRS().get_IMU_rate());

    return true;
}

#endif // HAL_EXTERNAL_AHRS_ENABLED
