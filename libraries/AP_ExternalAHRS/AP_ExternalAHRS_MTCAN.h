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

/*
 * MTi-680G CAN IMU bridge for ArduPilot.
 * Modified integration: external IMU primary with EKF3 internal-IMU failover.
 */

#pragma once

#include "AP_ExternalAHRS_config.h"

#if HAL_EXTERNAL_AHRS_ENABLED

#include "AP_ExternalAHRS_backend.h"

#include <AP_CANManager/AP_CANSensor.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

class AP_ExternalAHRS_MTCAN :
    public AP_ExternalAHRS_backend,
    public CANSensor
{
public:
    AP_ExternalAHRS_MTCAN(
        AP_ExternalAHRS *_frontend,
        AP_ExternalAHRS::state_t &_state);

    const char *get_name() const override;
    bool healthy() const override;
    bool initialised() const override;

    /*
     * AP_InertialSensor_ExternalAHRS currently creates the external IMU
     * when get_port() >= 0. MTCAN is CAN-based, so this is a stable logical
     * identifier only. The production architecture should eventually use a
     * native AP_InertialSensor CAN backend.
     */
    int8_t get_port() const override
    {
        return 0;
    }

    bool pre_arm_check(
        char *failure_msg,
        uint8_t failure_msg_len) const override;

    void get_filter_status(
        nav_filter_status &status) const override;

    void update() override;

    uint8_t num_gps_sensors() const override
    {
        return 0;
    }

    void handle_frame(
        AP_HAL::CANFrame &frame) override;

private:
    enum class StreamState : uint8_t {
        WAITING_FOR_CAN,
        WAITING_FOR_DATA,
        QUALIFYING,
        HEALTHY,
        LOST,
        RECOVERING,
        RATE_FAULT,
        FROZEN
    };

    struct PendingVector {
        Vector3f value;
        uint64_t rx_time_us = 0;
        bool valid = false;
    };

    struct PostPacket {
        Vector3f gyro;
        Vector3f accel;
        uint64_t sample_time_us = 0;
        bool valid = false;
    };

    // Xsens MT CAN output identifiers used by this integration.
    static constexpr uint16_t CAN_ID_RATE_OF_TURN = 0x032;
    static constexpr uint16_t CAN_ID_ACCELERATION = 0x034;
    static constexpr uint8_t EXPECTED_DLC = 6U;

    // MT CAN protocol scaling. Output is rad/s and m/s/s.
    static constexpr float GYRO_SCALE = 1.0f / 512.0f;
    static constexpr float ACCEL_SCALE = 1.0f / 256.0f;

    static constexpr uint16_t EXPECTED_RATE_HZ = 400U;
    static constexpr float MIN_ACCEPTED_RATE_HZ = 360.0f;
    static constexpr float MAX_ACCEPTED_RATE_HZ = 440.0f;

    // 400 Hz period is 2500 us. Same-cycle gyro/accel frames should arrive
    // well inside this bound. Verify this value using a CAN trace on the
    // final aircraft wiring and bus load.
    static constexpr uint64_t MAX_PAIR_SKEW_US = 1600ULL;

    // Accept a missed sample without declaring the stream invalid, but reject
    // long gaps and timestamp reversals.
    static constexpr uint64_t MIN_PAIR_INTERVAL_US = 1000ULL;
    static constexpr uint64_t MAX_PAIR_INTERVAL_US = 10000ULL;

    // 30 ms corresponds to 12 missing samples at 400 Hz.
    static constexpr uint64_t DATA_TIMEOUT_US = 30000ULL;

    // Require one full second of well-timed pairs at 400 Hz before initial
    // qualification or recovery.
    static constexpr uint16_t QUALIFY_PAIR_COUNT = 400U;
    static constexpr uint8_t RATE_WINDOWS_TO_RECOVER = 2U;

    // FIX(H-2): rate-fault hysteresis. A single 1 s window that falls outside
    // the accepted band no longer forces a fallback; a true disconnect is
    // still caught within DATA_TIMEOUT_US by service_timeout_locked(). This
    // prevents a momentary CAN-frame-loss burst from triggering a spurious
    // mid-flight EKF primary switch (attitude/position jump).
    static constexpr uint8_t RATE_WINDOWS_TO_FAULT = 2U;

    // FIX(H-1): stuck / frozen-sensor detection. A live inertial sensor always
    // carries LSB noise, so a run of bit-identical 400 Hz pairs means the MTi
    // has hung while its CAN peripheral keeps re-transmitting the last buffer.
    // Such a stream passes every rate/skew/saturation gate, so without this
    // check EKF3 would keep fusing dead data on the forced primary lane.
    // 400 repeated pairs is approximately 1.0 s at 400 Hz. This reduces
    // false positives during a perfectly stationary bench test while still
    // rejecting a CAN stream that is re-transmitting a stale sample.
    static constexpr uint32_t FROZEN_PAIR_LIMIT = 400U;

    // FIX(H-3): retry period for CAN driver registration when the first
    // attempt could not claim a driver slot (e.g. CAN interface not up yet).
    static constexpr uint32_t REGISTER_RETRY_MS = 500U;

    static constexpr uint64_t RATE_WINDOW_US = 1000000ULL;
    static constexpr uint32_t TELEMETRY_PERIOD_MS = 1000U;
    static constexpr uint32_t STATUS_TEXT_PERIOD_MS = 5000U;
    static constexpr uint32_t PRIMARY_MONITOR_PERIOD_MS = 200U;
    static constexpr uint32_t CAN_INIT_WARNING_MS = 5000U;

    // ExternalAHRS is inserted before board IMUs in ArduPilot 4.6.2.
    static constexpr uint8_t MTCAN_IMU_INSTANCE = 0U;

    // Reject values at the int16 rail. These indicate clipping, corruption,
    // or a sensor range violation and must not be fused.
    static constexpr int16_t RAW_SATURATION_LIMIT = 32760;

    // ---------------------------------------------------------------------
    // FLIGHT-CRITICAL: mounting transform from MTi sensor axes to autopilot
    // board axes. If the MTi is not mounted in perfect alignment with the FC
    // body frame, this MUST be set to the matching rotation, otherwise the
    // vehicle fuses mis-axed gyro/accel and can flip on takeoff.
    //
    // ROTATION_NONE means "MTi X/Y/Z axes == FC X/Y/Z axes". Change this
    // single constant to match the physical installation. The active value
    // is announced over MAVLink once at boot ("MTCAN: sensor rotation ...")
    // so it can be verified on the bench before any flight.
    // ---------------------------------------------------------------------
    // MTi raw sensor body frame is Z-up / right-handed; ArduPilot expects a
    // Z-down (FRD) body frame. ROLL_180 negates Y and Z to convert between
    // them, correcting the inverted-horizon seen with ROTATION_NONE. Confirm
    // on the bench with the tilt test (level=level, nose-up=pitch-up,
    // roll-right=roll-right) and adjust if any single axis is still reversed.
    static constexpr Rotation SENSOR_ROTATION = ROTATION_ROLL_180;

    static int16_t read_be_i16(const uint8_t *data);
    static float calculate_hz(
        uint32_t count,
        uint64_t first_us,
        uint64_t last_us);

    bool decode_vector(
        const uint8_t *data,
        float scale,
        Vector3f &value) const;

    void store_gyro_locked(
        const Vector3f &gyro,
        uint64_t now_us);

    void store_accel_locked(
        const Vector3f &accel,
        uint64_t now_us);

    bool make_pair_locked(
        PostPacket &packet);

    void count_gyro_frame_locked(uint64_t now_us);
    void count_accel_frame_locked(uint64_t now_us);
    void count_pair_locked(uint64_t now_us);
    void count_delivery_locked(uint64_t now_us);

    void update_rate_window_locked(uint64_t now_us);
    void service_timeout_locked(uint64_t now_us);
    void clear_pending_locked();

    bool stream_healthy_locked(
        uint64_t now_us,
        bool can_ready) const;

    void monitor_primary_imu(uint32_t now_ms);
    void send_telemetry(uint32_t now_ms);
    void send_periodic_status(uint32_t now_ms);

    mutable HAL_Semaphore _sem;

    PendingVector _pending_gyro;
    PendingVector _pending_accel;

    Vector3f _latest_gyro;
    Vector3f _latest_accel;

    uint64_t _last_gyro_us = 0;
    uint64_t _last_accel_us = 0;
    uint64_t _last_pair_us = 0;
    uint64_t _last_delivery_us = 0;

    bool _registration_attempted = false;
    bool _ever_initialised = false;
    bool _qualified = false;
    bool _rate_window_valid = false;
    // Latest completed one-second window result. Initial qualification and
    // pre-arm require this to be true. In-flight health still uses the
    // two-window RATE_WINDOWS_TO_FAULT hysteresis.
    bool _last_rate_window_good = false;
    bool _rate_fault = false;
    bool _timeout_latched = false;

    StreamState _stream_state = StreamState::WAITING_FOR_CAN;

    uint16_t _consecutive_good_pairs = 0;
    uint8_t _consecutive_good_rate_windows = 0;
    uint8_t _consecutive_bad_rate_windows = 0;   // FIX(H-2)

    // FIX(H-1): frozen-sensor detection state (all guarded by _sem).
    Vector3f _prev_delivered_gyro;
    Vector3f _prev_delivered_accel;
    bool _have_prev_delivered = false;
    bool _frozen_fault = false;
    uint32_t _frozen_pair_count = 0;
    uint32_t _frozen_fault_count = 0;

    uint32_t _gyro_frame_count = 0;
    uint64_t _gyro_first_us = 0;
    uint64_t _gyro_last_us = 0;

    uint32_t _accel_frame_count = 0;
    uint64_t _accel_first_us = 0;
    uint64_t _accel_last_us = 0;

    uint32_t _pair_count = 0;
    uint64_t _pair_first_us = 0;
    uint64_t _pair_last_us = 0;

    uint32_t _delivery_count = 0;
    uint64_t _delivery_first_us = 0;
    uint64_t _delivery_last_us = 0;

    uint64_t _rate_window_start_us = 0;

    float _gyro_rx_hz = 0.0f;
    float _accel_rx_hz = 0.0f;
    float _pair_hz = 0.0f;
    float _delivery_hz = 0.0f;

    uint32_t _bad_dlc_count = 0;
    uint32_t _saturation_count = 0;
    uint32_t _pair_skew_drop_count = 0;
    uint32_t _unpaired_overwrite_count = 0;
    uint32_t _timing_error_count = 0;
    uint32_t _timeout_count = 0;
    uint32_t _rate_fault_count = 0;

    bool _health_announced = false;
    bool _last_announced_health = false;
    bool _can_init_warning_sent = false;
    bool _startup_announced = false;

    uint8_t _last_primary_gyro = 0xFFU;
    uint8_t _last_primary_accel = 0xFFU;
    int8_t _last_primary_core = -2;

    uint32_t _last_telemetry_ms = 0;
    uint32_t _last_status_text_ms = 0;
    uint32_t _last_primary_monitor_ms = 0;
    uint32_t _last_register_ms = 0;          // FIX(H-3)
};

#endif // HAL_EXTERNAL_AHRS_ENABLED
