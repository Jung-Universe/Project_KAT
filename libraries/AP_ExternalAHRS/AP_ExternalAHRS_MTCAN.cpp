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

#include "AP_ExternalAHRS_MTCAN.h"

#if HAL_EXTERNAL_AHRS_ENABLED

#include <AP_AHRS/AP_AHRS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <GCS_MAVLink/GCS.h>

#include <cstring>

extern const AP_HAL::HAL &hal;

AP_ExternalAHRS_MTCAN::AP_ExternalAHRS_MTCAN(
    AP_ExternalAHRS *_frontend,
    AP_ExternalAHRS::state_t &_state) :
    AP_ExternalAHRS_backend(_frontend, _state),
    // FIX(C-1): the CANSensor default stack (2048 B) is sized for lightweight
    // rangefinder parsing. handle_frame() runs on this CANSensor thread and
    // calls AP::ins().handle_external(), which executes the full IMU raw-sample
    // pipeline inline: coning/sculling integration, apply_gyro/accel_filters
    // (harmonic-notch biquad banks) and log_gyro/accel_raw. That was never
    // meant to run on a 2 kB stack. Give it 8 kB and verify the real
    // high-water mark on hardware before delivery.
    CANSensor("MTCAN", 8192)
{
    // IMU-only backend. The parameter EAHRS_SENSORS must also be set to 2
    // because set_default_sensors() does not overwrite an already-saved value.
    set_default_sensors(
        uint16_t(AP_ExternalAHRS::AvailableSensor::IMU));
}

const char *AP_ExternalAHRS_MTCAN::get_name() const
{
    return "MTCAN";
}

int16_t AP_ExternalAHRS_MTCAN::read_be_i16(const uint8_t *data)
{
    const uint16_t value =
        (uint16_t(data[0]) << 8) |
        uint16_t(data[1]);

    return int16_t(value);
}

float AP_ExternalAHRS_MTCAN::calculate_hz(
    const uint32_t count,
    const uint64_t first_us,
    const uint64_t last_us)
{
    if (count < 2U || last_us <= first_us) {
        return 0.0f;
    }

    return float(count - 1U) *
           1000000.0f /
           float(last_us - first_us);
}

bool AP_ExternalAHRS_MTCAN::decode_vector(
    const uint8_t *data,
    const float scale,
    Vector3f &value) const
{
    const int16_t raw_x = read_be_i16(&data[0]);
    const int16_t raw_y = read_be_i16(&data[2]);
    const int16_t raw_z = read_be_i16(&data[4]);

    if (raw_x <= -RAW_SATURATION_LIMIT ||
        raw_x >= RAW_SATURATION_LIMIT ||
        raw_y <= -RAW_SATURATION_LIMIT ||
        raw_y >= RAW_SATURATION_LIMIT ||
        raw_z <= -RAW_SATURATION_LIMIT ||
        raw_z >= RAW_SATURATION_LIMIT) {
        return false;
    }

    value = Vector3f(
        float(raw_x) * scale,
        float(raw_y) * scale,
        float(raw_z) * scale);

    value.rotate(SENSOR_ROTATION);

    return !value.is_nan() && !value.is_inf();
}

void AP_ExternalAHRS_MTCAN::count_gyro_frame_locked(
    const uint64_t now_us)
{
    if (_gyro_frame_count == 0U) {
        _gyro_first_us = now_us;
    }

    _gyro_last_us = now_us;
    _gyro_frame_count++;
}

void AP_ExternalAHRS_MTCAN::count_accel_frame_locked(
    const uint64_t now_us)
{
    if (_accel_frame_count == 0U) {
        _accel_first_us = now_us;
    }

    _accel_last_us = now_us;
    _accel_frame_count++;
}

void AP_ExternalAHRS_MTCAN::count_pair_locked(
    const uint64_t now_us)
{
    if (_pair_count == 0U) {
        _pair_first_us = now_us;
    }

    _pair_last_us = now_us;
    _pair_count++;
}

void AP_ExternalAHRS_MTCAN::count_delivery_locked(
    const uint64_t now_us)
{
    if (_delivery_count == 0U) {
        _delivery_first_us = now_us;
    }

    _delivery_last_us = now_us;
    _delivery_count++;
}

void AP_ExternalAHRS_MTCAN::clear_pending_locked()
{
    _pending_gyro.valid = false;
    _pending_accel.valid = false;
}

void AP_ExternalAHRS_MTCAN::store_gyro_locked(
    const Vector3f &gyro,
    const uint64_t now_us)
{
    if (_pending_gyro.valid) {
        // A second gyro arrived before a matching accel. Discard the older
        // gyro instead of creating a cross-cycle pair.
        _unpaired_overwrite_count++;
        _consecutive_good_pairs = 0U;
    }

    _pending_gyro.value = gyro;
    _pending_gyro.rx_time_us = now_us;
    _pending_gyro.valid = true;

    _latest_gyro = gyro;
    _last_gyro_us = now_us;

    count_gyro_frame_locked(now_us);
}

void AP_ExternalAHRS_MTCAN::store_accel_locked(
    const Vector3f &accel,
    const uint64_t now_us)
{
    if (_pending_accel.valid) {
        // A second accel arrived before a matching gyro. Discard the older
        // accel instead of creating a cross-cycle pair.
        _unpaired_overwrite_count++;
        _consecutive_good_pairs = 0U;
    }

    _pending_accel.value = accel;
    _pending_accel.rx_time_us = now_us;
    _pending_accel.valid = true;

    _latest_accel = accel;
    _last_accel_us = now_us;

    count_accel_frame_locked(now_us);
}

bool AP_ExternalAHRS_MTCAN::make_pair_locked(
    PostPacket &packet)
{
    if (!_pending_gyro.valid || !_pending_accel.valid) {
        return false;
    }

    const uint64_t gyro_us = _pending_gyro.rx_time_us;
    const uint64_t accel_us = _pending_accel.rx_time_us;

    const uint64_t older_us = MIN(gyro_us, accel_us);
    const uint64_t newer_us = MAX(gyro_us, accel_us);
    const uint64_t skew_us = newer_us - older_us;

    if (skew_us > MAX_PAIR_SKEW_US) {
        // Discard only the older frame. The newer frame may still pair with
        // the next frame of the other type.
        if (gyro_us < accel_us) {
            _pending_gyro.valid = false;
        } else {
            _pending_accel.valid = false;
        }

        _pair_skew_drop_count++;
        _consecutive_good_pairs = 0U;
        return false;
    }

    // Use one common timestamp for gyro and accel. It is based on FC receive
    // time because the current CAN profile does not include Xsens SampleTime.
    const uint64_t pair_time_us = older_us + (skew_us / 2ULL);
    bool timing_valid = true;

    if (_last_pair_us != 0U) {
        if (pair_time_us <= _last_pair_us) {
            _timing_error_count++;
            _consecutive_good_pairs = 0U;
            clear_pending_locked();
            return false;
        }

        const uint64_t interval_us = pair_time_us - _last_pair_us;

        if (interval_us < MIN_PAIR_INTERVAL_US ||
            interval_us > MAX_PAIR_INTERVAL_US) {
            _timing_error_count++;
            _consecutive_good_pairs = 0U;
            timing_valid = false;
        } else if (_consecutive_good_pairs < UINT16_MAX) {
            _consecutive_good_pairs++;
        }
    } else {
        _consecutive_good_pairs = 1U;
    }

    packet.gyro = _pending_gyro.value;
    packet.accel = _pending_accel.value;
    packet.sample_time_us = pair_time_us;  /////MTCAN 내부 전용 시간값(gyro·accel pair 시각 계산, pair 간격 계산, 400Hz rate 계산, delivery rate 계산, timeout 및 상태 감시)

    _last_pair_us = pair_time_us;
    count_pair_locked(pair_time_us);

    clear_pending_locked();

    _timeout_latched = false;

    /*
     * FIX(H-1): frozen / stuck-sensor detection.
     *
     * A physical MTi always injects LSB noise, so two consecutive 400 Hz
     * pairs are effectively never bit-identical on all six channels at once.
     * A run of identical pairs therefore means the sensor MCU has hung while
     * its CAN peripheral keeps re-transmitting the last frame. This stream
     * passes every rate/skew/saturation gate, so it must be rejected here or
     * EKF3 will keep fusing dead data on the forced primary lane.
     *
     * While values are frozen we also pin _consecutive_good_pairs to 0 so the
     * stream can never (re)qualify on stale data. Once the values start moving
     * again the fault clears and normal qualification rebuilds from scratch.
     */
    const bool values_moving =
        !_have_prev_delivered ||
        !(packet.gyro == _prev_delivered_gyro &&
          packet.accel == _prev_delivered_accel);

    _prev_delivered_gyro = packet.gyro;
    _prev_delivered_accel = packet.accel;
    _have_prev_delivered = true;

    if (values_moving) {
        _frozen_pair_count = 0U;
        _frozen_fault = false;
    } else {
        if (_frozen_pair_count < UINT32_MAX) {
            _frozen_pair_count++;
        }
        // A frozen pair can never contribute to qualification.
        _consecutive_good_pairs = 0U;

        if (_frozen_pair_count >= FROZEN_PAIR_LIMIT) {
            if (!_frozen_fault) {
                _frozen_fault_count++;
            }
            _frozen_fault = true;
            _qualified = false;
            _stream_state = StreamState::FROZEN;
        }
    }

    const bool qualification_complete =
        _rate_window_valid &&
        _last_rate_window_good &&
        !_rate_fault &&
        !_frozen_fault &&
        _consecutive_good_pairs >= QUALIFY_PAIR_COUNT;

    if (_rate_fault) {
        _qualified = false;
        _stream_state = StreamState::RATE_FAULT;
    } else if (_frozen_fault) {
        _qualified = false;
        _stream_state = StreamState::FROZEN;
    } else if (qualification_complete) {
        _qualified = true;
        _stream_state = _ever_initialised ?
            StreamState::RECOVERING :
            StreamState::QUALIFYING;
    } else if (!_ever_initialised || !_qualified) {
        // During first qualification or post-fault recovery, keep the lane
        // blocked until the complete gate passes. If the lane was already
        // healthy, preserve _qualified through one bad rate window so the
        // two-window rate-fault hysteresis actually works in flight.
        _qualified = false;
        _stream_state = _ever_initialised ?
            StreamState::RECOVERING :
            StreamState::QUALIFYING;
    }

    /*
     * At first boot, deliver samples immediately so the MTi EKF lane can
     * initialise as core 0. After a real loss or rate fault, suppress delivery
     * until the stream has passed the full recovery qualification. This
     * prevents a reconnect transient from being marked healthy by
     * AP_InertialSensor.
     */
    const bool recovery_gate_open =
        !_ever_initialised || _qualified;

    packet.valid =
        timing_valid &&
        !_rate_fault &&
        !_frozen_fault &&        // FIX(H-1)
        recovery_gate_open;

    return packet.valid;
}

void AP_ExternalAHRS_MTCAN::update_rate_window_locked(
    const uint64_t now_us)
{
    /*
     * Do not create a false rate fault merely because the MTi powers up
     * later than the flight controller. Start the measurement window only
     * after at least one relevant CAN frame has arrived.
     */
    if (_last_gyro_us == 0U &&
        _last_accel_us == 0U) {
        _rate_window_start_us = now_us;
        return;
    }

    if (_rate_window_start_us == 0U) {
        _rate_window_start_us = now_us;
        return;
    }

    if ((now_us - _rate_window_start_us) < RATE_WINDOW_US) {
        return;
    }

    _gyro_rx_hz = calculate_hz(
        _gyro_frame_count,
        _gyro_first_us,
        _gyro_last_us);

    _accel_rx_hz = calculate_hz(
        _accel_frame_count,
        _accel_first_us,
        _accel_last_us);

    _pair_hz = calculate_hz(
        _pair_count,
        _pair_first_us,
        _pair_last_us);

    _delivery_hz = calculate_hz(
        _delivery_count,
        _delivery_first_us,
        _delivery_last_us);

    _rate_window_valid = true;

    const bool rate_good =
        _gyro_rx_hz >= MIN_ACCEPTED_RATE_HZ &&
        _gyro_rx_hz <= MAX_ACCEPTED_RATE_HZ &&
        _accel_rx_hz >= MIN_ACCEPTED_RATE_HZ &&
        _accel_rx_hz <= MAX_ACCEPTED_RATE_HZ &&
        _pair_hz >= MIN_ACCEPTED_RATE_HZ &&
        _pair_hz <= MAX_ACCEPTED_RATE_HZ;

    _last_rate_window_good = rate_good;

    if (rate_good) {
        _consecutive_bad_rate_windows = 0U;

        if (_consecutive_good_rate_windows <
            RATE_WINDOWS_TO_RECOVER) {
            _consecutive_good_rate_windows++;
        }

        if (_consecutive_good_rate_windows >=
            RATE_WINDOWS_TO_RECOVER) {
            _rate_fault = false;
        }
    } else {
        _consecutive_good_rate_windows = 0U;

        /*
         * FIX(H-2): require RATE_WINDOWS_TO_FAULT consecutive out-of-band
         * windows before declaring a rate fault. A single bad 1 s window
         * (e.g. a short CAN-frame-loss burst from EMI or arbitration) no
         * longer forces an EKF fallback. This does NOT weaken loss detection:
         * a genuine disconnect is still caught within DATA_TIMEOUT_US (30 ms)
         * by service_timeout_locked(), independently of this window logic.
         */
        if (_consecutive_bad_rate_windows < RATE_WINDOWS_TO_FAULT) {
            _consecutive_bad_rate_windows++;
        }

        if (_consecutive_bad_rate_windows >= RATE_WINDOWS_TO_FAULT) {
            if (!_rate_fault) {
                _rate_fault_count++;
            }

            _rate_fault = true;
            _qualified = false;
            _consecutive_good_pairs = 0U;
            _stream_state = StreamState::RATE_FAULT;
        }
    }

    _gyro_frame_count = 0U;
    _gyro_first_us = 0U;
    _gyro_last_us = 0U;

    _accel_frame_count = 0U;
    _accel_first_us = 0U;
    _accel_last_us = 0U;

    _pair_count = 0U;
    _pair_first_us = 0U;
    _pair_last_us = 0U;

    _delivery_count = 0U;
    _delivery_first_us = 0U;
    _delivery_last_us = 0U;

    _rate_window_start_us = now_us;
}

void AP_ExternalAHRS_MTCAN::service_timeout_locked(
    const uint64_t now_us)
{
    if (_last_pair_us == 0U) {
        _stream_state = CANSensor::initialized() ?
            StreamState::WAITING_FOR_DATA :
            StreamState::WAITING_FOR_CAN;
        return;
    }

    if ((now_us - _last_pair_us) <= DATA_TIMEOUT_US &&
        (now_us - _last_gyro_us) <= DATA_TIMEOUT_US &&
        (now_us - _last_accel_us) <= DATA_TIMEOUT_US) {
        return;
    }

    if (!_timeout_latched) {
        _timeout_latched = true;
        _timeout_count++;
    }

    _qualified = false;
    _consecutive_good_pairs = 0U;
    _consecutive_good_rate_windows = 0U;
    _consecutive_bad_rate_windows = 0U;   // FIX(H-2)
    _rate_window_valid = false;
    _last_rate_window_good = false;
    _rate_fault = true;
    _stream_state = StreamState::LOST;

    // FIX(H-1): forget frozen-detection history so a reconnecting sensor is
    // re-evaluated from a clean slate.
    _frozen_fault = false;
    _frozen_pair_count = 0U;
    _have_prev_delivered = false;

    clear_pending_locked();

    /*
     * Discard all pre-fault rate statistics. Recovery must be proven using
     * new post-reconnect data only.
     */
    _gyro_frame_count = 0U;
    _gyro_first_us = 0U;
    _gyro_last_us = 0U;

    _accel_frame_count = 0U;
    _accel_first_us = 0U;
    _accel_last_us = 0U;

    _pair_count = 0U;
    _pair_first_us = 0U;
    _pair_last_us = 0U;

    _delivery_count = 0U;
    _delivery_first_us = 0U;
    _delivery_last_us = 0U;

    _rate_window_start_us = now_us;

    // Prevent stale 400 Hz values from remaining visible after disconnect.
    _gyro_rx_hz = 0.0f;
    _accel_rx_hz = 0.0f;
    _pair_hz = 0.0f;
    _delivery_hz = 0.0f;
}

bool AP_ExternalAHRS_MTCAN::stream_healthy_locked(
    const uint64_t now_us,
    const bool can_ready) const
{
    if (!can_ready ||
        !_qualified ||
        !_rate_window_valid ||
        _rate_fault ||
        _frozen_fault ||          // FIX(H-1)
        _last_gyro_us == 0U ||
        _last_accel_us == 0U ||
        _last_pair_us == 0U) {
        return false;
    }

    return
        (now_us - _last_gyro_us) <= DATA_TIMEOUT_US &&
        (now_us - _last_accel_us) <= DATA_TIMEOUT_US &&
        (now_us - _last_pair_us) <= DATA_TIMEOUT_US;
}

void AP_ExternalAHRS_MTCAN::handle_frame(
    AP_HAL::CANFrame &frame)
{
    if (frame.isExtended() ||
        frame.isRemoteTransmissionRequest() ||
        frame.isErrorFrame()) {
        return;
    }

    const uint16_t can_id =
        uint16_t(frame.id &
                 AP_HAL::CANFrame::MaskStdID);

    if (can_id != CAN_ID_RATE_OF_TURN &&
        can_id != CAN_ID_ACCELERATION) {
        return;
    }

    if (frame.dlc != EXPECTED_DLC) {
        WITH_SEMAPHORE(_sem);
        _bad_dlc_count++;
        return;
    }

    Vector3f decoded;
    const float scale =
        (can_id == CAN_ID_RATE_OF_TURN) ?
        GYRO_SCALE :
        ACCEL_SCALE;

    if (!decode_vector(frame.data, scale, decoded)) {
        WITH_SEMAPHORE(_sem);
        _saturation_count++;
        _consecutive_good_pairs = 0U;
        return;
    }

    const uint64_t now_us = AP_HAL::micros64();

    PostPacket packet;

    {
        WITH_SEMAPHORE(_sem);

        update_rate_window_locked(now_us);

        if (can_id == CAN_ID_RATE_OF_TURN) {
            store_gyro_locked(decoded, now_us);
        } else {
            store_accel_locked(decoded, now_us);
        }

        make_pair_locked(packet);
    }

    if (!packet.valid) {
        return;
    }

    {
        WITH_SEMAPHORE(state.sem);
        state.gyro = packet.gyro;
        state.accel = packet.accel;
    }

    AP_ExternalAHRS::ins_data_message_t ins{};
    ins.gyro = packet.gyro;
    ins.accel = packet.accel;
    //ins.sample_time_us = packet.sample_time_us;

    // Sentinel for "temperature unavailable". The accompanying
    // AP_InertialSensor_ExternalAHRS patch rejects non-physical values.
    ins.temperature = -300.0f;

    AP::ins().handle_external(ins);

    {
        WITH_SEMAPHORE(_sem);
        _last_delivery_us = packet.sample_time_us;
        count_delivery_locked(packet.sample_time_us);
    }
}

void AP_ExternalAHRS_MTCAN::monitor_primary_imu(
    const uint32_t now_ms)
{
    if ((now_ms - _last_primary_monitor_ms) <
        PRIMARY_MONITOR_PERIOD_MS) {
        return;
    }

    _last_primary_monitor_ms = now_ms;

    AP_AHRS *ahrs = AP_AHRS::get_singleton();

    if (ahrs == nullptr) {
        return;
    }

    const int8_t primary_core =
        ahrs->get_primary_core_index();

    if (primary_core < 0) {
        return;
    }

    const uint8_t primary_gyro =
        ahrs->get_primary_gyro_index();

    const uint8_t primary_accel =
        ahrs->get_primary_accel_index();

    if (primary_core == _last_primary_core &&
        primary_gyro == _last_primary_gyro &&
        primary_accel == _last_primary_accel) {
        return;
    }

    const bool had_previous =
        _last_primary_core >= 0;

    _last_primary_core = primary_core;
    _last_primary_gyro = primary_gyro;
    _last_primary_accel = primary_accel;

    if (primary_gyro == MTCAN_IMU_INSTANCE &&
        primary_accel == MTCAN_IMU_INSTANCE) {
        GCS_SEND_TEXT(
            MAV_SEVERITY_INFO,
            "EKF PRIMARY: MTCAN IMU0 active");
        return;
    }

    if (primary_gyro == primary_accel) {
        GCS_SEND_TEXT(
            had_previous ?
                MAV_SEVERITY_WARNING :
                MAV_SEVERITY_INFO,
            "EKF FALLBACK: internal IMU%u active",
            unsigned(primary_gyro));
        return;
    }

    GCS_SEND_TEXT(
        MAV_SEVERITY_WARNING,
        "EKF MIXED IMU: gyro%u accel%u",
        unsigned(primary_gyro),
        unsigned(primary_accel));
}

void AP_ExternalAHRS_MTCAN::send_telemetry(
    const uint32_t now_ms)
{
    if ((now_ms - _last_telemetry_ms) <
        TELEMETRY_PERIOD_MS) {
        return;
    }

    _last_telemetry_ms = now_ms;

    Vector3f gyro;
    Vector3f accel;

    float gyro_hz;
    float accel_hz;
    float pair_hz;
    float delivery_hz;

    bool is_healthy;

    uint32_t drops;
    uint32_t frozen_faults;
    uint8_t bad_rate_windows;
    bool last_rate_good;   // FIX(H-1)

    {
        WITH_SEMAPHORE(_sem);

        const uint64_t now_us = AP_HAL::micros64();
        const bool can_ready = CANSensor::initialized();

        gyro = _latest_gyro;
        accel = _latest_accel;

        gyro_hz = _gyro_rx_hz;
        accel_hz = _accel_rx_hz;
        pair_hz = _pair_hz;
        delivery_hz = _delivery_hz;

        is_healthy =
            stream_healthy_locked(now_us, can_ready);

        frozen_faults = _frozen_fault_count;
        bad_rate_windows = _consecutive_bad_rate_windows;
        last_rate_good = _last_rate_window_good;

        drops =
            _bad_dlc_count +
            _saturation_count +
            _pair_skew_drop_count +
            _unpaired_overwrite_count +
            _timing_error_count +
            _frozen_fault_count;   // FIX(H-1)
    }

    gcs().send_named_float("MTC_AX", accel.x);
    gcs().send_named_float("MTC_AY", accel.y);
    gcs().send_named_float("MTC_AZ", accel.z);

    gcs().send_named_float("MTC_GX", gyro.x);
    gcs().send_named_float("MTC_GY", gyro.y);
    gcs().send_named_float("MTC_GZ", gyro.z);

    gcs().send_named_float("MTC_AHZ", accel_hz);
    gcs().send_named_float("MTC_GHZ", gyro_hz);
    gcs().send_named_float("MTC_PHZ", pair_hz);
    gcs().send_named_float("MTC_IHZ", delivery_hz);

    gcs().send_named_float(
        "MTC_HLTH",
        is_healthy ? 1.0f : 0.0f);

    gcs().send_named_float(
        "MTC_DROP",
        float(drops));

    // FIX(H-1): expose frozen-sensor fault latches separately for diagnosis.
    gcs().send_named_float(
        "MTC_FRZ",
        float(frozen_faults));

    gcs().send_named_float(
        "MTC_BADW",
        float(bad_rate_windows));

    gcs().send_named_float(
        "MTC_RGOOD",
        last_rate_good ? 1.0f : 0.0f);

    gcs().send_named_float(
        "MTC_PGYR",
        float(_last_primary_gyro));

    gcs().send_named_float(
        "MTC_PACC",
        float(_last_primary_accel));
}

void AP_ExternalAHRS_MTCAN::send_periodic_status(
    const uint32_t now_ms)
{
    if ((now_ms - _last_status_text_ms) <
        STATUS_TEXT_PERIOD_MS) {
        return;
    }

    float gyro_hz;
    float accel_hz;
    float pair_hz;
    float delivery_hz;
    bool is_healthy;

    {
        WITH_SEMAPHORE(_sem);

        const uint64_t now_us = AP_HAL::micros64();

        gyro_hz = _gyro_rx_hz;
        accel_hz = _accel_rx_hz;
        pair_hz = _pair_hz;
        delivery_hz = _delivery_hz;

        is_healthy = stream_healthy_locked(
            now_us,
            CANSensor::initialized());
    }

    if (!is_healthy) {
        return;
    }

    _last_status_text_ms = now_ms;

    const uint32_t gyro_hz_x10 =
        uint32_t(MAX(gyro_hz, 0.0f) *
                 10.0f + 0.5f);

    const uint32_t accel_hz_x10 =
        uint32_t(MAX(accel_hz, 0.0f) *
                 10.0f + 0.5f);

    const uint32_t pair_hz_x10 =
        uint32_t(MAX(pair_hz, 0.0f) *
                 10.0f + 0.5f);

    const uint32_t delivery_hz_x10 =
        uint32_t(MAX(delivery_hz, 0.0f) *
                 10.0f + 0.5f);

    GCS_SEND_TEXT(
        MAV_SEVERITY_INFO,
        "MTCAN OK G:%lu.%lu A:%lu.%lu P:%lu.%lu I:%lu.%lu",
        (unsigned long)(gyro_hz_x10 / 10U),
        (unsigned long)(gyro_hz_x10 % 10U),
        (unsigned long)(accel_hz_x10 / 10U),
        (unsigned long)(accel_hz_x10 % 10U),
        (unsigned long)(pair_hz_x10 / 10U),
        (unsigned long)(pair_hz_x10 % 10U),
        (unsigned long)(delivery_hz_x10 / 10U),
        (unsigned long)(delivery_hz_x10 % 10U));
}

void AP_ExternalAHRS_MTCAN::update()
{
    const uint32_t now_ms = AP_HAL::millis();
    const uint64_t now_us = AP_HAL::micros64();

    /*
     * FIX(H-3): register the CAN driver and keep retrying (throttled) until a
     * driver slot is actually claimed. The previous single-shot attempt left
     * the sensor permanently dead until reboot if the first call ran before
     * the CAN interface was up. register_driver() is idempotent: AP_CANManager
     * refuses to re-register an already-populated slot, so repeated calls when
     * the parameters are misconfigured are harmless.
     */
    if (!CANSensor::initialized()) {
        if (!_registration_attempted ||
            (now_ms - _last_register_ms) >= REGISTER_RETRY_MS) {
            _registration_attempted = true;
            _last_register_ms = now_ms;
            register_driver(AP_CAN::Protocol::MTCAN);
        }
    }

    // One-shot boot banner so the configured mounting rotation can be verified
    // on the bench before any flight, even with no CAN data connected yet.
    // A wrong SENSOR_ROTATION is flight-critical, so make it loud rather than
    // silent. ROTATION_NONE (0) means MTi axes are assumed identical to the FC.
    if (!_startup_announced) {
        _startup_announced = true;
        GCS_SEND_TEXT(
            MAV_SEVERITY_INFO,
            "MTCAN: sensor rotation %u (verify vs mounting)",
            unsigned(SENSOR_ROTATION));
    }

    const bool can_ready =
        CANSensor::initialized();

    if (!can_ready &&
        !_can_init_warning_sent &&
        now_ms >= CAN_INIT_WARNING_MS) {
        _can_init_warning_sent = true;

        GCS_SEND_TEXT(
            MAV_SEVERITY_CRITICAL,
            "MTCAN: CAN driver not initialised");
    }

    bool is_healthy;
    bool first_initialisation = false;

    {
        WITH_SEMAPHORE(_sem);

        update_rate_window_locked(now_us);
        service_timeout_locked(now_us);

        is_healthy =
            stream_healthy_locked(now_us, can_ready);

        if (is_healthy) {
            _stream_state = StreamState::HEALTHY;
            _qualified = true;

            // FIX(M-1): latch _ever_initialised inside _sem. It is read by
            // make_pair_locked() on the CAN thread, so the write must not race
            // with that read (it was previously written lock-free here).
            if (!_ever_initialised) {
                _ever_initialised = true;
                first_initialisation = true;
            }
        }
    }

    if (!_health_announced && is_healthy) {
        _health_announced = true;
        _last_announced_health = true;

        GCS_SEND_TEXT(
            MAV_SEVERITY_INFO,
            first_initialisation ?
                "MTCAN READY: IMU0 qualified" :
                "MTCAN RECOVERED: IMU0 valid");
    } else if (_health_announced &&
               is_healthy != _last_announced_health) {
        _last_announced_health = is_healthy;

        if (is_healthy) {
            GCS_SEND_TEXT(
                MAV_SEVERITY_INFO,
                "MTCAN RECOVERED: IMU0 valid");
        } else {
            GCS_SEND_TEXT(
                MAV_SEVERITY_WARNING,
                "MTCAN LOST: EKF fallback pending");
        }
    }

    monitor_primary_imu(now_ms);
    send_telemetry(now_ms);
    send_periodic_status(now_ms);
}

bool AP_ExternalAHRS_MTCAN::healthy() const
{
    const uint64_t now_us = AP_HAL::micros64();
    const bool can_ready = CANSensor::initialized();

    WITH_SEMAPHORE(_sem);

    return stream_healthy_locked(
        now_us,
        can_ready);
}

bool AP_ExternalAHRS_MTCAN::initialised() const
{
    WITH_SEMAPHORE(_sem);
    return _ever_initialised;
}

bool AP_ExternalAHRS_MTCAN::pre_arm_check(
    char *failure_msg,
    const uint8_t failure_msg_len) const
{
    if (get_rate() != EXPECTED_RATE_HZ) {
        hal.util->snprintf(
            failure_msg,
            failure_msg_len,
            "MTCAN: EAHRS_RATE must be 400");
        return false;
    }

    if (!CANSensor::initialized()) {
        hal.util->snprintf(
            failure_msg,
            failure_msg_len,
            "MTCAN: CAN driver not ready");
        return false;
    }

    uint64_t last_pair_us;
    bool rate_window_valid;
    bool last_rate_window_good;
    bool rate_fault;
    bool frozen_fault;
    bool qualified;

    {
        WITH_SEMAPHORE(_sem);

        last_pair_us = _last_pair_us;
        rate_window_valid = _rate_window_valid;
        last_rate_window_good = _last_rate_window_good;
        rate_fault = _rate_fault;
        frozen_fault = _frozen_fault;
        qualified = _qualified;
    }

    if (last_pair_us == 0U) {
        hal.util->snprintf(
            failure_msg,
            failure_msg_len,
            "MTCAN: no paired gyro/accel");
        return false;
    }

    if (!rate_window_valid) {
        hal.util->snprintf(
            failure_msg,
            failure_msg_len,
            "MTCAN: qualifying 400Hz stream");
        return false;
    }

    if (!last_rate_window_good) {
        hal.util->snprintf(
            failure_msg,
            failure_msg_len,
            "MTCAN: latest rate window invalid");
        return false;
    }

    if (rate_fault) {
        hal.util->snprintf(
            failure_msg,
            failure_msg_len,
            "MTCAN: rate outside 360-440Hz");
        return false;
    }

    if (frozen_fault) {   // FIX(H-1)
        hal.util->snprintf(
            failure_msg,
            failure_msg_len,
            "MTCAN: sensor data frozen");
        return false;
    }

    if (!qualified || !healthy()) {
        hal.util->snprintf(
            failure_msg,
            failure_msg_len,
            "MTCAN: stream not healthy");
        return false;
    }

    return true;
}

void AP_ExternalAHRS_MTCAN::get_filter_status(
    nav_filter_status &status) const
{
    memset(&status, 0, sizeof(status));

    // This backend provides IMU data only; it does not provide an external
    // attitude, position, velocity, or heading solution.
    status.flags.initalized = healthy();
}

#endif // HAL_EXTERNAL_AHRS_ENABLED
