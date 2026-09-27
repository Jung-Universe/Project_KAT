#include "Copter.h"

#if MODE_TDCN_ENABLED

#include "mode_tdcn_claw_extern.h"

extern "C" {
volatile uint8_t Arming = 0;
}

// ---------------------------------------------------------------------------
/* 모드 진입시 1회 실행 */
bool ModeTDCN::init(bool ignore_checks)
{
    _state = State::NONE;
    _gcs_cmd.pending = false;   // 모드 진입 전에 도착해 있던 명령은 버린다
    _gcs_cmd.auto_pending = false;
    _auto_step = State::NONE;
    _auto_waiting = false;

    _target_loc = copter.current_loc;
    _target_heading_deg = degrees(ahrs.get_yaw());         

    home_init = false;

    _prearm_ready = false;

    _state_entered = false;
    _state_start_ms = AP_HAL::millis();

    _state_done = true;
    _action_retry_ms = 0;
    _was_armed = motors->armed();
    _air_hold_valid = false;

    _armed_ms = AP_HAL::millis();
    _armed_prev = motors->armed();
    _takeoff_started = false;

    _status = TdcnStatus{};

    Arming = 1;

    return true;
}
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
/* 모드 이탈 */
void ModeTDCN::exit()
{
    Arming = 0;
}
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
/* 메인 루프 (400Hz) */
void ModeTDCN::run()
{
    // 1. 기체 상태 점검
    check_vehicle_status();

    // 2. GCS 메시지 점검
    check_gcs_message();     

    // 3. state 별 처리
    switch (_state) 
    {
        case State::NONE:             preflight_vehicle_handling(); break;

        case State::HANGAR_OPEN:      state_hangar_open();          break;      // 1  격납함 열기

        case State::TAKEOFF_WAIT:     state_takeoff_wait();         break;      // 2  이륙 대기

        case State::ARMED:            state_armed();                break;      // 3  ARMED

        case State::LAUNCH:           state_launch();               break;      // 4  이륙 사출

        case State::FLIGHT_WAIT:      state_flight_wait();          break;      // 5  비행 대기

        case State::TRACKING:         state_tracking();             break;      // 6  추종 비행

        case State::LANDING_WAIT:     state_landing_wait();         break;      // 7  착륙 대기

        case State::LANDING_SYNC:     state_landing_sync();         break;      // 8  착륙 동기

        case State::LANDING_STOW:     state_landing_stow();         break;      // 9  착륙 수납

        case State::DISARMED:         state_disarmed();             break;      // 10 DISARMED

        case State::HANGAR_CLOSE:     state_hangar_close();         break;      // 11 격납함 닫기

        case State::AUTO_TO_TRACKING: state_auto_to_tracking();     break;      // 12 1~6 자동 진행

        case State::AUTO_TO_CLOSE:    state_auto_to_close();        break;      // 13 6~11 자동 진행
    }

    // 4. 현재 상태 업데이트
    update_status();

    _state_entered = false;
}
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
/* 기체 상태 점검.  run() 맨 앞에서 state 와 무관하게 매 루프 부른다 */
void ModeTDCN::check_vehicle_status()
{
    // 무장 시각 추적.  무장은 state 3 / state 4 재무장 / 자동 해제 후 재무장 등
    // 여러 경로로 걸리므로 여기서 한 번에 잡는다.  state 4 의 이륙 정착 대기가 쓴다.
    const bool armed_now = motors->armed();
    if (armed_now && !_armed_prev) {
        _armed_ms = AP_HAL::millis();
    }
    _armed_prev = armed_now;
}
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
/* GCS 명령 처리 */
MAV_RESULT ModeTDCN::GCS_command(const mavlink_command_int_t &packet)
{
    // GCS_Mavlink.cpp 는 현재 모드와 관계없이 이 함수를 부른다.  TDCN 이 아닐 때
    // 받아 두면 GCS 는 시나리오가 진행되는 줄 알게 되므로 거부한다.
    // (재시도해도 소용없으므로 DENIED.  조작자가 모드를 TDCN 으로 되돌려야 한다)
    if (copter.flightmode != this) {
        return MAV_RESULT_DENIED;
    }

    if (!isfinite(packet.param1)) {
        return MAV_RESULT_DENIED;
    }
    const int32_t state_num = (int32_t)roundf(packet.param1);

    // 아직 반영 안 된 명령이 있으면 그 state 가 곧 현재 state 가 된다.
    // 순서 / 완료 검사를 그것 기준으로 해야 수신 즉시 반영하던 때와 결과가 같다.
    // 자동 진행 중이면 그 안의 단계가 현재 state 다.
    const State now_state = scenario_state();
    const State cur_state = _gcs_cmd.pending ? _gcs_cmd.state : now_state;
    const bool  cur_done  = (cur_state == now_state) ? _state_done : false;

    // --- 자동 진행 명령 (12 / 13) ---
    //   12  state 0~5 에서만 받는다.  6 (추종 비행) 까지 자동으로 진행한다
    //   13  state 6~10 에서만 받는다.  11 (격납함 닫기) 까지 자동으로 진행한다
    // 현재 단계가 끝나지 않았어도 받는다 - 기체가 완료를 기다렸다가 넘어간다.
    if (state_num == (int32_t)State::AUTO_TO_TRACKING ||
        state_num == (int32_t)State::AUTO_TO_CLOSE) {
        const State auto_state = (State)state_num;
        const bool ok = (auto_state == State::AUTO_TO_TRACKING)
                        ? (cur_state < State::TRACKING)
                        : (cur_state >= State::TRACKING && cur_state < State::HANGAR_CLOSE);
        if (!ok) {
            return MAV_RESULT_DENIED;
        }
        _gcs_cmd.auto_state = auto_state;
        _gcs_cmd.auto_pending = true;
        return MAV_RESULT_ACCEPTED;
    }

    if (state_num < (int32_t)State::HANGAR_OPEN ||
        state_num > (int32_t)State::HANGAR_CLOSE) {
        return MAV_RESULT_DENIED;
    }

    const State state = (State)state_num;

    // DENIED = 입력 거부.  기체는 하던 일을 계속하고, 조작자가 다시 입력해야 한다.
    // 현재 단계가 끝나기 전의 다음 번호도 거부한다 (GCS 자동 재시도 없이 입력한
    // 대로 움직였는지 확인하기 위해 TEMPORARILY_REJECTED 를 쓰지 않는다).
    if (!state_order_ok(cur_state, state)) {
        return MAV_RESULT_DENIED;
    }
    if (state != cur_state && !cur_done) {
        return MAV_RESULT_DENIED;
    }

    Location loc;
    if (state == State::TRACKING) {
        if (!isfinite(packet.z) || !isfinite(packet.param2)) {
            return MAV_RESULT_DENIED;
        }

        switch (packet.frame) {

        case MAV_FRAME_GLOBAL_RELATIVE_ALT:
            loc.lat = packet.x;
            loc.lng = packet.y;
            break;

        case MAV_FRAME_LOCAL_NED:
            if (!ahrs.home_is_set()) {
                return MAV_RESULT_DENIED;   
            }
            loc = ahrs.get_home();
            loc.offset(packet.x * 0.01,         
                       packet.y * 0.01);       
            break;

        default:
            return MAV_RESULT_DENIED;
        }

        if (!ahrs.home_is_set()) {
            return MAV_RESULT_DENIED;
        }

        loc.set_alt_cm((int32_t)(packet.z * 100.0f),
                       Location::AltFrame::ABOVE_HOME);
    }

    // 검사 통과 - 보관만 한다.  반영은 run() 의 check_gcs_message() 가 한다.
    _gcs_cmd.state = state;
    if (state == State::TRACKING) {
        _gcs_cmd.target_loc = loc;
        _gcs_cmd.target_heading_deg = packet.param2;
    }
    _gcs_cmd.pending = true;

    return MAV_RESULT_ACCEPTED;
}
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
/* GCS 메시지 점검.  GCS_command() 가 보관한 명령을 반영한다 */
void ModeTDCN::check_gcs_message()
{
    // --- GCS 가 보낸 state 명령 (1~11) ---
    if (_gcs_cmd.pending) {
        _gcs_cmd.pending = false;
        const State next = _gcs_cmd.state;

        if (next == State::TRACKING) {
            _target_loc = _gcs_cmd.target_loc;
            _target_heading_deg = _gcs_cmd.target_heading_deg;
        }

        if (is_auto(_state) && next == _auto_step) {
            // 자동 진행 중 지금 단계와 같은 번호 - 자동 진행을 계속한다
            // (state 6 이면 위에서 타겟만 갱신했다)
        } else {
            if (is_auto(_state)) {
                // 자동 진행 중 다른 번호 - 자동 진행을 멈추고 수동으로 넘겨받는다
                _state = _auto_step;
                gcs().send_text(MAV_SEVERITY_INFO, "%s: auto stopped", name());
            }
            change_state(next);
        }
    }

    // --- GCS 가 보낸 자동 진행 명령 (12 / 13) ---
    // 지금 실행 중인 단계부터 이어서 진행한다 (그 단계의 진입 처리를 다시 하지
    // 않는다).  대기 시간은 명령을 받은 순간부터 잰다.
    if (_gcs_cmd.auto_pending) {
        _gcs_cmd.auto_pending = false;
        if (_state != _gcs_cmd.auto_state) {
            _auto_step    = scenario_state();
            _state        = _gcs_cmd.auto_state;
            _auto_waiting = false;
            gcs().send_text(MAV_SEVERITY_INFO, "%s: auto start -> state %u", name(),
                            (unsigned)(_state == State::AUTO_TO_TRACKING ? State::TRACKING
                                                                         : State::HANGAR_CLOSE));
        }
    }
}
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
/* state 전이.  진입 훅을 세우고, 새 state 는 아직 완료되지 않은 것으로 둔다 */
void ModeTDCN::change_state(State next)
{
    if (next == _state) {
        return;                 // 같은 번호 재전송 (state 6 타겟 갱신 등)
    }
    _state = next;
    _state_entered = true;
    _state_start_ms = AP_HAL::millis();

    // 담당 state_*() 가 자기 조건을 보고 true 로 올릴 때까지 다음으로 못 넘어간다
    _state_done = false;
}
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
/* 현재 상태 업데이트.  state 별 처리가 끝난 뒤 목표값 / 현재값 / 제어값을 저장한다 */
void ModeTDCN::update_status()
{
    // --- 목표값: 지금 state 가 쫓는 목표 ---
    // 지상 처리 (make_safe_ground_handling) 중이면 쫓는 목표가 없다.
    const bool flying = !is_disarmed_or_landed();
    bool valid = false;
    Vector3p target = _hold_pos_neu_cm;
    const State step = scenario_state();    // 자동 진행 중이면 그 안의 단계

    switch (step) {
    case State::NONE:
    case State::HANGAR_OPEN:
    case State::TAKEOFF_WAIT:
    case State::ARMED:
        valid = flying && _air_hold_valid;              // 공중이면 유지 위치
        break;
    case State::LAUNCH:
        valid = _takeoff_started;                       // 이륙 목표
        target = _takeoff_target_neu_cm;
        break;
    case State::FLIGHT_WAIT:
    case State::LANDING_WAIT:
    case State::LANDING_SYNC:                           // Z 는 state 8 이 동기 고도로 바꿔 둔다
    case State::DISARMED:                               // disarm 이 거부되어 공중일 때
        valid = flying;
        break;
    case State::TRACKING:
        valid = flying;
        target = _track_pos_neu_cm;                     // GCS 타겟
        break;
    case State::LANDING_STOW: {
        // 착륙 지점: 수평은 위치제어 목표, 고도는 home
        valid = flying;
        target = pos_control->get_pos_target_cm();
        Vector3f home_neu_cm;
        if (ahrs.get_home().get_vector_from_origin_NEU(home_neu_cm)) {
            target.z = home_neu_cm.z;
        }
        break;
    }
    case State::HANGAR_CLOSE:
    case State::AUTO_TO_TRACKING:                       // scenario_state() 는 이 둘을
    case State::AUTO_TO_CLOSE:                          // 돌려주지 않는다
        break;
    }

    _status.target_valid = valid;
    _status.target_pos_neu_cm = target;
    _status.target_heading_deg = (step == State::TRACKING)
                                 ? _target_heading_deg
                                 : degrees(attitude_control->get_att_target_euler_rad().z);

    // --- 현재값 ---
    _status.pos_neu_cm  = inertial_nav.get_position_neu_cm().topostype();
    _status.vel_neu_cms = inertial_nav.get_velocity_neu_cms();
    _status.euler_rad   = Vector3f(ahrs.get_roll(), ahrs.get_pitch(), ahrs.get_yaw());
    _status.gyro_rads   = ahrs.get_gyro();

    // --- 제어값: 5번에서 믹서에 넣을 CLAW 값과 대체 여부 ---
    _status.claw_active = claw_output_active();
    if (step == State::TRACKING) {
        _status.claw_roll  = constrain_float(CLAW_Y.v_cmd.cmd_roll,  -1.0f, 1.0f);
        _status.claw_pitch = constrain_float(CLAW_Y.v_cmd.cmd_pitch, -1.0f, 1.0f);
        _status.claw_yaw   = constrain_float(CLAW_Y.v_cmd.cmd_yaw,   -1.0f, 1.0f);

        // 범위만 맞춘다: -1 ~ +1  ->  0 ~ 1  (호버 보정 없음)
        const float ch = constrain_float(CLAW_Y.v_cmd.cmd_height, -1.0f, 1.0f);
        _status.claw_throttle = constrain_float((ch + 1.0f) * 0.5f, 0.0f, 1.0f);
    } else {
        // CLAW 는 state 6 에서만 돈다.  그 밖에는 CLAW_Y 가 낡은 값이라 남기지 않는다
        _status.claw_roll = _status.claw_pitch = _status.claw_yaw = 0.0f;
        _status.claw_throttle = 0.0f;
    }
}
// ---------------------------------------------------------------------------



bool ModeTDCN::state_order_ok(State from, State to)
{
    if (to == from) {
        return true;                                    // 같은 번호 재전송
    }
    return (uint8_t)to == (uint8_t)from + 1;            // 바로 다음 번호만
}

bool ModeTDCN::is_taking_off() const
{
    return (scenario_state() == State::LAUNCH) && !auto_takeoff.complete;
}

bool ModeTDCN::is_landing() const
{
    return scenario_state() == State::LANDING_STOW;
}

const char *ModeTDCN::state_name(State state)
{
    switch (state) {
    case State::NONE:           return "NONE";
    case State::HANGAR_OPEN:    return "HANGAR_OPEN";
    case State::TAKEOFF_WAIT:   return "TAKEOFF_WAIT";
    case State::ARMED:          return "ARMED";
    case State::LAUNCH:         return "LAUNCH";
    case State::FLIGHT_WAIT:    return "FLIGHT_WAIT";
    case State::TRACKING:       return "TRACKING";
    case State::LANDING_WAIT:   return "LANDING_WAIT";
    case State::LANDING_SYNC:   return "LANDING_SYNC";
    case State::LANDING_STOW:   return "LANDING_STOW";
    case State::DISARMED:       return "DISARMED";
    case State::HANGAR_CLOSE:   return "HANGAR_CLOSE";
    case State::AUTO_TO_TRACKING: return "AUTO_TO_TRACKING";
    case State::AUTO_TO_CLOSE:    return "AUTO_TO_CLOSE";
    }
    return "?";
}

// ---------------------------------------------------------------------------
/* CLAW 로 넘길 정보 갱신 */
void ModeTDCN::Update_Info_for_CLAW()
{
    // IMU - Gyro => CLAW: STV[3..5]
    const Vector3f &gyro = ahrs.get_gyro();
    CLAW_U.p = gyro.x;                           // roll  rate (rad/s, body F)
    CLAW_U.q = gyro.y;                           // pitch rate (rad/s, body R)
    CLAW_U.r = gyro.z;                           // yaw   rate (rad/s, body D)

    // IMU - Euler angle => CLAW: STV[6..8]
    CLAW_U.Roll                    = (real32_T)ahrs.get_roll();     // rad
    CLAW_U.Pitch                   = (real32_T)ahrs.get_pitch();    // rad
    CLAW_U.DR_heading_f.DR_heading =           ahrs.get_yaw();      // rad

    // EKF - Velocity => CLAW: XTV[0..2]
    const Vector3f &vel_neu_cms = inertial_nav.get_velocity_neu_cms();
    XTV[0] =  (double)vel_neu_cms.x * 0.01;       // North (m/s, N)
    XTV[1] =  (double)vel_neu_cms.y * 0.01;       // East  (m/s, E)
    XTV[2] = -(double)vel_neu_cms.z * 0.01;       // Down  (m/s, D)

    // Current Position (MAV_CMD_USER_1)
    const Location &loc = copter.current_loc;
    CLAW_U.Cur_Pos.x = (double)loc.lat * 1.0e-7;    // 위도 (deg)
    CLAW_U.Cur_Pos.y = (double)loc.lng * 1.0e-7;    // 경도 (deg)
    CLAW_U.Cur_Pos.z = (double)loc.alt * 0.01;      // 고도 (m, up)

    // Target Position (MAV_CMD_USER_1)
    CLAW_U.Dest_poti_i.x = (double)_target_loc.lat * 1.0e-7;   // 위도 (deg)
    CLAW_U.Dest_poti_i.y = (double)_target_loc.lng * 1.0e-7;   // 경도 (deg)
    // 고도는 Cur_Pos.z 와 같은 기준(ArduPilot home 기준 up m)으로 맞춘다.
    CLAW_U.Dest_poti_i.z = (double)_target_loc.alt * 0.01;     // 고도 (m, up)

    // Target Heading  (MAV_CMD_USER_1)
    CLAW_U.Ship_heading = radians(wrap_180(_target_heading_deg));   // (rad, 진북)
}
// ---------------------------------------------------------------------------


bool ModeTDCN::claw_output_active() const
{
    return _claw_on_off == 1
           && scenario_state() == State::TRACKING
           && home_init
           && motors->armed()
           && !is_disarmed_or_landed();
}

void ModeTDCN::output_to_motors()
{
    // 5. 제어값 실행
    //
    // ArduPilot 제어값은 이 시점에 확정된다 (바로 앞 run_rate_controller 의 출력).
    // 대체하면 사라지므로 먼저 떠 둔다.  롤/피치/요는 믹서가 쓰는 대로 PID +
    // 피드포워드, 스로틀은 ArduPilot 이 요구한 값 (angle boost 전) 이다.
    _status.ap_roll     = motors->get_roll()  + motors->get_roll_ff();
    _status.ap_pitch    = motors->get_pitch() + motors->get_pitch_ff();
    _status.ap_yaw      = motors->get_yaw()   + motors->get_yaw_ff();
    _status.ap_throttle = attitude_control->get_throttle_in();

    // 대체만 한다.  대체할지와 대체할 값은 4번 (update_status) 이 정해 두었다.
    if (_status.claw_active) {
        motors->set_roll(_status.claw_roll);
        motors->set_pitch(_status.claw_pitch);
        motors->set_yaw(_status.claw_yaw);
        motors->set_throttle(_status.claw_throttle);

        // 아두파일럿 각속도 PID 의 피드포워드가 더해지지 않게 지운다
        motors->set_roll_ff(0.0f);
        motors->set_pitch_ff(0.0f);
        motors->set_yaw_ff(0.0f);
    }

    Mode::output_to_motors();
}

void ModeTDCN::Run_CLAW()
{
    claw_gains.apply();       
    Update_Info_for_CLAW();

    // 요 궤적 초기화 (Xtraj[3] = 현재 헤딩) 는 CLAW.c 의 home 블록이 한다
    CLAW_step();                // CLAW 실행 (CLAW.c 의 함수 직접 호출)
}

void ModeTDCN::preflight_vehicle_handling()
{
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        _air_hold_valid = false;    // 다음에 공중에 뜨면 위치를 새로 잡는다
        return;
    }

    // 공중이다.  잡을 위치를 한 번만 정한다.
    if (!_air_hold_valid) {
        _air_hold_valid = true;

        pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(),
                                            wp_nav->get_wp_acceleration());
        pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(),
                                                    wp_nav->get_wp_acceleration());
        pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(),
                                            wp_nav->get_default_speed_up(),
                                            wp_nav->get_accel_z());
        pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(),
                                                    wp_nav->get_default_speed_up(),
                                                    wp_nav->get_accel_z());

        // 다른 모드에서 막 넘어왔을 수 있으므로 정지점 기준으로 초기화한다.
        pos_control->init_xy_controller_stopping_point();
        pos_control->init_z_controller_stopping_point();
        auto_yaw.set_mode(AutoYaw::Mode::HOLD);

        _hold_pos_neu_cm = pos_control->get_pos_desired_cm();
    }

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    pos_control->input_pos_xyz(_hold_pos_neu_cm, 0.0f, 0.0f);
    pos_control->update_xy_controller();
    pos_control->update_z_controller();
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(),
                                                  auto_yaw.get_heading());
}

void ModeTDCN::state_hangar_open()      // 1 격납함 열기
{
    // 기체가 할 일이 없다.  격납함 개폐는 기체 밖에서 처리된다.
    _state_done = true;         // 즉시 완료 - 2번으로 넘어가도 된다

    preflight_vehicle_handling();
}

void ModeTDCN::state_takeoff_wait()     // 2 이륙 대기 (prearm check)
{
    const bool ready = copter.ap.pre_arm_check;

    // 완료 판정: arm 이 가능해야 3번으로 넘어갈 수 있다
    _state_done = ready;

    if (_state_entered || ready != _prearm_ready) {
        _prearm_ready = ready;
        gcs().send_text(ready ? MAV_SEVERITY_INFO : MAV_SEVERITY_WARNING,
                        "%s: prearm %s", name(), ready ? "OK" : "FAIL");
    }

    preflight_vehicle_handling();
}

void ModeTDCN::state_armed()            // 3 ARMED
{
    // 완료 판정: 실제로 무장돼야 4번으로 넘어갈 수 있다
    _state_done = motors->armed();

    if (!_state_done) {
        const uint32_t now_ms = AP_HAL::millis();
        if (_state_entered || _was_armed || (now_ms - _action_retry_ms) >= 1000) {
            _action_retry_ms = now_ms;
            copter.arming.arm(AP_Arming::Method::MAVLINK);
        }
    }
    _was_armed = motors->armed();

    preflight_vehicle_handling();
}

static const uint32_t _takeoff_settle_ms = 2000;    // 2 초

void ModeTDCN::state_launch()           // 4 이륙 사출
{
    const uint32_t now_ms = AP_HAL::millis();

    if (_state_entered) {
        _takeoff_started = false;
    }

    if (!_state_done && !motors->armed()) {
        if (_state_entered || (now_ms - _action_retry_ms) >= 1000) {
            _action_retry_ms = now_ms;
            copter.arming.arm(AP_Arming::Method::MAVLINK);
        }
        if (!motors->armed()) {
            make_safe_ground_handling();
            return;                 // 아직 무장 못 함.  다음 루프에서 재시도
        }
        _takeoff_started = false;   // 재무장했으니 이륙을 처음부터 다시 시작
    }

    if (!_takeoff_started && is_disarmed_or_landed() &&
        (now_ms - _armed_ms) < _takeoff_settle_ms) {
        make_safe_ground_handling();
        return;
    }

    if (!_takeoff_started) {
        // --- 이륙 시작 (ModeGuided::do_user_takeoff_start() 와 같은 순서) ---

        // 이륙 중에는 heading 을 유지한다
        auto_yaw.set_mode(AutoYaw::Mode::HOLD);

        // 상승 속도 제한.  하강 속도도 같은 값으로 둔다 (이륙에는 쓰이지 않지만
        // 컨트롤러 한계를 비대칭으로 두지 않는다).
        pos_control->set_max_speed_accel_z(-_takeoff_spd, _takeoff_spd,
                                          g.pilot_accel_z);
        pos_control->set_correction_speed_accel_z(-_takeoff_spd, _takeoff_spd,
                                                  g.pilot_accel_z);

        // 수직 위치 컨트롤러 초기화 (I 항 클리어)
        pos_control->init_z_controller();

        // auto_takeoff 는 목표 고도를 EKF origin 기준 cm 로 받는다.
        // _takeoff_alt 은 home 기준이므로 변환한다.
        Location target_loc = copter.current_loc;
        target_loc.set_alt_cm((int32_t)_takeoff_alt, Location::AltFrame::ABOVE_HOME);
        int32_t alt_above_origin_cm;
        if (!target_loc.get_alt_cm(Location::AltFrame::ABOVE_ORIGIN,
                                   alt_above_origin_cm)) {
            gcs().send_text(MAV_SEVERITY_WARNING, "%s: takeoff alt failed", name());
            return;
        }
        auto_takeoff.start((float)alt_above_origin_cm, false);

        // update_status() 의 목표값 (이륙 목표).  XY 는 이륙 시작 위치다.
        _takeoff_target_neu_cm = inertial_nav.get_position_neu_cm().topostype();
        _takeoff_target_neu_cm.z = alt_above_origin_cm;

        // auto_takeoff.run() 은 ap.auto_armed 가 false 면 즉시 리턴한다.
        // auto_armed 는 조종기 스로틀을 올려야 켜지는데 (system.cpp 의
        // update_auto_armed), 이 시나리오에는 스틱이 없으므로 직접 세운다.
        // Copter::start_takeoff() 도 같은 방식이다.
        copter.set_auto_armed(true);

        // 이륙 시퀀스가 시작됐다.  이 뒤로는 정착 대기 조건을 보지 않는다.
        _takeoff_started = true;
        gcs().send_text(MAV_SEVERITY_INFO, "%s: takeoff to %.1fm", name(),
                        (double)(_takeoff_alt * 0.01f));
    }

    // 이륙 제어.  complete 가 true 가 된 뒤에도 계속 호출하면 목표 고도와
    // 제자리(XY 속도 0)를 유지하므로 그대로 호버링한다.
    auto_takeoff.run();

    // 완료 판정: 목표 고도에 도달해야 5번으로 넘어갈 수 있다
    _state_done = auto_takeoff.complete;
}

void ModeTDCN::state_flight_wait()      // 5 비행 대기
{
    // 완료 판정: 호버 유지 상태라 언제든 추종을 시작할 수 있다
    _state_done = true;

    if (_state_entered) {
        // --- 유지할 위치 확정 ---
        //
        // state 4 의 이륙 완료 위치를 그대로 이어받는다.  이륙이 완료되지 않은
        // 채로 넘어왔으면 completion pos 가 없으므로, 지금 컨트롤러가 추종 중인
        // 목표(get_pos_desired_cm)를 쓴다.  현재 위치가 아니라 "추종 중인 목표"
        // 를 쓰는 이유는 그래야 전환 순간에 목표가 튀지 않기 때문이다.
        if (!auto_takeoff.get_completion_pos(_hold_pos_neu_cm)) {
            _hold_pos_neu_cm = pos_control->get_pos_desired_cm();
        }

        // 속도 / 가속 한계.  ModeGuided::pva_control_start() 와 같은 값이다.
        // Z 는 state 4 에서 이륙 속도(느림)로 좁혀 두었으므로 여기서 기본값으로
        // 되돌려 고도 보정 여력을 회복시킨다.
        pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(),
                                           wp_nav->get_wp_acceleration());
        pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(),
                                                   wp_nav->get_wp_acceleration());
        pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(),
                                           wp_nav->get_default_speed_up(),
                                           wp_nav->get_accel_z());
        pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(),
                                                  wp_nav->get_default_speed_up(),
                                                  wp_nav->get_accel_z());

        // 이미 활성인 컨트롤러는 다시 초기화하지 않는다 (위 주석 1번).
        if (!pos_control->is_active_xy()) {
            pos_control->init_xy_controller();
        }
        if (!pos_control->is_active_z()) {
            pos_control->init_z_controller();
        }

        auto_yaw.set_mode(AutoYaw::Mode::HOLD);
    }

    // 무장 전이거나 착지 상태면 제어하지 않는다
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // 이륙한 위치와 고도를 계속 유지한다 (XY, Z 모두)
    pos_control->input_pos_xyz(_hold_pos_neu_cm, 0.0f, 0.0f);

    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(),
                                                  auto_yaw.get_heading());
}

void ModeTDCN::state_tracking()         // 6 추종 비행
{
    // 완료 판정: 추종을 언제 끝낼지는 조작자가 정한다
    _state_done = true;

    if (_state_entered) {
        // 속도 / 가속 한계 (ModeGuided::pva_control_start() 와 동일)
        pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(),
                                           wp_nav->get_wp_acceleration());
        pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(),
                                                   wp_nav->get_wp_acceleration());
        pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(),
                                           wp_nav->get_default_speed_up(),
                                           wp_nav->get_accel_z());
        pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(),
                                                  wp_nav->get_default_speed_up(),
                                                  wp_nav->get_accel_z());

        // 이미 활성인 컨트롤러는 재초기화하지 않는다 (state 5 에서 이어받는다)
        if (!pos_control->is_active_xy()) {
            pos_control->init_xy_controller();
        }
        if (!pos_control->is_active_z()) {
            pos_control->init_z_controller();
        }

        // 타겟 변환이 실패했을 때 쓸 안전 초기값.  0 으로 두면 EKF origin 으로
        // 날아가므로, 지금 추종 중인 목표를 넣어 제자리 유지가 되게 한다.
        _track_pos_neu_cm = pos_control->get_pos_desired_cm();
    }

    // --- CLAW 병렬 실행 -----------------------------------------------------
    // 기체 제어와 무관하게 매 루프 돌린다.  CLAW 가 받는 입력은 아두파일럿이
    // 실제로 쓰는 것과 같은 값이므로, 로그를 비교하면 CLAW 를 판정할 수 있다.
    //
    //   Update_Info_for_CLAW()  IMU, EKF 속도, 현재 위치, 타겟 위치, 타겟 heading
    //   CLAW_step()             CLAW 실행
    Run_CLAW();

    // --- 기체 제어: 아두파일럿 위치제어로 GCS 타겟 추종 ----------------------
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // GCS 타겟 -> EKF origin 기준 NEU cm.
    // _target_loc 이 int32 위경도 + AltFrame 을 그대로 들고 있으므로 여기서는
    // 변환만 한다 (부동소수 왕복이 없어 cm 정밀도가 유지된다).
    Vector3f target_neu_cm;
    if (_target_loc.get_vector_from_origin_NEU(target_neu_cm)) {
        _track_pos_neu_cm = target_neu_cm.topostype();
    }
    // 변환 실패(EKF origin 미설정)면 마지막 목표를 유지한다

    pos_control->input_pos_xyz(_track_pos_neu_cm, 0.0f, 0.0f);
    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    // 타겟 heading 을 향한다.  CLAW 도 ps_cmd = Ship_heading 을 추종하므로,
    // 같은 heading 조건이어야 CLAW 의 yaw 채널을 공정하게 비교할 수 있다.
    auto_yaw.set_yaw_angle_rate(_target_heading_deg, 0.0f);

    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(),
                                                  auto_yaw.get_heading());

    if (claw_output_active()) {
        attitude_control->reset_target_and_rate(false);
        attitude_control->reset_rate_controller_I_terms();
    }
}

void ModeTDCN::state_landing_wait()     // 7 착륙 대기
{
    // 완료 판정: 호버 유지 상태라 언제든 하강할 수 있다
    _state_done = true;

    if (_state_entered) {
        // 속도 / 가속 한계 (ModeGuided::pva_control_start() 와 동일)
        pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(),
                                           wp_nav->get_wp_acceleration());
        pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(),
                                                   wp_nav->get_wp_acceleration());
        pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(),
                                           wp_nav->get_default_speed_up(),
                                           wp_nav->get_accel_z());
        pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(),
                                                  wp_nav->get_default_speed_up(),
                                                  wp_nav->get_accel_z());

        // --- CLAW -> 아두파일럿 인수인계 ---
        //
        // 조건 없이 다시 초기화한다.  CLAW 가 몰던 동안 pos_control 의 목표는
        // 낡아 있으므로 그것을 이어받으면 안 된다.  현재 위치 / 속도에서
        // 계산한 정지점을 새 목표로 삼는다.
        pos_control->init_xy_controller_stopping_point();
        pos_control->init_z_controller_stopping_point();

        // CLAW 가 만든 자세를 쫓느라 쌓인 적분항을 비우고, yaw 목표를 현재
        // 자세로 맞춘다.  이걸 안 하면 인계 직후 자세가 요동친다.
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate();

        auto_yaw.set_mode(AutoYaw::Mode::HOLD);

        // 위에서 계산된 정지점을 유지 목표로 잡는다
        _hold_pos_neu_cm = pos_control->get_pos_desired_cm();
    }

    // --- 기체 제어: 정지점 유지 (CLAW 는 여기서 돌지 않는다) ---
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    pos_control->input_pos_xyz(_hold_pos_neu_cm, 0.0f, 0.0f);

    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(),
                                                  auto_yaw.get_heading());
}

void ModeTDCN::state_landing_sync()     // 8 착륙 동기
{
    if (_state_entered) {
        // XY 는 지금 추종 중인 목표를 그대로 이어받는다 (그 자리 유지).
        // Z 는 아래에서 매 루프 목표 고도로 덮어쓴다.
        _hold_pos_neu_cm = pos_control->get_pos_desired_cm();

        // XY 한계는 기본값, Z 하강 속도만 따로 준다.
        // 하강 속도를 제한하는 이유는 착륙 준비 단계에서 급강하를 막기 위함이다.
        pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(),
                                           wp_nav->get_wp_acceleration());
        pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(),
                                                   wp_nav->get_wp_acceleration());
        pos_control->set_max_speed_accel_z(-_land_spd,
                                           wp_nav->get_default_speed_up(),
                                           wp_nav->get_accel_z());
        pos_control->set_correction_speed_accel_z(-_land_spd,
                                                  wp_nav->get_default_speed_up(),
                                                  wp_nav->get_accel_z());

        // 이미 활성인 컨트롤러는 재초기화하지 않는다.  state 7 이 정지점을
        // 잡아둔 상태로 들어오므로 그것을 이어받아야 목표가 튀지 않는다.
        if (!pos_control->is_active_xy()) {
            pos_control->init_xy_controller();
        }
        if (!pos_control->is_active_z()) {
            pos_control->init_z_controller();
        }

        auto_yaw.set_mode(AutoYaw::Mode::HOLD);
    }

    Location sync_loc = copter.current_loc;
    sync_loc.set_alt_cm((int32_t)_land_alt, Location::AltFrame::ABOVE_HOME);
    Vector3f sync_neu_cm;
    if (sync_loc.get_vector_from_origin_NEU(sync_neu_cm)) {
        _hold_pos_neu_cm.z = sync_neu_cm.z;     // XY 는 건드리지 않는다
    }

    // 완료 판정: 착륙 준비 고도에 도달해야 9번으로 넘어갈 수 있다
    _state_done = fabsf((float)copter.current_loc.alt - _land_alt) < 50.0f;

    // --- 기체 제어 ---
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    pos_control->input_pos_xyz(_hold_pos_neu_cm, 0.0f, 0.0f);

    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(),
                                                  auto_yaw.get_heading());
}

void ModeTDCN::state_landing_stow()     // 9 착륙 수납
{
    // 완료 판정: 착지 감지가 떠야 10번(disarm)으로 넘어갈 수 있다.
    // 이게 없으면 공중에서 disarm 을 시도하게 된다.
    _state_done = copter.ap.land_complete;

    if (_state_entered) {
        // ModeLand::init() 과 같은 순서 / 같은 값
        pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(),
                                           wp_nav->get_wp_acceleration());
        pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(),
                                                   wp_nav->get_wp_acceleration());

        if (!pos_control->is_active_xy()) {
            pos_control->init_xy_controller();
        }

        pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(),
                                           wp_nav->get_default_speed_up(),
                                           wp_nav->get_accel_z());
        pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(),
                                                  wp_nav->get_default_speed_up(),
                                                  wp_nav->get_accel_z());

        if (!pos_control->is_active_z()) {
            pos_control->init_z_controller();
        }

        // 조종자 재위치 / 정밀착륙 플래그 초기화 (ModeLand::init() 과 동일)
        copter.ap.land_repo_active = false;
        copter.ap.prec_land_active = false;

        auto_yaw.set_mode(AutoYaw::Mode::HOLD);
    }

    // 착지했고 모터가 ground idle 이면 더 내려갈 곳이 없다.
    // disarm 은 state 10 이 담당하므로 여기서는 안전 처리만 한다.
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        pos_control->relax_z_controller(0.0f);
        return;
    }

    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    land_run_horiz_and_vert_control();
}

void ModeTDCN::state_disarmed()         // 10 DISARMED
{
    // 완료 판정: 실제로 무장이 풀려야 11번으로 넘어갈 수 있다
    _state_done = !motors->armed();

    // 무장 해제 상태를 "유지" 한다.  아직 무장돼 있으면 (착륙이 끝나지 않아
    // disarm 이 거부됐거나, 어떤 이유로 다시 무장됐다면) 계속 시도한다.
    // disarm() 은 체크를 돌리고 결과를 GCS 에 출력하므로 400Hz 로 부를 수 없다.
    if (!_state_done) {
        const uint32_t now_ms = AP_HAL::millis();
        if (_state_entered || (now_ms - _action_retry_ms) >= 1000) {
            _action_retry_ms = now_ms;
            copter.arming.disarm(AP_Arming::Method::MAVLINK);
        }
    }

    if (_state_entered) {
        // disarm 이 거부될 경우(아직 비행 중)를 대비해 호버 목표를 잡아둔다
        _hold_pos_neu_cm = pos_control->get_pos_desired_cm();

        pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(),
                                           wp_nav->get_wp_acceleration());
        pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(),
                                                   wp_nav->get_wp_acceleration());
        pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(),
                                           wp_nav->get_default_speed_up(),
                                           wp_nav->get_accel_z());
        pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(),
                                                  wp_nav->get_default_speed_up(),
                                                  wp_nav->get_accel_z());

        if (!pos_control->is_active_xy()) {
            pos_control->init_xy_controller();
        }
        if (!pos_control->is_active_z()) {
            pos_control->init_z_controller();
        }

        auto_yaw.set_mode(AutoYaw::Mode::HOLD);
    }

    if (is_disarmed_or_landed()) {
        // 정상 경로 - 지상에서 모터를 내리고 적분항 / yaw 목표를 리셋한다
        make_safe_ground_handling();
        return;
    }

    // 아직 비행 중이다 = disarm 이 거부되었다는 뜻이다.
    // 여기서 지상 처리를 하면 추락하므로, 제자리 호버를 유지한다.
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    pos_control->input_pos_xyz(_hold_pos_neu_cm, 0.0f, 0.0f);

    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(),
                                                  auto_yaw.get_heading());
}

void ModeTDCN::state_hangar_close()     // 11 격납함 닫기
{
    // 마지막 단계 - 뒤가 없으므로 완료로 둔다
    _state_done = true;

    // 지상 대기.  혹시 무장 상태로 들어와도 모터를 올리지 않는다.
    make_safe_ground_handling();
}

void ModeTDCN::state_auto_to_tracking() // 12 1~6 자동 진행
{
    // 다음 단계로 넘어갈지 먼저 정한다.  넘어가면 이번 루프에 그 단계의 state
    // 함수가 진입 처리를 한다 (_state_entered 는 run() 끝에서 내려간다).
    auto_advance(State::TRACKING);

    // 지금 단계의 state 함수를 그대로 부른다
    switch (_auto_step) {
    case State::NONE:           preflight_vehicle_handling(); break;
    case State::HANGAR_OPEN:    state_hangar_open();    break;  // 1
    case State::TAKEOFF_WAIT:   state_takeoff_wait();   break;  // 2
    case State::ARMED:          state_armed();          break;  // 3
    case State::LAUNCH:         state_launch();         break;  // 4
    case State::FLIGHT_WAIT:    state_flight_wait();    break;  // 5
    case State::TRACKING:       state_tracking();       break;  // 6 (도착한 루프)
    default:                                            break;
    }
}

void ModeTDCN::state_auto_to_close()    // 13 6~11 자동 진행
{
    auto_advance(State::HANGAR_CLOSE);

    switch (_auto_step) {
    case State::TRACKING:       state_tracking();       break;  // 6
    case State::LANDING_WAIT:   state_landing_wait();   break;  // 7
    case State::LANDING_SYNC:   state_landing_sync();   break;  // 8
    case State::LANDING_STOW:   state_landing_stow();   break;  // 9
    case State::DISARMED:       state_disarmed();       break;  // 10
    case State::HANGAR_CLOSE:   state_hangar_close();   break;  // 11 (도착한 루프)
    default:                                            break;
    }
}

// ---------------------------------------------------------------------------
/* 자동 진행 공통.  지금 단계가 TDCN_AUTO_DWELL 동안 계속 완료 상태면 다음 단계로 */
void ModeTDCN::auto_advance(State end)
{
    // 완료되면 곧바로 넘기지 않는다.  완료 상태가 대기 시간 동안 계속 유지돼야
    // 넘긴다 (예: 무장 직후 곧바로 이륙하지 않는다).  중간에 완료가 풀리면
    // (예: 고도가 흔들림) 처음부터 다시 잰다.
    // _state_done 은 직전 루프에 그 단계의 state 함수가 판정한 값이다.
    // state 0 (NONE) 은 할 일이 없어 늘 완료로 본다.
    const bool done = (_auto_step == State::NONE) || _state_done;
    if (!done) {
        _auto_waiting = false;
        return;
    }
    const uint32_t now_ms = AP_HAL::millis();
    if (!_auto_waiting) {
        _auto_waiting = true;
        _auto_done_ms = now_ms;
        return;
    }
    const uint32_t dwell_ms = (uint32_t)(MAX(_auto_dwell.get(), 0.0f) * 1000.0f);
    if (now_ms - _auto_done_ms < dwell_ms) {
        return;
    }

    // --- 다음 단계로 ---
    const State next = (State)((uint8_t)_auto_step + 1);
    if (next == State::TRACKING) {
        // 12 의 끝: 초기 목표 = 현재 상태 (위치 / 고도 / 헤딩).  제자리에서
        // 추종을 시작하고, 이후 GCS 가 state 6 타겟을 보내면 그것을 따라간다.
        _target_loc = copter.current_loc;
        _target_heading_deg = degrees(ahrs.get_yaw());
    }
    _auto_step     = next;
    _auto_waiting  = false;
    _state_entered = true;
    _state_start_ms = now_ms;
    _state_done    = false;
    gcs().send_text(MAV_SEVERITY_INFO, "%s: auto state %u", name(), (unsigned)next);

    if (next == end) {
        // 끝 단계 도착 - 자동 진행을 끝내고, 다음 루프부터 그 state 로 일반 처리한다
        _state = next;
        gcs().send_text(MAV_SEVERITY_INFO, "%s: auto done (state %u)", name(), (unsigned)next);
    }
}
// ---------------------------------------------------------------------------


#endif  // MODE_TDCN_ENABLED