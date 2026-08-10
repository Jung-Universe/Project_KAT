/* Sejong */
//
// TDCN mode - CLAW 제어기 통합
//
// [Version 1] CLAW 는 항상 돌린다 (모니터링 목적).
//             CLAW 의 최종 출력(CLAW_Y.v_cmd)을 아두파일럿 제어기 대신 쓸지
//             말지는 state 6 에서 결정하며, v1 에서는 적용하지 않는다.
//
// run() 구성
//   1. GCS 명령 파싱   GCS_command() 가 MAVLink 수신 시점에 이미 끝내둔다
//   2. state 처리      case 문으로 state 1~11 별 함수 호출
//
// CLAW 는 Run_CLAW() 한 함수로 묶여 있고, 그것을 필요한 state_*() 에서 호출한다
//   Run_CLAW() = Update_Info_for_CLAW() -> CLAW_step() -> Log_Write_TDCN()
//
// CLAW 의 home 은 TDCN 이 관여하지 않는다.  CLAW 가 CLAW_step() 안에서 스스로
// 캡처한다 (init() 이 home_init = false 로 만들어 두면 재진입마다 다시 잡는다).
//
// GCS_command() 는 run() 이 아니라 MAVLink 수신 경로에서 호출된다.
// update_receive 가 메인 스레드 스케줄러 태스크라 run() 과 같은 스레드이므로,
// 수신값을 보관해 두지 않고 도착 즉시 파싱한다.
//
// 관련 파일
//   mode.h                                 ModeTDCN 클래스 선언
//   GCS_Mavlink.cpp                        MAV_CMD_USER_1 수신 -> GCS_command()
//   mode_tdcn_CLAW_IBSC_ship_Fianl_NED.c   CLAW 제어기 본체
//

#include "Copter.h"

#if MODE_TDCN_ENABLED

// ---------------------------------------------------------------------------
// CLAW 연동
//
// CLAW 는 순수 C 코드이고 헤더에 extern "C" 가드가 없다.  vendor 파일을 고치지
// 않기 위해 include 를 여기서 감싼다.
// ---------------------------------------------------------------------------

extern "C" {
#include "mode_tdcn_CLAW.h"

// CLAW_step() 이 참조하는 arming 플래그.  정의는 우리 쪽 책임이다.
// CLAW 는 이 값이 0 이면 즉시 return 하며 출력을 0 으로 만든다.  모니터링을
// 위해 TDCN 모드에 있는 동안은 1 로 둬서 CLAW 가 계속 돌게 한다.
volatile uint8_t Arming = 0;

// CLAW.c 의 home 래치 플래그.  vendor 파일에서 static 을 제거해 외부에서
// 리셋할 수 있게 했다.  false 로 만들면 CLAW 가 다음 스텝에서 home 을 다시
// 캡처하고, 적분기 / 궤적 / 가속도 추정기 상태까지 전부 초기화한다.
extern bool home_init;

// CLAW 의 관성좌표계 상태 벡터 [vx,vy,vz, wx,wy,wz].  CLAW.h 에 선언이 없다.
// XTV[0..2] 는 TDCN 이 공급하는 EKF 속도이고, XTV[3..5] 는 CLAW 가 각속도에서
// 직접 계산한다.
extern double XTV[6];
}

// Home_Lat / Home_Lon / Home_Alt / Home_Yaw 와 STV[12] 는 mode_tdcn_CLAW.h
// 가 이미 extern 선언해 두었으므로 여기서 다시 선언하지 않는다.
// (CLAW 내부의 home_init 은 file-scope static 이라 외부에서 건드릴 수 없다)

// ---------------------------------------------------------------------------
// 모드 진입
// ---------------------------------------------------------------------------

bool ModeTDCN::init(bool ignore_checks)
{
    // GCS 명령을 받기 전까지는 아무 state 도 아니다.
    // 모드 진입 전에 도착해 있던 명령은 이 대입으로 버려진다.
    _state = State::NONE;

    // 타겟 초기값 = 현재 위치 / 현재 헤딩.
    //
    // 0 으로 두면 안 된다.  Dest_poti_i 에 수신값을 그대로 넣는 구조이므로
    // 위도 0 은 "적도" 를 뜻하고, home 이 래치된 뒤 타겟 명령이 오기 전까지
    // 수천 km 짜리 위치 오차 (leash 25m 로 clamp) 와 진북 heading 명령이
    // 걸린다.  현재 위치로 두면 "제자리 유지" 가 된다.
    _target_loc = copter.current_loc;                      // 위경도 + 고도(AltFrame)
    _target_heading_deg = degrees(ahrs.get_yaw());         // rad -> deg, 진북

    // 모드에 들어올 때마다 home 을 다시 잡는다.  CLAW 는 다음 스텝의
    // home_init 블록에서 현재 위치를 home 으로 캡처하고 내부 상태
    // (TV_BSC, Xtraj, 가속도 추정기 버퍼, UD_DSTATE) 를 모두 리셋한다.
    home_init = false;
    _log_counter = 0;

    // state 2 진입 시 무조건 한 번 보고하도록 초기값을 false 로 둔다
    _prearm_ready = false;

    // state 진입 훅.  NONE 은 아무 동작도 없으므로 entry 를 걸지 않는다.
    _state_entered = false;
    _state_start_ms = AP_HAL::millis();

    // NONE 은 할 일이 없으므로 완료 상태로 둔다 (1번은 언제든 받는다)
    _state_done = true;
    _action_retry_ms = 0;
    _was_armed = motors->armed();
    _air_hold_valid = false;

    // CLAW 상시 실행 - 모니터링을 위해 모드에 있는 동안은 계속 돌린다
    Arming = 1;

    return true;
}

// ---------------------------------------------------------------------------
// 모드 이탈
// ---------------------------------------------------------------------------

void ModeTDCN::exit()
{
    // CLAW 를 정지시킨다.  CLAW 는 Arming == 0 에서 출력을 0 으로 만들고
    // 가속도 추정기 버퍼를 리셋한다.
    Arming = 0;
}

// ---------------------------------------------------------------------------
// 메인 루프 (400Hz)
// ---------------------------------------------------------------------------

void ModeTDCN::run()
{
    // (완료) 1단계 - GCS command 파싱

    // 2단계 - State 처리
    switch (_state) {

    // 0  GCS 명령 대기.  기체는 지상이면 안전 처리, 공중이면 제자리 유지.
    case State::NONE:           preflight_vehicle_handling(); break;

    case State::HANGAR_OPEN:    state_hangar_open();    break;  // 1  격납함 열기

    case State::TAKEOFF_WAIT:   state_takeoff_wait();   break;  // 2  이륙 대기

    case State::ARMED:          state_armed();          break;  // 3  ARMED

    case State::LAUNCH:         state_launch();         break;  // 4  이륙 사출

    case State::FLIGHT_WAIT:    state_flight_wait();    break;  // 5  비행 대기

    case State::TRACKING:       state_tracking();       break;  // 6  추종 비행

    case State::LANDING_WAIT:   state_landing_wait();   break;  // 7  착륙 대기

    case State::LANDING_SYNC:   state_landing_sync();   break;  // 8  착륙 동기

    case State::LANDING_STOW:   state_landing_stow();   break;  // 9  착륙 수납

    case State::DISARMED:       state_disarmed();       break;  // 10 DISARMED

    case State::HANGAR_CLOSE:   state_hangar_close();   break;  // 11 격납함 닫기
     
    }

    // 진입 훅 소비.  위 state_*() 들이 읽은 뒤 여기서 내린다.
    _state_entered = false;
}

// ---------------------------------------------------------------------------
// MAV_CMD_USER_1 (31010) 수신 + 파싱
//
// [반드시 COMMAND_INT 로 보내야 한다]
//   param1    state              (1~11)
//   param2    Target Heading     (deg, 진북)
//   param3    예약 (0)
//   param4    예약 (0)
//
// 타겟 좌표(x/y/z)의 의미는 frame 이 선언한다.  두 가지를 받는다:
//
//   frame = 3 (MAV_FRAME_GLOBAL_RELATIVE_ALT)   [최종 결과물용]
//     x   Target Latitude    (int32, 1e7 deg)   <- 정밀도 약 1.1 cm
//     y   Target Longitude   (int32, 1e7 deg)   <- 정밀도 약 1.1 cm
//     z   Target Altitude    (float, m, home 기준 up)
//
//   frame = 1 (MAV_FRAME_LOCAL_NED)             [SITL 테스트용]
//     x   Target North       (int32, cm, home 기준)
//     y   Target East        (int32, cm, home 기준)
//     z   Target Altitude    (float, m, home 기준 up)
//     -> 받는 즉시 위경도로 바꿔 _target_loc 에 넣는다.  그래서 이 아래
//        (state 6, CLAW 전달) 는 두 경우를 구분할 필요가 없다.
//
// 테스트가 끝나면 GCS 가 frame 만 3 으로 바꾸면 된다 - 기체 코드는 그대로다.
//
// 타겟 정보(x/y/z/param2)는 state 6 에서만 유효하다.
//
// COMMAND_LONG 으로 보내면 위경도가 망가진다.  ArduPilot 이 COMMAND_LONG 을
// COMMAND_INT 로 변환할 때 x/y 는 param5/param6 에서 오는데,
// convert_COMMAND_LONG_loc_param() 이 MAV_CMD_USER_1 을 "위치를 담는 명령"
// 목록(command_long_stores_location())에 넣어두지 않아 1e7 배를 곱하지 않고
// 그냥 int 로 절단한다 (37.5 -> 37, 즉 정수 도 단위).
//
// 값의 유효성은 GCS 가 보장한다.  여기서는 상태와 목표값으로 파싱만 한다.
// (isfinite / frame 검사는 값 검증이 아니라 캐스트·기준계 보호다)
//
// update_receive 는 메인 스레드 스케줄러 태스크이므로 run() 과 같은 스레드에서
// 돈다.  그래서 수신값을 따로 보관해 두지 않고 여기서 바로 파싱한다.
// ---------------------------------------------------------------------------

MAV_RESULT ModeTDCN::GCS_command(const mavlink_command_int_t &packet)
{
    if (!isfinite(packet.param1)) {
        return MAV_RESULT_DENIED;
    }
    const int32_t state_num = (int32_t)roundf(packet.param1);
    if (state_num < (int32_t)State::HANGAR_OPEN ||
        state_num > (int32_t)State::HANGAR_CLOSE) {
        return MAV_RESULT_DENIED;
    }

    const State state = (State)state_num;

    // --- state 순서 가드 ---
    //
    // GCS 는 1 -> 11 을 차례로 보내지만, 조작자가 급하면 순서를 건너뛰거나
    // 현재 단계가 끝나기 전에 다음 번호를 누를 수 있다.  그걸 막는다.
    //
    // 반환값을 둘로 나눠 GCS 가 대응을 구분할 수 있게 한다:
    //   DENIED               순서가 틀렸다 - 다시 보내도 소용없다
    //   TEMPORARILY_REJECTED 순서는 맞지만 아직 완료 전 - 잠시 뒤 재시도하면 된다
    if (!state_order_ok(_state, state)) {
        return MAV_RESULT_DENIED;
    }
    if (state != _state && !_state_done) {
        return MAV_RESULT_TEMPORARILY_REJECTED;
    }

    // 목표값 파싱 (state 6 에서만 유효).
    // 멤버에 반영하기 전에 먼저 전부 검사한다 - 중간에 거부하면 state 만 바뀐
    // 어중간한 상태가 남는다.
    Location loc;
    if (state == State::TRACKING) {
        if (!isfinite(packet.z) || !isfinite(packet.param2)) {
            return MAV_RESULT_DENIED;
        }

        switch (packet.frame) {

        case MAV_FRAME_GLOBAL_RELATIVE_ALT:
            // [최종 결과물] 위경도를 그대로 받는다.
            // int32 (1e7 deg) 를 그대로 넣으므로 부동소수 변환이 없어 무손실이다
            // (float 로 받으면 0.4~1.4 m 로 양자화된다).
            loc.lat = packet.x;
            loc.lng = packet.y;
            break;

        case MAV_FRAME_LOCAL_NED:
            // [SITL 테스트] home 기준 NEU 를 받아 위경도로 바꾼다.
            // x = North cm, y = East cm.  int32 cm 라 1 cm 단위가 유지된다.
            //
            // 여기서 위경도로 바꿔 두면 아래 단계(state 6 의 위치제어,
            // CLAW 전달)가 두 경우를 구분할 필요가 없다.
            if (!ahrs.home_is_set()) {
                return MAV_RESULT_DENIED;       // 기준점이 없으면 변환 불가
            }
            loc = ahrs.get_home();
            loc.offset(packet.x * 0.01,         // North (cm -> m)
                       packet.y * 0.01);        // East  (cm -> m)
            break;

        default:
            return MAV_RESULT_DENIED;
        }

        // 고도는 두 frame 모두 home 기준 up (m) 이다
        loc.set_alt_cm((int32_t)(packet.z * 100.0f),
                       Location::AltFrame::ABOVE_HOME);
    }

    // --- 여기서부터 반영 ---
    const State prev_state = _state;
    _state = state;

    if (state == State::TRACKING) {
        _target_loc = loc;
        _target_heading_deg = packet.param2;    // 진북 기준 (deg)
    }

    // 상태 전이 - state 진입 훅.
    if (_state != prev_state) {
        _state_entered = true;
        _state_start_ms = AP_HAL::millis();

        // 새 단계는 아직 안 끝났다.  담당 state_*() 가 자기 조건을 보고
        // 다시 true 로 올릴 때까지 다음 번호로 넘어갈 수 없다.
        _state_done = false;
    }

    return MAV_RESULT_ACCEPTED;
}

// 완료
// ---------------------------------------------------------------------------
// state 순서 검사
//
// 허용하는 것은 두 가지뿐이다.
//   같은 번호 재전송   state 6 은 타겟 갱신 때문에 5Hz 로 계속 들어온다
//   바로 다음 번호     N -> N+1  (NONE(0) -> 1 도 여기 해당)
//
// 건너뛰기(3 -> 6)와 되돌아가기(7 -> 6)는 모두 막는다.  모드에 다시 들어오면
// init() 이 _state 를 NONE 으로 되돌리므로 1번부터 다시 시작한다.
// ---------------------------------------------------------------------------

bool ModeTDCN::state_order_ok(State from, State to)
{
    if (to == from) {
        return true;                                    // 같은 번호 재전송
    }
    return (uint8_t)to == (uint8_t)from + 1;            // 바로 다음 번호만
}

// ---------------------------------------------------------------------------
// 이륙/착륙 상태 통보
//
// ArduPilot 본체(land_detector, failsafe, fence, throttle mix)가 지금 이륙
// 중인지 착륙 중인지 물어본다.  ModeAuto/ModeGuided 와 같은 방식으로 답한다.
// ---------------------------------------------------------------------------

bool ModeTDCN::is_taking_off() const
{
    // state 4 에서 auto_takeoff 가 목표 고도에 도달하기 전까지가 이륙 중이다.
    // (ModeAuto::is_taking_off() 와 동일)
    return (_state == State::LAUNCH) && !auto_takeoff.complete;
}

bool ModeTDCN::is_landing() const
{
    // state 9 (착륙 수납) 만 착륙이다.  state 8 (착륙 동기) 은 고도만 맞추는
    // 하강이므로 착륙으로 보지 않는다.
    return _state == State::LANDING_STOW;
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
    }
    return "?";
}

// 완료
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

    // Current Position
    //
    // Dest_poti 를 double 로 올렸으므로 (mode_tdcn_CLAW_types.h) 위경도가
    // 그대로 전달된다.  float32 이던 시절에는 여기서 위도 0.42 m / 경도
    // 0.675 m (127도 기준) 씩 양자화됐다.
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
// CLAW 출력 모니터링
//
// v1 은 CLAW 출력을 기체에 적용하지 않으므로, 값이 맞는지는 로그로만 확인한다.
// 400Hz 를 그대로 남기면 과하므로 8회마다 1회 (50Hz) 기록한다.
//
//   CLAW_Y.cur_poti  CLAW 가 계산한 home 기준 상대 위치 (m)
//                    = (STV[9], STV[10], -STV[11]) = (North, East, Up)
//   Err_N / E / D    CLAW 내부 위치 오차 (m).  N/E 는 leash 25m 로 clamp 된다
//   CLAW_Y.v_cmd     CLAW 제어 출력 (-1 ~ +1 정규화)
// ---------------------------------------------------------------------------

void ModeTDCN::Log_Write_TDCN()
{
#if HAL_LOGGING_ENABLED
    if (_log_counter++ % 8 != 0) {
        return;
    }

// @LoggerMessage: TDCN
// @Description: TDCN mode CLAW controller monitor
// @Field: TimeUS: Time since system startup
// @Field: St: TDCN scenario state, 1 to 11
// @Field: PN: CLAW computed position North, relative to CLAW home
// @Field: PE: CLAW computed position East, relative to CLAW home
// @Field: PU: CLAW computed position Up, relative to CLAW home
// @Field: EN: CLAW position error North
// @Field: EE: CLAW position error East
// @Field: ED: CLAW position error Down
// @Field: CR: CLAW roll command, normalised
// @Field: CP: CLAW pitch command, normalised
// @Field: CY: CLAW yaw command, normalised
// @Field: CH: CLAW height command, normalised
    AP::logger().WriteStreaming("TDCN",
                                "TimeUS,St,PN,PE,PU,EN,EE,ED,CR,CP,CY,CH",
                                "QBffffffffff",
                                AP_HAL::micros64(),
                                (uint8_t)_state,
                                (double)CLAW_Y.cur_poti.x,
                                (double)CLAW_Y.cur_poti.y,
                                (double)CLAW_Y.cur_poti.z,
                                (double)Err_N,
                                (double)Err_E,
                                (double)Err_D,
                                (double)CLAW_Y.v_cmd.cmd_roll,
                                (double)CLAW_Y.v_cmd.cmd_pitch,
                                (double)CLAW_Y.v_cmd.cmd_yaw,
                                (double)CLAW_Y.v_cmd.cmd_height);
#endif  // HAL_LOGGING_ENABLED
}

// ---------------------------------------------------------------------------
// CLAW 한 스텝
//
// CLAW 를 돌려야 하는 state_*() 에서 호출한다.  세 단계의 순서가 고정이다 -
// 입력이 채워져야 CLAW_step 이 의미가 있고, 출력은 CLAW_step 이후에 읽어야 한다.
//
// home 은 TDCN 이 관여하지 않는다.  CLAW 가 CLAW_step() 안에서 home_init 이
// false 인 동안 Cur_Pos 를 보고 스스로 캡처하므로, 여기서 줄 필요가 없다.
// (init() 이 home_init = false 로 만들어 두면 모드 재진입마다 다시 잡는다)
//
// v1 은 CLAW 출력을 기체에 적용하지 않고 로그로만 확인한다.
// ---------------------------------------------------------------------------

void ModeTDCN::Run_CLAW()
{
    Update_Info_for_CLAW();     // CLAW_U 에 기체 정보 + GCS 타겟 전달
    CLAW_step();                // CLAW 실행 (CLAW.c 의 함수 직접 호출)
    Log_Write_TDCN();           // CLAW 출력(위치 / 제어값) 받아오기
}

// ---------------------------------------------------------------------------
// state 1~11 별 처리
// ---------------------------------------------------------------------------

// 이륙 전 state (0~3) 의 기체 처리.
//
// 이 단계들은 "격납함 열기 / prearm / 무장" 이라 기체를 움직일 일이 없다.
// 그래도 제어를 아예 안 하면 안 된다.  모드는 언제든 바꿀 수 있어서 비행
// 중에 TDCN 으로 들어올 수 있는데, 그때 아무도 기체를 잡지 않으면 자세/추력
// 목표가 갱신되지 않은 채 그대로 가라앉는다 (SITL 에서 6.9m -> 2.7m 확인).
//
// 그래서 지상이면 모터를 안전하게 내려두고, 공중이면 그 자리를 유지한다.
// 유지 로직은 state 5 (비행 대기) 와 같은 GUIDED 방식이다.
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
    // 진입할 때 한 번만 arm 한다.  arm() 은 체크를 전부 다시 돌리고 실패 사유를
    // GCS 에 출력하므로 400Hz 로 부를 수 없다.
    // 성공/실패 메시지는 arm() 이 직접 GCS 에 보낸다.

    // 완료 판정: 실제로 무장돼야 4번으로 넘어갈 수 있다
    _state_done = motors->armed();

    // 무장 상태를 "유지" 한다.
    //
    // 다음 번호가 올 때까지 이 state 의 조건을 지켜야 한다.  지상에서 무장한
    // 채로 DISARM_DELAY (기본 10초) 가 지나면 아두파일럿이 자동으로 무장을
    // 푸는데 (Copter::auto_disarm_check), 그때 그냥 두면 state 3 의 조건이
    // 깨진 채로 남는다.  그래서 무장이 풀려 있으면 다시 무장한다.
    //
    // 재시도는 1Hz 로 제한한다.  arm() 은 실패할 때마다 사유를 GCS 에 출력하므로
    // 400Hz 로 부르면 링크가 메시지로 막힌다.  다만 "방금까지 무장돼 있다가
    // 풀린" 첫 순간에는 곧바로 시도한다.  이 경우는 직전에 arm 이 성공했으니
    // 실패 메시지가 쏟아질 일이 없고, 무장 공백을 한 루프로 줄일 수 있다.
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

static const float _takeoff_alt_cm    = 1000.0f;    // 목표 고도 (cm, home 기준 up)  10 m
static const float _takeoff_speed_cms =  100.0f;    // 상승 속도 (cm/s)              1 m/s
void ModeTDCN::state_launch()           // 4 이륙 사출
{
    // 무장 상태를 "유지" 한다.
    //
    // state 3 은 지상에서 DISARM_DELAY (기본 10초) 로 자동 무장 해제가 반복되고
    // 그때마다 state_armed() 가 다시 무장한다.  하필 그 무장이 풀린 순간에
    // 4번이 들어오면 이 state 가 무장 없이 시작된다.  auto_takeoff.run() 은
    // 무장이 없으면 아무 것도 못 하고, state 4 에는 회복 수단이 없으므로
    // 이륙도 못 하고 다음 번호로도 못 넘어간 채 영구히 멈춘다.
    // 그래서 이륙이 끝나기 전에 무장이 풀려 있으면 다시 무장하고, 무장이
    // 될 때까지 이륙 시작을 미룬다.
    bool takeoff_start = _state_entered;
    if (!_state_done && !motors->armed()) {
        const uint32_t now_ms = AP_HAL::millis();
        if (_state_entered || (now_ms - _action_retry_ms) >= 1000) {
            _action_retry_ms = now_ms;
            copter.arming.arm(AP_Arming::Method::MAVLINK);
        }
        if (!motors->armed()) {
            return;                 // 아직 무장 못 함.  다음 루프에서 재시도
        }
        takeoff_start = true;       // 재무장했으니 이륙을 처음부터 다시 시작
    }

    if (takeoff_start) {
        // --- 이륙 시작 (ModeGuided::do_user_takeoff_start() 와 같은 순서) ---

        // 이륙 중에는 heading 을 유지한다
        auto_yaw.set_mode(AutoYaw::Mode::HOLD);

        // 상승 속도 제한.  하강 속도도 같은 값으로 둔다 (이륙에는 쓰이지 않지만
        // 컨트롤러 한계를 비대칭으로 두지 않는다).
        pos_control->set_max_speed_accel_z(-_takeoff_speed_cms, _takeoff_speed_cms,
                                          g.pilot_accel_z);
        pos_control->set_correction_speed_accel_z(-_takeoff_speed_cms, _takeoff_speed_cms,
                                                  g.pilot_accel_z);

        // 수직 위치 컨트롤러 초기화 (I 항 클리어)
        pos_control->init_z_controller();

        // auto_takeoff 는 목표 고도를 EKF origin 기준 cm 로 받는다.
        // _takeoff_alt_cm 은 home 기준이므로 변환한다.
        Location target_loc = copter.current_loc;
        target_loc.set_alt_cm((int32_t)_takeoff_alt_cm, Location::AltFrame::ABOVE_HOME);
        int32_t alt_above_origin_cm;
        if (!target_loc.get_alt_cm(Location::AltFrame::ABOVE_ORIGIN,
                                   alt_above_origin_cm)) {
            gcs().send_text(MAV_SEVERITY_WARNING, "%s: takeoff alt failed", name());
            return;
        }
        auto_takeoff.start((float)alt_above_origin_cm, false);

        // auto_takeoff.run() 은 ap.auto_armed 가 false 면 즉시 리턴한다.
        // auto_armed 는 조종기 스로틀을 올려야 켜지는데 (system.cpp 의
        // update_auto_armed), 이 시나리오에는 스틱이 없으므로 직접 세운다.
        // Copter::start_takeoff() 도 같은 방식이다.
        copter.set_auto_armed(true);
    }

    // 이륙 제어.  complete 가 true 가 된 뒤에도 계속 호출하면 목표 고도와
    // 제자리(XY 속도 0)를 유지하므로 그대로 호버링한다.
    auto_takeoff.run();

    // 완료 판정: 목표 고도에 도달해야 5번으로 넘어갈 수 있다
    _state_done = auto_takeoff.complete;
}

// ---------------------------------------------------------------------------
// state 5 (비행 대기) - 이륙 위치/고도 유지
//
// [기반 모드] GUIDED (Position 서브모드)
//   초기화: ModeGuided::pva_control_start()   (mode_guided.cpp)
//   제어  : ModeGuided::pos_control_run()     (mode_guided.cpp)
//
// 자율비행 모드 중 "고정점 유지" 가 가장 정확한 것을 골랐다.  세 후보 비교:
//
//   GUIDED  pos_control->input_pos_xyz(고정 3D 목표)
//           목표를 좌표로 직접 준다.  적분이 없으므로 목표가 흐를 수 없고,
//           pos_control 까지 거치는 층이 가장 얇다.            <- 채택
//
//   AUTO    wp_nav->update_wpnav()  (ModeAuto::loiter_run)
//           정상상태 강성은 같다 (같은 pos_control / 같은 PSC_* 게인).  다만
//           S-curve 궤적 생성과 leash 를 거쳐 목표가 만들어지므로, 한 점을
//           지키는 용도로는 불필요한 층이 더 있다.
//
//   BRAKE   input_vel_accel_xy(0,0) + set_pos_target_z_from_climb_rate_cm(0)
//           "속도 0 / 상승률 0" 을 적분해 목표를 만든다.  고정점 보장이 가장
//           약하고, input_thrust_vector_rate_heading(...,0) 이라 heading 도
//           고정되지 않는다.  선박 정렬이 필요한 이 프로젝트에는 부적합.
//
// GUIDED 원본과 의도적으로 다른 점 2가지
//   1. init_xy/z_controller() 를 무조건 부르지 않는다.  Guided 는 새 목표를
//      현재 위치에서 시작하므로 매번 초기화하지만, 여기서는 state 4 의 이륙
//      위치를 지켜야 하므로 이미 활성인 컨트롤러는 건드리지 않는다.
//   2. auto_yaw 를 set_mode_to_default() 가 아니라 HOLD 로 둔다.  WP_YAW_BEHAVIOR
//      파라미터에 따라 heading 이 달라지지 않게 고정한다.
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// state 6 (추종 비행) - CLAW 검증 모드
//
// [목적] CLAW 는 외부에서 받은 코드이고 기체도 달라 게인이 맞지 않는다.  그래서
//        먼저 SITL 에서 CLAW 를 검증한다.
//
//   기체 제어 : 아두파일럿 위치제어가 GCS 타겟을 실제로 따라간다 (GUIDED 방식)
//   CLAW      : 같은 입력으로 병렬 실행.  출력은 로그로만 남기고 기체에 쓰지 않는다
//   비교      : TDCN 로그의 PN/PE/PU (CLAW 계산 위치) 와 CR/CP/CY/CH (CLAW 제어값) 를
//               아두파일럿이 실제로 만든 거동과 대조해 CLAW 를 판정한다
//
// [기반 모드] GUIDED (Position 서브모드) - state 5 와 동일한 제어 경로다.
//             차이는 목표가 "이륙 위치 고정" 이 아니라 "GCS 가 주는 타겟" 이라는 점.
// ---------------------------------------------------------------------------

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
    //   Log_Write_TDCN()        CLAW_Y.cur_poti (계산 위치) / CLAW_Y.v_cmd (제어값)
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
}

// ---------------------------------------------------------------------------
// state 7 (착륙 대기) - CLAW 종료 + 아두파일럿 호버 인수인계
//
// [역할] state 6 에서 CLAW 가 기체를 몰았다는 전제 아래, CLAW 를 끝내고
//        아두파일럿 위치제어로 넘겨받아 그 자리에서 안전하게 호버한다.
//        (state 5 와 결과는 같지만, 들어오는 경로가 달라 처리가 다르다)
//
// [왜 그냥 넘겨받으면 안 되는가]
//   CLAW 가 기체를 모는 동안 아두파일럿 pos_control 은 갱신되지 않는다.  그래서
//   내부 목표(_pos_desired)와 적분항이 "CLAW 가 몰기 시작한 시점" 에 멈춰 있다.
//   그 낡은 목표를 그대로 쓰면 기체가 옛 위치로 튄다.  자세제어기의 I 항도
//   CLAW 가 만든 자세를 쫓느라 쌓여 있어 그대로 두면 인계 직후 요동친다.
//
// [처리]
//   1. pos_control 을 "현재 상태" 에서 다시 초기화한다.
//      init_*_stopping_point() 를 쓰는 이유는, 인계 시점에 기체가 속도를 갖고
//      있을 수 있기 때문이다.  현재 속도와 감속 한계로 계산한 정지점을 목표로
//      잡으므로 급정거가 아니라 부드럽게 멈춘다.
//      (state 5 처럼 is_active 로 건너뛰면 낡은 목표를 그대로 쓰게 된다)
//   2. 자세제어기 I 항과 yaw 목표를 현재 자세 기준으로 리셋한다.
//   3. 그 정지점을 이후 계속 유지한다.
//
// [CLAW] 여기서 끝난다.  Run_CLAW() 를 부르지 않으므로 CLAW_step() 이 더는
//        돌지 않고 CLAW_Y 도 갱신되지 않는다.  기체는 전적으로 아두파일럿이
//        잡는다.
//
// [기반 모드] GUIDED (Position 서브모드)
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// state 8 (착륙 동기) - 착륙 준비 고도로 하강 후 호버
//
// XY 는 state 7 이 잡은 자리를 그대로 유지하고, 고도만 착륙 준비 고도로
// 내려간 뒤 그 자리에서 호버한다.
//
// [CLAW] 돌지 않는다.  state 7 에서 이미 끝났다.
//
// [기반 모드] GUIDED (Position 서브모드)
// ---------------------------------------------------------------------------

// state 8 착륙 동기 파라미터 - 여기서 직접 수정한다
static const float _sync_alt_cm       = 1000.0f;    // 목표 고도 (cm, home 기준 up)  10 m
static const float _sync_speed_dn_cms =  100.0f;    // 하강 속도 (cm/s)              1.0 m/s

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
        pos_control->set_max_speed_accel_z(-_sync_speed_dn_cms,
                                           wp_nav->get_default_speed_up(),
                                           wp_nav->get_accel_z());
        pos_control->set_correction_speed_accel_z(-_sync_speed_dn_cms,
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

    // 목표 고도를 EKF origin 기준 NEU cm 로 바꿔 Z 만 갈아끼운다.
    // _sync_alt_cm 은 home 기준이므로 현재 위치를 기준점으로 삼아 변환한다.
    // (매 루프 계산하므로 home 이 갱신되어도 따라간다)
    Location sync_loc = copter.current_loc;
    sync_loc.set_alt_cm((int32_t)_sync_alt_cm, Location::AltFrame::ABOVE_HOME);
    Vector3f sync_neu_cm;
    if (sync_loc.get_vector_from_origin_NEU(sync_neu_cm)) {
        _hold_pos_neu_cm.z = sync_neu_cm.z;     // XY 는 건드리지 않는다
    }

    // 완료 판정: 착륙 준비 고도에 도달해야 9번으로 넘어갈 수 있다
    _state_done = fabsf((float)copter.current_loc.alt - _sync_alt_cm) < 50.0f;

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

// ---------------------------------------------------------------------------
// state 9 (착륙 수납) - 현재 자리에 착륙
//
// [기반 모드] LAND (GPS 있는 경우)
//   초기화  ModeLand::init() 과 동일
//   제어    Mode::land_run_normal_or_precland()
//
// 착륙 로직을 새로 짜지 않고 아두파일럿 공용 함수를 그대로 호출한다.  그래야
// 착지 감지(land detector), 지면 효과 보정, 정밀 착륙(AC_PRECLAND), 조종자
// 재위치(land_repo) 같은 검증된 처리가 전부 따라온다.
//
// [착륙 속도] 파라미터로 조정한다.  TDCN 안에 변수를 두지 않는다.
//
//   LAND_SPEED       최종 접지 속도 (cm/s, 기본 50).  LAND_ALT_LOW 아래 구간.
//   LAND_SPEED_HIGH  고고도 하강 속도 (cm/s, 기본 0 = WPNAV_SPEED_DN 사용)
//   LAND_ALT_LOW     이 고도(cm, 기본 1000 = 10 m) 아래로 내려오면 LAND_SPEED 로 감속
//
//   state 8 이 기체를 LAND_ALT_LOW(10 m)에 놓으므로, 기본 설정에서는 state 9
//   전 구간이 최종 저속 하강(LAND_SPEED)이다.  더 빠르게/느리게 하려면
//   LAND_SPEED 를 바꾸면 된다.  고고도 구간까지 조절하려면 state 8 의
//   _sync_alt_cm 을 LAND_ALT_LOW 보다 높게 잡고 LAND_SPEED_HIGH 를 설정한다.
//
// [CLAW] 돌지 않는다.  state 7 에서 이미 끝났다.
//
// [disarm] 여기서 하지 않는다.  시나리오에 state 10(DISARMED)이 따로 있다.
//          (ModeLand 는 착지 감지 시 스스로 disarm 한다 - 그 부분만 뺐다)
// ---------------------------------------------------------------------------

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

    // 아두파일럿 착륙 제어 (수평 유지 + 수직 하강).
    // 정밀 착륙이 켜져 있으면 그쪽으로, 아니면 일반 착륙으로 분기한다.
    land_run_normal_or_precland();
}

// ---------------------------------------------------------------------------
// state 10 (DISARMED) - 착륙 후 무장 해제
//
// state 3(ARMED)의 반대다.  진입할 때 한 번만 disarm 한다.
//
// [안전 1] Method::MAVLINK 를 쓰면 아두파일럿이 "비행 중 GCS disarm" 을 막아준다.
//   AP_Arming_Copter::disarm() 이 method_is_GCS(method) && !land_complete 이면
//   그냥 false 를 반환한다.  공중에서 프로펠러가 멈추는 일은 없다.
//
// [안전 2] 그런데 disarm 이 거부되면 기체는 여전히 떠 있다.  그 상태에서
//   make_safe_ground_handling() 을 부르면 모터가 ground idle 로 내려가 기체가
//   추락한다.  그래서 착지 상태일 때만 지상 처리를 하고, 아직 비행 중이면
//   제자리 호버를 유지한다.
//
// [CLAW] 돌지 않는다.  state 7 에서 이미 끝났다.
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// state 11 (격납함 닫기) - 기체가 할 일 없음
//
// state 1(격납함 열기)과 대칭이다.  격납함 개폐는 기체 밖에서 처리된다.
//
// state 10 에서 이미 disarm 되어 지상에 있으므로, 모터가 꺼진 상태를 유지하도록
// 지상 처리만 해 둔다 (자세제어 적분항 / yaw 목표를 계속 리셋).
//
// [CLAW] 돌지 않는다.  state 7 에서 이미 끝났다.
// ---------------------------------------------------------------------------

void ModeTDCN::state_hangar_close()     // 11 격납함 닫기
{
    // 마지막 단계 - 뒤가 없으므로 완료로 둔다
    _state_done = true;

    // 지상 대기.  혹시 무장 상태로 들어와도 모터를 올리지 않는다.
    make_safe_ground_handling();
}


#endif  // MODE_TDCN_ENABLED