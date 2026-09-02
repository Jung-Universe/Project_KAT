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

// CLAW 내부 상태.  진단 로그(TDC2)에만 쓴다.  CLAW.h 에 선언이 없지만 셋 다
// CLAW.c 의 file-scope 전역이고 static 이 아니라 vendor 파일 수정 없이 닿는다.
extern double TV_BSC[4];      // 적분기.  [0..1] 위치오차(N,E), [2..3] 속도오차(v,u)
extern double pos_dot_des[3]; // NED 속도 명령.  이것을 회전시킨 것이 uv_des 다
extern double uv_des[3];      // body 속도 명령 (u,v).  max_vel 5 m/s 로 clamp 된다
extern double Xtraj[4], dXtraj[4];  // 궤적 생성기 출력/미분 [zz, roll, pitch, yaw]
extern double alpha[4];       // 백스테핑 가상 제어 = 각속도 명령 [zz, p, q, r]
extern double Del_Control[4]; // 백스테핑 최종 출력.  scale/clamp 전 [thr,roll,pitch,yaw]
extern double ps_cmd;         // CLAW 가 해석한 타겟 헤딩 (rad) = wrapToPi(Ship_heading)

// CLAW 가 입력을 받아 넣는 내부 변수.  vendor 파일에서 지역변수를 파일 전역으로
// 올려 두었다.  TDCN 이 넘긴 값과 짝지어 비교하는 데 쓴다.
extern double Cur_Lat, Cur_Lon, Cur_Alt;      // Cur_Alt 는 down 양수
extern double Dest_Lat, Dest_Lon, Dest_Alt;   // Dest_Alt 는 down 양수
}

// Home_Lat / Home_Lon / Home_Alt / Home_Yaw 와 STV[12] 는 mode_tdcn_CLAW.h
// 가 이미 extern 선언해 두었으므로 여기서 다시 선언하지 않는다.
// (CLAW 내부의 home_init 은 file-scope static 이라 외부에서 건드릴 수 없다)

// ---------------------------------------------------------------------------
// 파라미터
//
// 시나리오가 쓰는 고도 / 속도를 소스에서 빼 파라미터로 만든다.  박아 두면 값을
// 바꿀 때마다 재빌드해야 하고 실기체에서는 현장 조정이 불가능하다.
//
// 여기 없는 것 (이미 아두파일럿 파라미터로 조절 가능하다):
//   state 5/6/7/8 의 수평 이동 속도   WPNAV_SPEED, WPNAV_ACCEL
//   state 5/6/7 의 수직 속도          WPNAV_SPEED_UP, WPNAV_SPEED_DN, WPNAV_ACCEL_Z
//   state 9 의 착륙 속도              LAND_SPEED, LAND_SPEED_HIGH, LAND_ALT_LOW
// ---------------------------------------------------------------------------

const AP_Param::GroupInfo ModeTDCN::var_info[] = {

    // @Param: TKO_ALT
    // @DisplayName: TDCN takeoff altitude
    // @Description: Target altitude for state 4 launch, above home
    // @Units: cm
    // @Range: 100 5000
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("TKO_ALT", 1, ModeTDCN, _takeoff_alt, 1000),

    // @Param: TKO_SPD
    // @DisplayName: TDCN takeoff climb speed
    // @Description: Climb speed used during state 4 launch
    // @Units: cm/s
    // @Range: 20 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("TKO_SPD", 2, ModeTDCN, _takeoff_spd, 100),

    // @Param: LND_ALT
    // @DisplayName: TDCN landing sync altitude
    // @Description: Altitude held at the end of state 8 landing sync, above home. State 9 starts its descent from here.
    // @Units: cm
    // @Range: 100 5000
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("LND_ALT", 3, ModeTDCN, _land_alt, 1000),

    // @Param: LND_SPD
    // @DisplayName: TDCN landing sync descent speed
    // @Description: Descent speed used during state 8 landing sync. The final touchdown in state 9 uses LAND_SPEED instead.
    // @Units: cm/s
    // @Range: 20 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("LND_SPD", 4, ModeTDCN, _land_spd, 100),

    // @Param: CLAW_ON_OFF
    // @DisplayName: TDCN use CLAW control output
    // @Description: 0 leaves ArduPilot flying the vehicle with CLAW running in parallel for monitoring only. 1 replaces the ArduPilot roll pitch yaw and throttle mixer inputs with the CLAW output during state 6 tracking. Only take off with 1 after the CLAW gains have been verified for this airframe.
    // @Values: 0:ArduPilot flies CLAW monitors,1:CLAW flies
    // @User: Advanced
    AP_GROUPINFO("CLAW_ON_OFF", 5, ModeTDCN, _claw_on_off, 0),

    AP_GROUPEND
};

ModeTDCN::ModeTDCN(void) : Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

// ---------------------------------------------------------------------------
// CLAW 게인 파라미터
//
// mode_tdcn_CLAW_data_0729.c 의 CLAW_P 초기값을 그대로 기본값으로 옮겼다.
// 파라미터를 건드리지 않으면 기존과 완전히 같은 값으로 동작한다.
//
// BSC_B_mat (제어효과 행렬, 48개) 은 제외했다.  기체 제원에서 나오는 값이라
// 현장에서 조정할 성질이 아니고, 파라미터로 내기에도 개수가 맞지 않는다.
// ---------------------------------------------------------------------------

const AP_Param::GroupInfo CLAW_Gains::var_info[] = {

    // @Param: SCALE_TH
    // @DisplayName: CLAW thrust output scale
    // @Description: Scales the backstepping thrust output into the normalised command range
    // @Range: 0.1 10
    // @User: Advanced
    AP_GROUPINFO("SCALE_TH", 1, CLAW_Gains, _scale_th, 3.2),

    // @Param: SCALE_R
    // @DisplayName: CLAW roll output scale
    // @Description: Scales the backstepping roll output into the normalised command range
    // @Range: 0.01 2
    // @User: Advanced
    AP_GROUPINFO("SCALE_R", 2, CLAW_Gains, _scale_r, 0.25),

    // @Param: SCALE_P
    // @DisplayName: CLAW pitch output scale
    // @Description: Scales the backstepping pitch output into the normalised command range
    // @Range: 0.01 2
    // @User: Advanced
    AP_GROUPINFO("SCALE_P", 3, CLAW_Gains, _scale_p, 0.20),

    // @Param: SCALE_Y
    // @DisplayName: CLAW yaw output scale
    // @Description: Scales the backstepping yaw output into the normalised command range
    // @Range: 0.01 2
    // @User: Advanced
    AP_GROUPINFO("SCALE_Y", 4, CLAW_Gains, _scale_y, 0.13),

    // @Param: K_POS_P
    // @DisplayName: CLAW outer loop position P
    // @Description: Position error to velocity command gain
    // @Range: 0 2
    // @User: Advanced
    AP_GROUPINFO("K_POS_P", 5, CLAW_Gains, _k_pos_p, 0.3),

    // @Param: K_POS_I
    // @DisplayName: CLAW outer loop position I
    // @Description: Position error integral gain
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("K_POS_I", 6, CLAW_Gains, _k_pos_i, 0.01),

    // @Param: K_VEL_P
    // @DisplayName: CLAW outer loop velocity P
    // @Description: Velocity error to attitude command gain
    // @Range: 0 2
    // @User: Advanced
    AP_GROUPINFO("K_VEL_P", 7, CLAW_Gains, _k_vel_p, 0.4),

    // @Param: K_VEL_I
    // @DisplayName: CLAW outer loop velocity I
    // @Description: Velocity error integral gain
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("K_VEL_I", 8, CLAW_Gains, _k_vel_i, 0.05),

    // @Param: AWU_LIMIT
    // @DisplayName: CLAW integrator limit
    // @Description: Anti windup clamp applied to all four CLAW integrators
    // @Range: 0.1 10
    // @User: Advanced
    AP_GROUPINFO("AWU_LIMIT", 9, CLAW_Gains, _awu_limit, 1.0),

    // @Param: OMEGA_XX
    // @DisplayName: CLAW trajectory natural frequency North
    // @Description: Natural frequency of the North axis trajectory filter
    // @Units: rad/s
    // @Range: 0.05 20
    // @User: Advanced
    AP_GROUPINFO("OMEGA_XX", 10, CLAW_Gains, _ome_xx, 0.3),

    // @Param: OMEGA_YY
    // @DisplayName: CLAW trajectory natural frequency East
    // @Description: Natural frequency of the East axis trajectory filter
    // @Units: rad/s
    // @Range: 0.05 20
    // @User: Advanced
    AP_GROUPINFO("OMEGA_YY", 11, CLAW_Gains, _ome_yy, 0.3),

    // @Param: OMEGA_ZZ
    // @DisplayName: CLAW trajectory natural frequency height
    // @Description: Natural frequency of the height trajectory filter
    // @Units: rad/s
    // @Range: 0.05 20
    // @User: Advanced
    AP_GROUPINFO("OMEGA_ZZ", 12, CLAW_Gains, _ome_zz, 2.0),

    // @Param: OMEGA_PH
    // @DisplayName: CLAW trajectory natural frequency roll
    // @Description: Natural frequency of the roll trajectory filter
    // @Units: rad/s
    // @Range: 0.05 30
    // @User: Advanced
    AP_GROUPINFO("OMEGA_PH", 13, CLAW_Gains, _ome_ph, 9.3),

    // @Param: OMEGA_TH
    // @DisplayName: CLAW trajectory natural frequency pitch
    // @Description: Natural frequency of the pitch trajectory filter
    // @Units: rad/s
    // @Range: 0.05 30
    // @User: Advanced
    AP_GROUPINFO("OMEGA_TH", 14, CLAW_Gains, _ome_th, 12.0),

    // @Param: OMEGA_PS
    // @DisplayName: CLAW trajectory natural frequency yaw
    // @Description: Natural frequency of the yaw trajectory filter
    // @Units: rad/s
    // @Range: 0.05 30
    // @User: Advanced
    AP_GROUPINFO("OMEGA_PS", 15, CLAW_Gains, _ome_ps, 3.0),

    // @Param: ZETA_XX
    // @DisplayName: CLAW trajectory damping North
    // @Description: Damping ratio of the North axis trajectory filter
    // @Range: 0.1 2
    // @User: Advanced
    AP_GROUPINFO("ZETA_XX", 16, CLAW_Gains, _zeta_xx, 1.015),

    // @Param: ZETA_YY
    // @DisplayName: CLAW trajectory damping East
    // @Description: Damping ratio of the East axis trajectory filter
    // @Range: 0.1 2
    // @User: Advanced
    AP_GROUPINFO("ZETA_YY", 17, CLAW_Gains, _zeta_yy, 1.015),

    // @Param: ZETA_ZZ
    // @DisplayName: CLAW trajectory damping height
    // @Description: Damping ratio of the height trajectory filter
    // @Range: 0.1 2
    // @User: Advanced
    AP_GROUPINFO("ZETA_ZZ", 18, CLAW_Gains, _zeta_zz, 0.75),

    // @Param: ZETA_PH
    // @DisplayName: CLAW trajectory damping roll
    // @Description: Damping ratio of the roll trajectory filter
    // @Range: 0.1 2
    // @User: Advanced
    AP_GROUPINFO("ZETA_PH", 19, CLAW_Gains, _zeta_ph, 0.98),

    // @Param: ZETA_TH
    // @DisplayName: CLAW trajectory damping pitch
    // @Description: Damping ratio of the pitch trajectory filter
    // @Range: 0.1 2
    // @User: Advanced
    AP_GROUPINFO("ZETA_TH", 20, CLAW_Gains, _zeta_th, 0.98),

    // @Param: ZETA_PS
    // @DisplayName: CLAW trajectory damping yaw
    // @Description: Damping ratio of the yaw trajectory filter
    // @Range: 0.1 2
    // @User: Advanced
    AP_GROUPINFO("ZETA_PS", 21, CLAW_Gains, _zeta_ps, 0.9),

    // @Param: TAU_HDOT
    // @DisplayName: CLAW climb rate time constant
    // @Description: Time constant of the climb rate channel
    // @Units: s
    // @Range: 0.01 2
    // @User: Advanced
    AP_GROUPINFO("TAU_HDOT", 22, CLAW_Gains, _tau_hdot, 0.164297),

    // @Param: TAU_R
    // @DisplayName: CLAW yaw rate time constant
    // @Description: Time constant of the yaw rate channel
    // @Units: s
    // @Range: 0.01 2
    // @User: Advanced
    AP_GROUPINFO("TAU_R", 23, CLAW_Gains, _tau_r, 0.150985),

    AP_GROUPEND
};

CLAW_Gains::CLAW_Gains(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

void CLAW_Gains::apply(void) const
{
    CLAW_P.BSC_Scale_Thrust = _scale_th;
    CLAW_P.BSC_Scale_Roll   = _scale_r;
    CLAW_P.BSC_Scale_Pitch  = _scale_p;
    CLAW_P.BSC_Scale_Yaw    = _scale_y;

    CLAW_P.BSC_K_POS_P = _k_pos_p;
    CLAW_P.BSC_K_POS_I = _k_pos_i;
    CLAW_P.BSC_K_VEL_P = _k_vel_p;
    CLAW_P.BSC_K_VEL_I = _k_vel_i;
    CLAW_P.BSC_Int_Limit = _awu_limit;

    CLAW_P.BSC_Ome_XX = _ome_xx;
    CLAW_P.BSC_Ome_YY = _ome_yy;
    CLAW_P.BSC_Ome_ZZ = _ome_zz;
    CLAW_P.BSC_Ome_PH = _ome_ph;
    CLAW_P.BSC_Ome_TH = _ome_th;
    CLAW_P.BSC_Ome_PS = _ome_ps;

    CLAW_P.BSC_Zeta_XX = _zeta_xx;
    CLAW_P.BSC_Zeta_YY = _zeta_yy;
    CLAW_P.BSC_Zeta_ZZ = _zeta_zz;
    CLAW_P.BSC_Zeta_PH = _zeta_ph;
    CLAW_P.BSC_Zeta_TH = _zeta_th;
    CLAW_P.BSC_Zeta_PS = _zeta_ps;

    CLAW_P.BSC_Tau_hdot = _tau_hdot;
    CLAW_P.BSC_Tau_r    = _tau_r;

    // BSC_B_mat 은 건드리지 않는다 - data_0729.c 의 값을 그대로 쓴다.
}

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

    // 무장 시각.  이미 무장된 채로 모드에 들어왔더라도 "방금 무장한 것" 으로
    // 보수적으로 잡는다.  지상이면 state 4 가 정착 대기를 한 번 거치게 되고,
    // 공중이면 어차피 대기 조건(is_disarmed_or_landed)에 안 걸린다.
    _armed_ms = AP_HAL::millis();
    _armed_prev = motors->armed();
    _takeoff_started = false;

    _in_cur_lat = _in_cur_lng = _in_cur_alt = 0.0;
    _in_dst_lat = _in_dst_lng = _in_dst_alt = 0.0;
    _in_vel_n = _in_vel_e = _in_vel_d = 0.0;
    _in_p = _in_q = _in_r = 0.0f;
    _in_roll = _in_pitch = _in_yaw = 0.0f;
    _in_ship_hdg = 0.0f;
    _ap_roll_out = _ap_pitch_out = _ap_yaw_out = _ap_throttle_out = 0.0f;

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

    // 무장 엣지 추적.  state 4 가 "무장한 지 얼마나 됐는가" 를 봐야 하는데,
    // 무장은 state 3 / state 4 / 자동 해제 후 재무장 등 여러 경로로 걸리므로
    // 한 곳에서 모아 잡는다.
    {
        const bool armed_now = motors->armed();
        if (armed_now && !_armed_prev) {
            _armed_ms = AP_HAL::millis();
        }
        _armed_prev = armed_now;
    }

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

        // z 는 home 기준 상대고도다.  기준점이 없으면 아래 set_alt_cm 이
        // 붙이는 ABOVE_HOME 플래그가 의미를 잃는다.  그때 명령을 수락해 두면
        // state 6 의 get_vector_from_origin_NEU() 가 고도 변환 단계에서
        // 조용히 실패하고 (Location::get_alt_cm 의 ABOVE_HOME 분기),
        // 위경도는 변환조차 되지 않은 채 직전 타겟이 유지된다.  GCS 는
        // ACCEPTED 를 받았으므로 타겟이 반영된 줄로 안다.
        //
        // frame 1 은 자체 검사가 있지만 이 줄은 두 frame 이 공유하므로
        // 여기서 막는다 (frame 1 을 제거해도 검사가 남는다).
        if (!ahrs.home_is_set()) {
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
// TDCN <-> CLAW 인터페이스 검증 로그
//
// 통합에서 가장 먼저 확정해야 할 것은 "TDCN 이 넘긴 값을 CLAW 가 같은 값으로
// 인식하는가" 다.  그래서 Update_Info_for_CLAW() 가 채운 값과, CLAW_step() 이
// 그것을 받아 내부 변수에 넣은 값을 짝지어 남긴다.
//
//   TDCP  현재 위치   Cur_Pos       <-> Cur_Lat  / Cur_Lon  / Cur_Alt
//   TDCT  타겟        Dest_poti_i   <-> Dest_Lat / Dest_Lon / Dest_Alt
//                     Ship_heading  <-> ps_cmd
//   TDCI  자세/각속도 p,q,r,Roll,Pitch,DR_heading <-> STV[3..8]
//   TDCV  속도        XTV[0..2] (넘긴 값)         <-> XTV[0..2] (CLAW 가 본 값)
//
// [시퀀스] 같은 스텝끼리 비교해야 의미가 있다.  그래서
//   Update_Info_for_CLAW() -> [넘긴 값 스냅샷] -> CLAW_step() -> [CLAW 내부값 읽기]
// 순으로 고정했다.  로그 데시메이션(8회당 1회)도 이 경로 안에서 걸리므로 두
// 쪽이 한 스텝 어긋날 일이 없다.
//
// [고도] CLAW 는 안에서 down 양수로 뒤집는다 (Cur_Alt = -Cur_Pos.z).  그래서
// 넘긴 쪽도 down 으로 뒤집어 남긴다.  양쪽 모두 down 양수다.
//
// [바뀌는 것] 아래는 CLAW 가 의도적으로 바꾸는 부분이라 차이가 나는 게 정상이다.
//   Dest_*   home 을 래치하는 그 한 스텝만 현재 위치로 덮어쓴다 (CLAW.c L187-189)
//   STV[8]   wrapToPi(DR_heading)
//   ps_cmd   wrapToPi(Ship_heading)
// 나머지는 대입만 하므로 모든 스텝에서 정확히 같아야 한다.
//
// ---------------------------------------------------------------------------
// 제어 비교 로그 (인터페이스가 확정된 뒤에 본다)
//
//   TDCA  목표자세   Xtraj[1..3]  <-> 아두파일럿 attitude target
//   TDCR  각속도     alpha[1..3]  <-> 아두파일럿 rate target
//   TDCC  제어출력   v_cmd        <-> motors 출력 (+ clamp 전 값)
//   TDCE  CLAW 내부  오차 / 적분기 / 속도명령 / 궤적
// ---------------------------------------------------------------------------

void ModeTDCN::Log_Write_TDCN()
{
#if HAL_LOGGING_ENABLED
    if (_log_counter++ % 8 != 0) {
        return;
    }

    const uint64_t now_us = AP_HAL::micros64();

// @LoggerMessage: TDCP
// @Description: TDCN current position handed to CLAW vs read inside CLAW
// @Field: TimeUS: Time since system startup
// @Field: St: TDCN scenario state, 1 to 11
// @Field: TLat: Current latitude handed to CLAW
// @Field: TLng: Current longitude handed to CLAW
// @Field: TDwn: Current altitude handed to CLAW, down positive
// @Field: CLat: Current latitude inside CLAW
// @Field: CLng: Current longitude inside CLAW
// @Field: CDwn: Current altitude inside CLAW, down positive
    AP::logger().WriteStreaming("TDCP",
                                "TimeUS,St,TLat,TLng,TDwn,CLat,CLng,CDwn",
                                "QBddfddf",
                                now_us,
                                (uint8_t)_state,
                                _in_cur_lat,
                                _in_cur_lng,
                                (double)(-_in_cur_alt),     // up -> down
                                Cur_Lat,
                                Cur_Lon,
                                (double)Cur_Alt);

// @LoggerMessage: TDCT
// @Description: TDCN target handed to CLAW vs read inside CLAW
// @Field: TimeUS: Time since system startup
// @Field: TLat: Target latitude handed to CLAW
// @Field: TLng: Target longitude handed to CLAW
// @Field: TDwn: Target altitude handed to CLAW, down positive
// @Field: THdg: Target heading handed to CLAW
// @Field: CLat: Target latitude inside CLAW
// @Field: CLng: Target longitude inside CLAW
// @Field: CDwn: Target altitude inside CLAW, down positive
// @Field: CHdg: Target heading inside CLAW, after wrapToPi
    AP::logger().WriteStreaming("TDCT",
                                "TimeUS,TLat,TLng,TDwn,THdg,CLat,CLng,CDwn,CHdg",
                                "Qddffddff",
                                now_us,
                                _in_dst_lat,
                                _in_dst_lng,
                                (double)(-_in_dst_alt),     // up -> down
                                (double)degrees(_in_ship_hdg),
                                Dest_Lat,
                                Dest_Lon,
                                (double)Dest_Alt,
                                (double)degrees(ps_cmd));

// @LoggerMessage: TDCI
// @Description: TDCN IMU values handed to CLAW vs read inside CLAW
// @Field: TimeUS: Time since system startup
// @Field: Tp: Roll rate handed to CLAW
// @Field: Tq: Pitch rate handed to CLAW
// @Field: Tr: Yaw rate handed to CLAW
// @Field: TRol: Roll angle handed to CLAW
// @Field: TPit: Pitch angle handed to CLAW
// @Field: TYaw: Yaw angle handed to CLAW
// @Field: Cp: Roll rate inside CLAW, STV3
// @Field: Cq: Pitch rate inside CLAW, STV4
// @Field: Cr: Yaw rate inside CLAW, STV5
// @Field: CRol: Roll angle inside CLAW, STV6
// @Field: CPit: Pitch angle inside CLAW, STV7
// @Field: CYaw: Yaw angle inside CLAW, STV8 after wrapToPi
    AP::logger().WriteStreaming("TDCI",
                                "TimeUS,Tp,Tq,Tr,TRol,TPit,TYaw,Cp,Cq,Cr,CRol,CPit,CYaw",
                                "Qffffffffffff",
                                now_us,
                                (double)_in_p,
                                (double)_in_q,
                                (double)_in_r,
                                (double)_in_roll,
                                (double)_in_pitch,
                                (double)_in_yaw,
                                (double)STV[3],
                                (double)STV[4],
                                (double)STV[5],
                                (double)STV[6],
                                (double)STV[7],
                                (double)STV[8]);

// @LoggerMessage: TDCV
// @Description: TDCN velocity handed to CLAW vs read inside CLAW, NED
// @Field: TimeUS: Time since system startup
// @Field: TVN: North velocity handed to CLAW
// @Field: TVE: East velocity handed to CLAW
// @Field: TVD: Down velocity handed to CLAW
// @Field: CVN: North velocity inside CLAW, XTV0
// @Field: CVE: East velocity inside CLAW, XTV1
// @Field: CVD: Down velocity inside CLAW, XTV2
    AP::logger().WriteStreaming("TDCV",
                                "TimeUS,TVN,TVE,TVD,CVN,CVE,CVD",
                                "Qffffff",
                                now_us,
                                (double)_in_vel_n,
                                (double)_in_vel_e,
                                (double)_in_vel_d,
                                (double)XTV[0],
                                (double)XTV[1],
                                (double)XTV[2]);

    // --- NED 위치 비교 (TDCL) ---
    //
    // 아두파일럿은 EKF 가 낸 NED 를 그대로 쓰고, CLAW 는 위경도를 받아 자기
    // 식(Lat2m/Lon2m)으로 NED 를 다시 만든다.  그 변환식 차이를 보는 것이 목적이다.
    //
    // [원점] 서로 다르다.  아두파일럿은 EKF origin, CLAW 는 state 6 진입 위치다.
    // 제자리 이륙 후 넘어오므로 수평은 거의 같고 고도만 진입 고도만큼 차이난다.
    // 원점을 억지로 맞추지 않고 각자 값을 그대로 남긴다 - 맞추려면 환산식이
    // 하나 더 끼어들어 정작 보려는 변환식 차이가 가려진다.
    //
    // [부호] 양쪽 다 down 양수다.  CLAW 의 STV[11] 은 Cur_Alt - Home_Alt 로
    // 이미 down 양수이고, 아두파일럿은 NEU 의 z 를 뒤집어 맞춘다.
    const Vector3f &pos_neu_cm = inertial_nav.get_position_neu_cm();

// @LoggerMessage: TDCL
// @Description: TDCN NED position, ArduPilot EKF vs CLAW lat lon conversion
// @Field: TimeUS: Time since system startup
// @Field: AN: ArduPilot North, from EKF origin
// @Field: AE: ArduPilot East, from EKF origin
// @Field: AD: ArduPilot Down, from EKF origin
// @Field: CN: CLAW North, STV9, from CLAW home
// @Field: CE: CLAW East, STV10, from CLAW home
// @Field: CD: CLAW Down, STV11, from CLAW home
    AP::logger().WriteStreaming("TDCL",
                                "TimeUS,AN,AE,AD,CN,CE,CD",
                                "Qffffff",
                                now_us,
                                (double)(pos_neu_cm.x * 0.01f),
                                (double)(pos_neu_cm.y * 0.01f),
                                (double)(-pos_neu_cm.z * 0.01f),   // up -> down
                                (double)STV[9],
                                (double)STV[10],
                                (double)STV[11]);

// @LoggerMessage: TDCA
// @Description: TDCN attitude target, CLAW vs ArduPilot
// @Field: TimeUS: Time since system startup
// @Field: XR: CLAW trajectory generator roll target
// @Field: XP: CLAW trajectory generator pitch target
// @Field: XY: CLAW trajectory generator yaw target
// @Field: DR: ArduPilot attitude controller roll target
// @Field: DP: ArduPilot attitude controller pitch target
// @Field: DY: ArduPilot attitude controller yaw target
    AP::logger().WriteStreaming("TDCA",
                                "TimeUS,XR,XP,XY,DR,DP,DY",
                                "Qffffff",
                                now_us,
                                (double)Xtraj[1],
                                (double)Xtraj[2],
                                (double)Xtraj[3],
                                (double)attitude_control->get_att_target_euler_rad().x,
                                (double)attitude_control->get_att_target_euler_rad().y,
                                (double)attitude_control->get_att_target_euler_rad().z);

// @LoggerMessage: TDCR
// @Description: TDCN angular rate command, CLAW vs ArduPilot
// @Field: TimeUS: Time since system startup
// @Field: CRR: CLAW roll rate command
// @Field: CRP: CLAW pitch rate command
// @Field: CRY: CLAW yaw rate command
// @Field: ARR: ArduPilot roll rate target
// @Field: ARP: ArduPilot pitch rate target
// @Field: ARY: ArduPilot yaw rate target
    AP::logger().WriteStreaming("TDCR",
                                "TimeUS,CRR,CRP,CRY,ARR,ARP,ARY",
                                "Qffffff",
                                now_us,
                                (double)alpha[1],
                                (double)alpha[2],
                                (double)alpha[3],
                                (double)attitude_control->get_rate_ef_targets().x,
                                (double)attitude_control->get_rate_ef_targets().y,
                                (double)attitude_control->get_rate_ef_targets().z);

// @LoggerMessage: TDCC
// @Description: TDCN control output, CLAW vs ArduPilot, normalised
// @Field: TimeUS: Time since system startup
// @Field: CR: CLAW roll command
// @Field: CP: CLAW pitch command
// @Field: CY: CLAW yaw command
// @Field: CH: CLAW height command
// @Field: MR: ArduPilot roll input to mixer, rate PID plus feedforward
// @Field: MP: ArduPilot pitch input to mixer, rate PID plus feedforward
// @Field: MY: ArduPilot yaw input to mixer, rate PID plus feedforward
// @Field: MT: ArduPilot throttle input to mixer, 0 to 1
// @Field: UR: CLAW roll command before clamp
// @Field: UP: CLAW pitch command before clamp
// @Field: UY: CLAW yaw command before clamp
// @Field: UH: CLAW height command before clamp
// @Field: ACT: 1 when the CLAW output is actually driving the mixer
    AP::logger().WriteStreaming("TDCC",
                                "TimeUS,CR,CP,CY,CH,MR,MP,MY,MT,UR,UP,UY,UH,ACT",
                                "QffffffffffffB",
                                now_us,
                                (double)CLAW_Y.v_cmd.cmd_roll,
                                (double)CLAW_Y.v_cmd.cmd_pitch,
                                (double)CLAW_Y.v_cmd.cmd_yaw,
                                (double)CLAW_Y.v_cmd.cmd_height,
                                // 믹서가 실제로 소비하는 값과 같게 맞춘다.
                                // AP_MotorsMatrix::output_armed_stabilizing() 은
                                //   roll_thrust = (_roll_in + _roll_in_ff) * gain
                                // 처럼 rate PID 출력에 피드포워드를 더해 쓴다.
                                // get_roll() 은 _roll_in 만이라 그것만 비교하면
                                // 아두파일럿 몫을 과소평가한다.  스로틀은
                                // 피드포워드가 없어 get_throttle() 그대로다.
                                (double)_ap_roll_out,
                                (double)_ap_pitch_out,
                                (double)_ap_yaw_out,
                                (double)_ap_throttle_out,
                                // CLAW.c 의 스케일 순서를 그대로 재현한다.
                                // clamp 만 빼면 CR..CH 와 같아야 하므로,
                                // 벌어지는 구간이 곧 포화 구간이다.
                                (double)(Del_Control[1] * CLAW_P.BSC_Scale_Roll),
                                (double)(Del_Control[2] * CLAW_P.BSC_Scale_Pitch * -1.0),
                                (double)(Del_Control[3] * CLAW_P.BSC_Scale_Yaw),
                                (double)(Del_Control[0] * CLAW_P.BSC_Scale_Thrust),
                                // CLAW 출력이 실제로 믹서를 몰고 있는가.
                                // TDCN_CLAW_ON_OFF 를 켰어도 state / home_init /
                                // 비행 여부 조건이 안 맞으면 0 이다.
                                (uint8_t)(claw_output_active() ? 1 : 0));

// @LoggerMessage: TDCE
// @Description: TDCN CLAW controller internal state
// @Field: TimeUS: Time since system startup
// @Field: EN: CLAW position error North, leash clamped
// @Field: EE: CLAW position error East, leash clamped
// @Field: ED: CLAW position error Down
// @Field: I0: CLAW position error integrator North
// @Field: I1: CLAW position error integrator East
// @Field: I2: CLAW velocity error integrator lateral
// @Field: I3: CLAW velocity error integrator forward
// @Field: DN: CLAW velocity command North, before body rotation
// @Field: DE: CLAW velocity command East, before body rotation
// @Field: UU: CLAW desired body forward velocity
// @Field: UV: CLAW desired body lateral velocity
// @Field: XZ: CLAW trajectory generator height target
    AP::logger().WriteStreaming("TDCE",
                                "TimeUS,EN,EE,ED,I0,I1,I2,I3,DN,DE,UU,UV,XZ",
                                "Qffffffffffff",
                                now_us,
                                (double)Err_N,
                                (double)Err_E,
                                (double)Err_D,
                                (double)TV_BSC[0],
                                (double)TV_BSC[1],
                                (double)TV_BSC[2],
                                (double)TV_BSC[3],
                                (double)pos_dot_des[0],
                                (double)pos_dot_des[1],
                                (double)uv_des[0],
                                (double)uv_des[1],
                                (double)Xtraj[0]);
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

// ---------------------------------------------------------------------------
// CLAW 출력을 믹서에 넣어도 되는 상태인가
//
// 아래를 모두 만족해야 한다.  하나라도 어긋나면 아두파일럿이 계속 몬다.
//
//   TDCN_CLAW_ON_OFF == 1   조작자가 명시적으로 켰는가
//   state 6                 CLAW 가 도는 유일한 state.  다른 state 에서는
//                           CLAW_Y 가 갱신되지 않아 낡은 값이 나간다
//   home_init               CLAW 가 home 을 잡기 전에는 STV[9..11] 이 0 이라
//                           오차가 통째로 틀리다
//   비행 중                 지상에서 넣으면 make_safe_ground_handling() 과
//                           싸우게 된다
// ---------------------------------------------------------------------------

bool ModeTDCN::claw_output_active() const
{
    return _claw_on_off == 1
           && _state == State::TRACKING
           && home_init
           && motors->armed()
           && !is_disarmed_or_landed();
}

// ---------------------------------------------------------------------------
// 믹서 직전 훅
//
// Copter 의 fast loop 는 run_rate_controller() 로 제어값을 만들어 motors 에
// 넣어두고, 그 뒤 motors_output() 이 그 값으로 모터를 돌린다.  이 함수는
// motors_output() 이 flightmode->output_to_motors() 로 부르는 자리이므로,
// [계산은 끝났고 아직 모터로 안 나간] 시점이다.  여기서 갈아끼운다.
//
// run_rate_controller() 자체를 건너뛰지 않는 이유:
//   - 각속도 PID 의 적분항과 필터가 계속 갱신되어, CLAW 를 껐을 때 (또는
//     조건이 깨져 아두파일럿으로 돌아올 때) 튀지 않는다
//   - attitude_control 의 목표 자세도 살아 있어 다른 state 로 넘어갈 때
//     인수인계가 매끄럽다
//
// [피드포워드] motors 믹서는 (_roll_in + _roll_in_ff) 를 쓴다.  CLAW 값만
// 넣고 ff 를 두면 아두파일럿 몫이 섞이므로 ff 는 0 으로 지운다.
//
// [스로틀] 범위만 맞춘다.  CLAW 는 -1 ~ +1, 믹서는 0 ~ 1 이므로
//
//     thr = (cmd_height + 1) / 2
//
//     cmd_height  -1  ->  0.0
//     cmd_height   0  ->  0.5
//     cmd_height  +1  ->  1.0
//
// [왜 호버 보정을 하지 않는가]
//
// 이 단계의 목적은 CLAW 를 있는 그대로 통합하는 것이다.  기준점 보정은 통합이
// 아니라 제어 설계의 문제이므로 CLAW 쪽에서 다룬다.  여기서 보정하면 CLAW 가
// 자기 출력이 기체에 어떻게 반영되는지 알 수 없게 되고, 게인을 판정할 때
// 통합 코드가 끼워 넣은 비선형이 섞인다.
//
// 그래서 로그 스크립트(TDCN/tdcn_log_compare.py Figure 3)와 환산이 같다.
// 스크립트가 CH 를 (x+1)/2 로 그리므로 그래프의 CLAW 선이 곧 믹서 입력이다.
//
// [알려진 영향]  CLAW 의 0 은 호버를 뜻한다.  아두파일럿이 고도를 잡고 있던
// 7101 샘플에서 cmd_height 평균이 +0.0009 (표준편차 0.018) 였고 같은 구간
// 실제 스로틀은 0.332 였다.  즉 cmd_height 는 절대 스로틀이 아니라 호버 기준
// 증분이다.  (x+1)/2 는 그 0 을 0.5 로 보내므로, 호버가 0.5 가 아닌 기체에서는
// 인계 순간 (0.5 - hover) 만큼의 스로틀 계단이 들어간다.  호버 0.334 기체에서
// +0.17 (호버 대비 +50%) 이 주입돼 튀어오른 것을 확인했다.
//
// 이 계단은 통합 검증 단계에서 감수한다.  고도 거동이 문제가 되면 그때
// 기준점 보정을 CLAW 쪽 또는 이 자리에서 다시 검토한다.
// ---------------------------------------------------------------------------

void ModeTDCN::output_to_motors()
{
    // 아두파일럿 값을 먼저 잡아둔다.  아래에서 CLAW 값으로 덮어쓰면 이 값은
    // 어디에도 남지 않는다.  TDCC 로그의 MR/MP/MY/MT 가 이것을 쓴다.
    //
    // 롤/피치/요는 get_roll() 등이 _roll_in 을 그대로 주므로 이 시점 값이
    // 곧 run_rate_controller() 가 방금 넣은 아두파일럿 출력이다.
    //
    // 스로틀은 motors->get_throttle() 을 쓸 수 없다.  그것은 _throttle_filter
    // 값이고 필터는 AP_MotorsMulticopter::output() 안에서 갱신되는데, output()
    // 은 아래 Mode::output_to_motors() 에서 불린다.  즉 이 시점의 필터값은
    // [직전 루프에 CLAW 값으로 갱신된 것] 이다.  아두파일럿 자기 요구값은
    // attitude_control 이 들고 있다 (set_throttle_out 이 넣은 _throttle_in,
    // angle boost 적용 전).  이 값은 pos_control->update_z_controller() 가
    // 갱신하고 그것은 Run_CLAW() 뒤에 오므로 한 루프 늦다.
    _ap_roll_out     = motors->get_roll()  + motors->get_roll_ff();
    _ap_pitch_out    = motors->get_pitch() + motors->get_pitch_ff();
    _ap_yaw_out      = motors->get_yaw()   + motors->get_yaw_ff();
    _ap_throttle_out = attitude_control->get_throttle_in();

    if (claw_output_active()) {
        motors->set_roll(constrain_float(CLAW_Y.v_cmd.cmd_roll,  -1.0f, 1.0f));
        motors->set_pitch(constrain_float(CLAW_Y.v_cmd.cmd_pitch, -1.0f, 1.0f));
        motors->set_yaw(constrain_float(CLAW_Y.v_cmd.cmd_yaw,   -1.0f, 1.0f));

        // 범위만 맞춘다: -1 ~ +1  ->  0 ~ 1  (호버 보정 없음, 위 주석 참조)
        const float ch = constrain_float(CLAW_Y.v_cmd.cmd_height, -1.0f, 1.0f);
        motors->set_throttle(constrain_float((ch + 1.0f) * 0.5f, 0.0f, 1.0f));

        // 아두파일럿 각속도 PID 의 피드포워드가 더해지지 않게 지운다
        motors->set_roll_ff(0.0f);
        motors->set_pitch_ff(0.0f);
        motors->set_yaw_ff(0.0f);
    }

    Mode::output_to_motors();
}

void ModeTDCN::Run_CLAW()
{
    claw_gains.apply();         // 파라미터 -> CLAW_P (바꾸면 즉시 반영된다)
    Update_Info_for_CLAW();     // CLAW_U 에 기체 정보 + GCS 타겟 전달

    // --- 넘긴 값 스냅샷 ---
    //
    // "TDCN 이 넘긴 값" 과 "CLAW 가 내부에서 인식한 값" 을 같은 스텝끼리 비교
    // 하기 위한 것이다.  CLAW_step() 은 home 래치 스텝에서 Dest_poti_i 를
    // 스스로 덮어쓰므로, 호출 뒤에 읽으면 넘긴 값이 아니게 된다.
    _in_cur_lat   = CLAW_U.Cur_Pos.x;
    _in_cur_lng   = CLAW_U.Cur_Pos.y;
    _in_cur_alt   = CLAW_U.Cur_Pos.z;           // up 양수 (로그에서 뒤집는다)
    _in_dst_lat   = CLAW_U.Dest_poti_i.x;
    _in_dst_lng   = CLAW_U.Dest_poti_i.y;
    _in_dst_alt   = CLAW_U.Dest_poti_i.z;       // up 양수
    _in_vel_n     = XTV[0];
    _in_vel_e     = XTV[1];
    _in_vel_d     = XTV[2];
    _in_p         = CLAW_U.p;
    _in_q         = CLAW_U.q;
    _in_r         = CLAW_U.r;
    _in_roll      = CLAW_U.Roll;
    _in_pitch     = CLAW_U.Pitch;
    _in_yaw       = CLAW_U.DR_heading_f.DR_heading;
    _in_ship_hdg  = CLAW_U.Ship_heading;

    // CLAW 가 이번 스텝에 home 을 래치하는지 미리 봐 둔다.  래치 직후에만
    // 손볼 것이 있어서다 (아래 참조).
    const bool home_was_init = home_init;

    CLAW_step();                // CLAW 실행 (CLAW.c 의 함수 직접 호출)

    // --- 래치 직후 요 궤적 초기화 ---
    //
    // CLAW 는 home 을 잡는 스텝에서 궤적 생성기를 전부 0 으로 리셋한다
    // (CLAW.c 의 for 문, Xtraj/dXtraj/ddXtraj/TV_BSC).  고도(Xtraj[0]) 와
    // 롤/피치(Xtraj[1..2]) 는 0 이 맞다 - 래치 시점의 상대고도가 0 이고,
    // 호버 중이면 자세도 수평이라 오차가 없다.
    //
    // 그런데 Xtraj[3] 은 요 목표이고 0 은 [진북] 을 뜻한다.  기수가 북쪽이
    // 아닌 채로 state 6 에 들어가면 그 헤딩이 통째로 자세 오차가 된다:
    //     z1[3] = wrapToPi(STV[8] - Xtraj[3]) = 현재 헤딩
    // SITL 은 매번 진북(-1도) 으로 진입해 안 드러났지만, 실기체 로그에서는
    // 진입 헤딩이 121도였다.  그 상태로 CLAW 출력을 물리면 진입 즉시 최대
    // 요 명령이 나간다 (179도 오차에서 cmd_yaw 가 1.0 으로 포화하는 것을 확인).
    //
    // 그래서 래치 직후 요 궤적을 현재 헤딩에서 출발시킨다.  이러면 초기 오차가
    // 0 이고, 궤적 생성기가 목표 헤딩(ps_cmd) 으로 부드럽게 옮겨 간다.
    //
    // CLAW.c 를 고치지 않고 여기서 처리한다.  Xtraj 가 전역으로 노출되어 있어
    // 가능하고, vendor 파일 수정분을 늘리지 않는 편이 새 버전을 받을 때 낫다.
    if (!home_was_init && home_init) {
        Xtraj[3]  = STV[8];     // 요 목표 = 현재 헤딩
        dXtraj[3] = 0.0;        // 각속도도 0 에서 출발
    }

    Log_Write_TDCN();           // 스냅샷 + CLAW 내부값을 짝지어 기록
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


// 무장 후 이륙을 시작하기까지의 정착 대기 (ms).
//
// 무장하자마자 auto_takeoff 를 걸면, 모터가 아직 GROUND_IDLE 로 spool-up 하는
// 중인데 추력 명령이 올라가 튀어오르듯 이륙한다 (실기체에서 확인).  아두파일럿의
// spool-up 자체는 MOT_SPOOL_TIME (기본 0.5s) 인데, 그 뒤로 자세제어가 자리를
// 잡을 시간까지 보고 여유 있게 잡는다.
//
// 상한은 DISARM_DELAY (기본 10초) 다.  이보다 오래 기다리면 정착 대기 중에
// 자동 무장 해제가 걸려 다시 무장 -> 다시 대기가 반복된다.
static const uint32_t _takeoff_settle_ms = 2000;    // 2 초

void ModeTDCN::state_launch()           // 4 이륙 사출
{
    const uint32_t now_ms = AP_HAL::millis();

    if (_state_entered) {
        _takeoff_started = false;
    }

    // --- 1. 무장 상태를 "유지" 한다 ---
    //
    // state 3 은 지상에서 DISARM_DELAY (기본 10초) 로 자동 무장 해제가 반복되고
    // 그때마다 state_armed() 가 다시 무장한다.  하필 그 무장이 풀린 순간에
    // 4번이 들어오면 이 state 가 무장 없이 시작된다.  auto_takeoff.run() 은
    // 무장이 없으면 아무 것도 못 하고, state 4 에는 회복 수단이 없으므로
    // 이륙도 못 하고 다음 번호로도 못 넘어간 채 영구히 멈춘다.
    // 그래서 이륙이 끝나기 전에 무장이 풀려 있으면 다시 무장한다.
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

    // --- 2. 무장 직후에는 곧바로 이륙하지 않는다 ---
    //
    // 무장 시각은 run() 이 엣지로 잡아둔 _armed_ms 다.  state 3 에서 이미
    // 무장돼 시간이 지났으면 이 조건에 걸리지 않고 바로 이륙하므로, 정상
    // 시나리오에는 지연이 생기지 않는다.  여기서 걸리는 것은 "방금 무장한"
    // 경우뿐이다 (state 3 을 건너뛰었거나 자동 해제 후 재무장).
    //
    // 대기 중에는 GROUND_IDLE 을 유지한다.  is_disarmed_or_landed() 로 감싸는
    // 이유는 공중에서 이 state 에 들어온 경우에 지상 처리를 하면 추락하기
    // 때문이다.
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

    // --- CLAW 가 모는 동안 아두파일럿 제어기를 현재 상태에 붙여 둔다 ---
    //
    // state 7 로 넘어가면 아두파일럿이 기체를 다시 잡는데, 그때 제어기 내부
    // 상태가 낡아 있으면 인계 순간 자세와 목표가 어긋나 튄다.  CLAW 가 모는
    // 동안에는 기체가 아두파일럿 명령을 따르지 않으므로 그 어긋남이 계속 커진다.
    //
    // 그래서 매 루프 목표를 현재 자세로 되맞추고 적분항을 비운다.  인계 시점에
    // 이미 목표 = 실제 이므로 state 7 은 "정지점 계산" 만 하면 된다.
    //
    // reset_rate 를 false 로 두는 이유는 각속도 제어기를 계속 돌게 두기
    // 위해서다.  그래야 PID 필터 상태가 살아 있어 되돌아올 때 부드럽다.
    //
    // 위 input_thrust_vector_heading() 을 그대로 두는 것은 의도다.  아두파일럿이
    // "같은 상황에서 무엇을 명령했을지" 가 로그에 남아야 두 제어기를 비교할 수
    // 있다 (v1 의 목적).  여기서는 그 결과만 기체에 반영되지 않게 덮는다.
    if (claw_output_active()) {
        attitude_control->reset_target_and_rate(false);
        attitude_control->reset_rate_controller_I_terms();
    }
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

    // 목표 고도를 EKF origin 기준 NEU cm 로 바꿔 Z 만 갈아끼운다.
    // _land_alt 은 home 기준이므로 현재 위치를 기준점으로 삼아 변환한다.
    // (매 루프 계산하므로 home 이 갱신되어도 따라간다)
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

// ---------------------------------------------------------------------------
// state 9 (착륙 수납) - 현재 자리에 착륙
//
// [기반 모드] LAND (GPS 있는 경우)
//   초기화  ModeLand::init() 과 동일
//   제어    Mode::land_run_horiz_and_vert_control()  (정밀 착륙은 쓰지 않는다)
//
// 착륙 로직을 새로 짜지 않고 아두파일럿 공용 함수를 그대로 호출한다.  그래야
// 착지 감지(land detector), 지면 효과 보정, 조종자 재위치(land_repo) 같은
// 검증된 처리가 전부 따라온다.
//
// [정밀 착륙] 쓰지 않는다.  제자리 착륙만 하면 되고, PLND_* 파라미터에 따라
// 동작이 갈리면 안 되기 때문이다.  아래 land_run_horiz_and_vert_control() 주석 참조.
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
//   _land_alt 을 LAND_ALT_LOW 보다 높게 잡고 LAND_SPEED_HIGH 를 설정한다.
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
    //
    // land_run_normal_or_precland() 가 아니라 일반 착륙을 직접 부른다.
    // 그 함수는 PLND_ENABLED 가 켜져 있으면 정밀 착륙으로 분기하는데,
    // TDCN 은 제자리 착륙만 하면 되므로 파라미터에 따라 동작이 갈리면 안 된다.
    //
    // 실제로 PLND_ENABLED=1, PLND_TYPE=3(SITL_Gazebo) 인 기체에서 타겟을 못 찾아
    // "PrecLand: Failsafe Measures" 가 뜨고 명령대로 내려오지 않는 것을 확인했다.
    // (정밀 착륙 상태기계가 재시도 4회 후 failsafe 로 빠져 제자리 정지 -> 수직
    //  하강으로 자기 판단하에 동작한다)
    //
    // 선박 상대 착륙은 CLAW 가 담당할 부분이라, 아두파일럿 정밀 착륙을 함께
    // 쓰면 선박 추종 제어기가 둘이 된다.  그것도 여기서 쓰지 않는 이유다.
    land_run_horiz_and_vert_control();
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