#include "Copter.h"

#if MODE_TDCN_ENABLED

// ===========================================================================
// Part 3. 로그 (TDIM / TDST / TDTG / TDMX)    bin 로그 / TDCN_LOG_HZ 주기
//
//   TDIM  IMU 6축          gyro (rad/s), accel (m/s/s)     주 IMU, body FRD
//   TDST  모드 / state / 현재값   위치 / 속도 (NED m), 자세 (deg)
//   TDTG  목표값           위치 (NED m), 헤딩 (deg)
//   TDMX  믹서 입력        아두파일럿 / CLAW 의 roll, pitch, yaw (-1~1), throttle (0~1)
//
// 전원 인가부터 비행 모드와 관계없이 남긴다.  모드 재진입 때 init() 이 값을
// 초기화하는 것까지 보기 위해서다.  그래서 모드의 run() / output_to_motors() 가
// 아니라 스케줄러 fast task (Copter.cpp, motors_output 바로 다음) 가 부른다.
//
// 그 시점의 값은
//   TDIM       이번 루프 IMU 샘플 (아두파일럿 rate 제어가 방금 쓴 값).  항상 실시간
//   TDST/TDTG  직전 run() 의 update_status() 가 채운 _status
//   TDMX       아두파일럿 = output_to_motors() 가 대체 직전에 잡은 값
//              CLAW       = update_status() 가 믹서용으로 만든 값 (state 6 밖에서는 0)
// TDIM 과 TDST 의 Mode 말고는 모두 TDCN 이 가진 값 그대로다.  TDCN 이 아닌 모드
// 에서는 0 이고 (처음 진입 전에는 원래 0, 이탈하면 exit() 가 지운다), 진입하면
// init() 이 지운 값에서 다시 채워진다.  0 은 원점 / state 0 과도 같으므로 TDCN 이
// 도는 구간은 Mode 가 29 인 곳으로 가린다.
// Act 가 1 인 구간만 CLAW 값이 실제로 믹서에 들어간다.
//
// 위치는 _status 와 같은 EKF origin 기준이고, NEU cm 를 NED m 로 바꿔 남긴다
// (PSCN / PSCE / PSCD 와 같은 기준이라 겹쳐 볼 수 있다).
//
// 아두파일럿은 기본 (LOG_DISARMED = 0) 으로 무장 중에만 로그를 쓴다.  전원 인가
// 부터 남기려고 TDCN_LOG_HZ 가 0 이 아니면 LOG_DISARMED 와 관계없이 무장 해제
// 중에도 기록한다 (이 기체의 로그 전체가 LOG_DISARMED = 1 처럼 된다).
// ===========================================================================

// ---------------------------------------------------------------------------
/* TDCN 로그 기록.  fast task 가 매 루프 부른다 (motors_output 다음) */
void ModeTDCN::Log_Write_TDCN()
{
#if HAL_LOGGING_ENABLED
    AP_Logger &logger = AP::logger();

    // TDCN_LOG_HZ 가 0 이면 TDCN 로그도, 무장 해제 중 기록도 끈다
    logger.set_force_log_disarmed(_log_hz > 0);
    if (_log_hz <= 0) {
        return;
    }

    // 메인 루프 (SCHED_LOOP_RATE) 를 정수로 나눠 주기를 맞춘다.  400Hz 루프에서 400 이면 매 루프
    uint16_t div = AP::scheduler().get_loop_rate_hz() / (uint16_t)_log_hz.get();
    if (div < 1) {
        div = 1;
    }
    if (++_log_count < div) {
        return;
    }
    _log_count = 0;

    const uint64_t now_us = AP_HAL::micros64();
    const TdcnStatus &s = _status;

    // --- IMU 6축 ---
    const Vector3f &gyro  = AP::ins().get_gyro();
    const Vector3f &accel = AP::ins().get_accel();

// @LoggerMessage: TDIM
// @Description: TDCN IMU, primary sensor, body frame
// @Field: TimeUS: Time since system startup
// @Field: GyrX: Roll rate
// @Field: GyrY: Pitch rate
// @Field: GyrZ: Yaw rate
// @Field: AccX: Forward specific force
// @Field: AccY: Right specific force
// @Field: AccZ: Down specific force
    logger.WriteStreaming("TDIM",
                          "TimeUS,GyrX,GyrY,GyrZ,AccX,AccY,AccZ",
                          "sEEEooo",
                          "F000000",
                          "Qffffff",
                          now_us,
                          (double)gyro.x,
                          (double)gyro.y,
                          (double)gyro.z,
                          (double)accel.x,
                          (double)accel.y,
                          (double)accel.z);

    // --- 모드 / state / 현재값 ---
// @LoggerMessage: TDST
// @Description: TDCN state and current vehicle state. Except Mode, the values are held by TDCN: 0 outside TDCN mode (cleared on exit), reset by init on entry
// @Field: TimeUS: Time since system startup
// @Field: Mode: Current flight mode number, 29 while TDCN runs
// @Field: St: TDCN state, 0 to 13
// @Field: Stp: Scenario step being run, 0 to 11. Differs from St only during auto sequence 12 or 13
// @Field: PN: Position North from EKF origin
// @Field: PE: Position East from EKF origin
// @Field: PD: Position Down from EKF origin
// @Field: VN: Velocity North
// @Field: VE: Velocity East
// @Field: VD: Velocity Down
// @Field: Roll: Roll angle
// @Field: Pitch: Pitch angle
// @Field: Yaw: Yaw angle, true north
    logger.WriteStreaming("TDST",
                          "TimeUS,Mode,St,Stp,PN,PE,PD,VN,VE,VD,Roll,Pitch,Yaw",
                          "s---mmmnnnddh",
                          "F---000000000",
                          "QBBBfffffffff",
                          now_us,
                          (uint8_t)copter.flightmode->mode_number(),
                          (uint8_t)_state,
                          (uint8_t)scenario_state(),
                          (double)(s.pos_neu_cm.x * 0.01),
                          (double)(s.pos_neu_cm.y * 0.01),
                          (double)(-s.pos_neu_cm.z * 0.01),     // up -> down
                          (double)(s.vel_neu_cms.x * 0.01f),
                          (double)(s.vel_neu_cms.y * 0.01f),
                          (double)(-s.vel_neu_cms.z * 0.01f),   // up -> down
                          (double)degrees(s.euler_rad.x),
                          (double)degrees(s.euler_rad.y),
                          (double)wrap_360(degrees(s.euler_rad.z)));

    // --- 목표값 ---
// @LoggerMessage: TDTG
// @Description: TDCN target the current state is tracking
// @Field: TimeUS: Time since system startup
// @Field: Val: 1 when the current state has a position target. 0 during ground handling, where PN PE PD are stale
// @Field: PN: Target position North from EKF origin
// @Field: PE: Target position East from EKF origin
// @Field: PD: Target position Down from EKF origin
// @Field: Hdg: Target heading, true north. GCS heading in state 6, attitude controller yaw target otherwise
    logger.WriteStreaming("TDTG",
                          "TimeUS,Val,PN,PE,PD,Hdg",
                          "s-mmmh",
                          "F-0000",
                          "QBffff",
                          now_us,
                          (uint8_t)(s.target_valid ? 1 : 0),
                          (double)(s.target_pos_neu_cm.x * 0.01),
                          (double)(s.target_pos_neu_cm.y * 0.01),
                          (double)(-s.target_pos_neu_cm.z * 0.01),  // up -> down
                          (double)wrap_360(s.target_heading_deg));

    // --- 믹서 입력 (아두파일럿 / CLAW) ---
// @LoggerMessage: TDMX
// @Description: TDCN mixer input, ArduPilot vs CLAW
// @Field: TimeUS: Time since system startup
// @Field: Act: 1 when the CLAW values replace the ArduPilot values in the mixer
// @Field: AR: ArduPilot roll input, rate PID plus feedforward
// @Field: AP: ArduPilot pitch input, rate PID plus feedforward
// @Field: AY: ArduPilot yaw input, rate PID plus feedforward
// @Field: AT: ArduPilot throttle input, before angle boost
// @Field: CR: CLAW roll input
// @Field: CP: CLAW pitch input
// @Field: CY: CLAW yaw input
// @Field: CT: CLAW throttle input
    logger.WriteStreaming("TDMX",
                          "TimeUS,Act,AR,AP,AY,AT,CR,CP,CY,CT",
                          "s---------",
                          "F-00000000",
                          "QBffffffff",
                          now_us,
                          (uint8_t)(s.claw_active ? 1 : 0),
                          (double)s.ap_roll,
                          (double)s.ap_pitch,
                          (double)s.ap_yaw,
                          (double)s.ap_throttle,
                          (double)s.claw_roll,
                          (double)s.claw_pitch,
                          (double)s.claw_yaw,
                          (double)s.claw_throttle);
#endif  // HAL_LOGGING_ENABLED
}
// ---------------------------------------------------------------------------

#endif  // MODE_TDCN_ENABLED
