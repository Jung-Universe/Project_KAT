#include "Copter.h"

#if MODE_TDCN_ENABLED

#include "mode_tdcn_claw_extern.h"     // CLAW_P

// ===========================================================================
// Part 2. 게인 (CLAW_*)    GCS 에서 수정 / apply() 가 CLAW_P 로 복사
//
// 기본값은 mode_tdcn_CLAW_data_0729.c 의 CLAW_P 초기값을 그대로 쓴다.  데이터 파일을
// 고치고 빌드하면 그 값이 곧 CLAW_* 기본값이다 (표의 기본값은 _def_* 를 가리킨다).
// 단, 기체에 CLAW_* 값이 저장돼 있으면 (GCS 에서 바꾼 적이 있으면) 저장값이 우선한다.
//
// AP_GROUPINFO 의 번호는 기체에 저장된 값을 찾는 키다.  순서를 옮기거나 번호를
// 다시 매기면 저장값과 어긋나므로, 새 항목은 빈 번호를 뒤에 붙인다.
// ===========================================================================

// ---------------------------------------------------------------------------
/* CLAW gains parameters */
const AP_Param::GroupInfo CLAW_Gains::var_info[] = {

    // @Param: SCALE_TH
    // @DisplayName: CLAW thrust output scale
    // @Description: Scales the backstepping thrust output into the normalised command range
    // @Range: 0.1 10
    // @User: Advanced
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("SCALE_TH", 1, CLAW_Gains, _scale_th, _def_scale_th),

    // @Param: SCALE_R
    // @DisplayName: CLAW roll output scale
    // @Description: Scales the backstepping roll output into the normalised command range
    // @Range: 0.01 2
    // @User: Advanced
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("SCALE_R", 2, CLAW_Gains, _scale_r, _def_scale_r),

    // @Param: SCALE_P
    // @DisplayName: CLAW pitch output scale
    // @Description: Scales the backstepping pitch output into the normalised command range
    // @Range: 0.01 2
    // @User: Advanced
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("SCALE_P", 3, CLAW_Gains, _scale_p, _def_scale_p),

    // @Param: SCALE_Y
    // @DisplayName: CLAW yaw output scale
    // @Description: Scales the backstepping yaw output into the normalised command range
    // @Range: 0.01 2
    // @User: Advanced
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("SCALE_Y", 4, CLAW_Gains, _scale_y, _def_scale_y),

    // @Param: K_POS_P
    // @DisplayName: CLAW outer loop position P
    // @Description: Position error to velocity command gain
    // @Range: 0 2
    // @User: Advanced
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("K_POS_P", 5, CLAW_Gains, _k_pos_p, _def_k_pos_p),

    // @Param: K_POS_I
    // @DisplayName: CLAW outer loop position I
    // @Description: Position error integral gain
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("K_POS_I", 6, CLAW_Gains, _k_pos_i, _def_k_pos_i),

    // @Param: K_VEL_P
    // @DisplayName: CLAW outer loop velocity P
    // @Description: Velocity error to attitude command gain
    // @Range: 0 2
    // @User: Advanced
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("K_VEL_P", 7, CLAW_Gains, _k_vel_p, _def_k_vel_p),

    // @Param: K_VEL_I
    // @DisplayName: CLAW outer loop velocity I
    // @Description: Velocity error integral gain
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("K_VEL_I", 8, CLAW_Gains, _k_vel_i, _def_k_vel_i),

    // @Param: AWU_LIMIT
    // @DisplayName: CLAW integrator limit
    // @Description: Anti windup clamp applied to all four CLAW integrators
    // @Range: 0.1 10
    // @User: Advanced
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("AWU_LIMIT", 9, CLAW_Gains, _awu_limit, _def_awu_limit),

    // @Param: OMEGA_XX
    // @DisplayName: CLAW trajectory natural frequency North
    // @Description: Natural frequency of the North axis trajectory filter
    // @Units: rad/s
    // @Range: 0.05 20
    // @User: Advanced
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("OMEGA_XX", 10, CLAW_Gains, _ome_xx, _def_ome_xx),

    // @Param: OMEGA_YY
    // @DisplayName: CLAW trajectory natural frequency East
    // @Description: Natural frequency of the East axis trajectory filter
    // @Units: rad/s
    // @Range: 0.05 20
    // @User: Advanced
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("OMEGA_YY", 11, CLAW_Gains, _ome_yy, _def_ome_yy),

    // @Param: OMEGA_ZZ
    // @DisplayName: CLAW trajectory natural frequency height
    // @Description: Natural frequency of the height trajectory filter
    // @Units: rad/s
    // @Range: 0.05 20
    // @User: Advanced
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("OMEGA_ZZ", 12, CLAW_Gains, _ome_zz, _def_ome_zz),

    // @Param: OMEGA_PH
    // @DisplayName: CLAW trajectory natural frequency roll
    // @Description: Natural frequency of the roll trajectory filter
    // @Units: rad/s
    // @Range: 0.05 30
    // @User: Advanced
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("OMEGA_PH", 13, CLAW_Gains, _ome_ph, _def_ome_ph),

    // @Param: OMEGA_TH
    // @DisplayName: CLAW trajectory natural frequency pitch
    // @Description: Natural frequency of the pitch trajectory filter
    // @Units: rad/s
    // @Range: 0.05 30
    // @User: Advanced
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("OMEGA_TH", 14, CLAW_Gains, _ome_th, _def_ome_th),

    // @Param: OMEGA_PS
    // @DisplayName: CLAW trajectory natural frequency yaw
    // @Description: Natural frequency of the yaw trajectory filter
    // @Units: rad/s
    // @Range: 0.05 30
    // @User: Advanced
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("OMEGA_PS", 15, CLAW_Gains, _ome_ps, _def_ome_ps),

    // @Param: ZETA_XX
    // @DisplayName: CLAW trajectory damping North
    // @Description: Damping ratio of the North axis trajectory filter
    // @Range: 0.1 2
    // @User: Advanced
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("ZETA_XX", 16, CLAW_Gains, _zeta_xx, _def_zeta_xx),

    // @Param: ZETA_YY
    // @DisplayName: CLAW trajectory damping East
    // @Description: Damping ratio of the East axis trajectory filter
    // @Range: 0.1 2
    // @User: Advanced
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("ZETA_YY", 17, CLAW_Gains, _zeta_yy, _def_zeta_yy),

    // @Param: ZETA_ZZ
    // @DisplayName: CLAW trajectory damping height
    // @Description: Damping ratio of the height trajectory filter
    // @Range: 0.1 2
    // @User: Advanced
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("ZETA_ZZ", 18, CLAW_Gains, _zeta_zz, _def_zeta_zz),

    // @Param: ZETA_PH
    // @DisplayName: CLAW trajectory damping roll
    // @Description: Damping ratio of the roll trajectory filter
    // @Range: 0.1 2
    // @User: Advanced
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("ZETA_PH", 19, CLAW_Gains, _zeta_ph, _def_zeta_ph),

    // @Param: ZETA_TH
    // @DisplayName: CLAW trajectory damping pitch
    // @Description: Damping ratio of the pitch trajectory filter
    // @Range: 0.1 2
    // @User: Advanced
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("ZETA_TH", 20, CLAW_Gains, _zeta_th, _def_zeta_th),

    // @Param: ZETA_PS
    // @DisplayName: CLAW trajectory damping yaw
    // @Description: Damping ratio of the yaw trajectory filter
    // @Range: 0.1 2
    // @User: Advanced
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("ZETA_PS", 21, CLAW_Gains, _zeta_ps, _def_zeta_ps),

    // @Param: TAU_HDOT
    // @DisplayName: CLAW climb rate time constant
    // @Description: Time constant of the climb rate channel
    // @Units: s
    // @Range: 0.01 2
    // @User: Advanced
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("TAU_HDOT", 22, CLAW_Gains, _tau_hdot, _def_tau_hdot),

    // @Param: TAU_R
    // @DisplayName: CLAW yaw rate time constant
    // @Description: Time constant of the yaw rate channel
    // @Units: s
    // @Range: 0.01 2
    // @User: Advanced
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("TAU_R", 23, CLAW_Gains, _tau_r, _def_tau_r),

    AP_GROUPEND
};
// ---------------------------------------------------------------------------

// 기본값 = mode_tdcn_CLAW_data_0729.c 의 CLAW_P 초기값.
//
// CLAW_P 는 상수로만 초기화되는 C 전역이라 이 생성자보다 먼저 값이 들어 있다
// (정적 초기화).  여기서 복사해 두므로, 비행 중 apply() 가 CLAW_P 를 덮어써도
// 기본값은 데이터 파일 값 그대로다.
CLAW_Gains::CLAW_Gains(void) :
    _def_scale_th((float)CLAW_P.BSC_Scale_Thrust),
    _def_scale_r((float)CLAW_P.BSC_Scale_Roll),
    _def_scale_p((float)CLAW_P.BSC_Scale_Pitch),
    _def_scale_y((float)CLAW_P.BSC_Scale_Yaw),
    _def_k_pos_p((float)CLAW_P.BSC_K_POS_P),
    _def_k_pos_i((float)CLAW_P.BSC_K_POS_I),
    _def_k_vel_p((float)CLAW_P.BSC_K_VEL_P),
    _def_k_vel_i((float)CLAW_P.BSC_K_VEL_I),
    _def_awu_limit((float)CLAW_P.BSC_Int_Limit),
    _def_ome_xx((float)CLAW_P.BSC_Ome_XX),
    _def_ome_yy((float)CLAW_P.BSC_Ome_YY),
    _def_ome_zz((float)CLAW_P.BSC_Ome_ZZ),
    _def_ome_ph((float)CLAW_P.BSC_Ome_PH),
    _def_ome_th((float)CLAW_P.BSC_Ome_TH),
    _def_ome_ps((float)CLAW_P.BSC_Ome_PS),
    _def_zeta_xx((float)CLAW_P.BSC_Zeta_XX),
    _def_zeta_yy((float)CLAW_P.BSC_Zeta_YY),
    _def_zeta_zz((float)CLAW_P.BSC_Zeta_ZZ),
    _def_zeta_ph((float)CLAW_P.BSC_Zeta_PH),
    _def_zeta_th((float)CLAW_P.BSC_Zeta_TH),
    _def_zeta_ps((float)CLAW_P.BSC_Zeta_PS),
    _def_tau_hdot((float)CLAW_P.BSC_Tau_hdot),
    _def_tau_r((float)CLAW_P.BSC_Tau_r)
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
}

#endif  // MODE_TDCN_ENABLED
