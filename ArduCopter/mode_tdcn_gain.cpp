#include "Copter.h"

#if MODE_TDCN_ENABLED

#include "mode_tdcn_claw_extern.h"     // CLAW_P

// ===========================================================================
// Part 2. 게인 (CLAW_*)    GCS 에서 수정 / apply() 가 CLAW_P 로 복사
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
// ---------------------------------------------------------------------------

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
}

#endif  // MODE_TDCN_ENABLED
