#include "Copter.h"

#if MODE_TDCN_ENABLED

// ===========================================================================
// Part 1. 파라미터 (TDCN_*)    GCS 에서 수정 / 시나리오 동작값
//
// AP_GROUPINFO 의 번호는 기체에 저장된 값을 찾는 키다.  순서를 옮기거나 번호를
// 다시 매기면 저장값과 어긋나므로, 새 항목은 빈 번호를 뒤에 붙인다.
// ===========================================================================

// ---------------------------------------------------------------------------
/* GCS에서 직접 수정할 수 있는 파라미터 목록 */
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

    // @Param: AUTO_DWELL
    // @DisplayName: TDCN auto sequence dwell time
    // @Description: Used by the GCS auto sequence commands 12 (states 1 to 6) and 13 (states 6 to 11). Each state must stay complete for this long before the sequence moves on to the next state, so that transitions such as arming to takeoff do not happen the instant a state completes. The timer restarts if the state stops being complete. Manual state commands are not affected.
    // @Units: s
    // @Range: 0 30
    // @Increment: 0.5
    // @User: Standard
    AP_GROUPINFO("AUTO_DWELL", 6, ModeTDCN, _auto_dwell, 3.0f),

    AP_GROUPEND
};
// ---------------------------------------------------------------------------

ModeTDCN::ModeTDCN(void) : Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

#endif  // MODE_TDCN_ENABLED
