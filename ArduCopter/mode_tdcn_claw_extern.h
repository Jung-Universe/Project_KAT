#pragma once

// ---------------------------------------------------------------------------
/* mode_tdcn_CLAW_IBSC.ship_Final_NED.c를 함수처럼 사용하기 위해 extern "C"로 묶음 */
/* mode_tdcn.cpp 와 mode_tdcn_gain.cpp 가 함께 쓴다.
   Arming 은 선언이 아니라 정의라서 여기 두지 않는다 (mode_tdcn.cpp 에만 있다) */
extern "C" {
#include "mode_tdcn_CLAW.h"

extern bool home_init;

extern double XTV[6];
}
// ---------------------------------------------------------------------------
