/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: CLAW.c
 *
 * Code generated for Simulink model 'CLAW'.
 *
 * Model version                  : 1.138
 * Simulink Coder version         : 23.2 (R2023b) 01-Aug-2023
 * C/C++ source code generated on : Tue Mar 26 18:06:28 2024
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: STMicroelectronics->ST10/Super10
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "CLAW.h"
#include "rtwtypes.h"
#include "CLAW_types.h"
#include "CLAW_private.h"
#include <math.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

extern volatile uint8_t Arming;

// =========================================================
// [BSC 제어 변수 정의] - 내부 연산용 double 사용
// =========================================================
#define DT           0.0025   // 제어 주기 (400Hz 가정)
#define D2R          0.017453292519943295

#ifndef M_PI
#define M_PI         3.14159265358979323846
#endif

// 상태 변수 및 제어 입력
double STV[12];         // State Vector [u,v,w, p,q,r, phi,theta,psi, x,y,z]
double XTV[6];          // Inertial Vector [vx,vy,vz, wx,wy,wz]
double Del_Control[4];  // Control Input (Thrust, Roll, Pitch, Yaw)

// 선박 목표 heading 각 전역 변수 정의
double ps_cmd = 0.0;

// 내부 연산 변수 (Trim 값 및 궤적)
double STV_trim[12] = { 0.0, };
double pos_des_dot_trim[3] = { 0.0, };
double Xtraj[4] = { 0 }, dXtraj[4] = { 0 }, ddXtraj[4] = { 0 };
double TV_BSC[4] = { 0 };

// IBSC 전용 전역 변수
double Est_Acc_Body[6] = { 0.0, };
double Acc_0[4] = { 0.0, };
double Prev_CTRL[4] = { 0.0, 0.0, 0.0, 0.0 };
double STV_OLD[3] = { 0.0, };

// 임시 변수들
double Ph, Th, Ps, Sph, Cph, Sth, Cth, Sps, Cps;
double CTML[3][3], CTMA[3][3], CTML_inv[3][3], CTMA_inv[3][3];
double uv_des[3], pos_dot_des[3];
double z1[4], z2[4], alpha[4], alpha_dot[4], f[4];
double G_mat[4][4], G_mat_inv[4][4];

// [가속도 추정기용 버퍼]
#define BUFFER_SIZE 3 
static double state_buffer[4][BUFFER_SIZE];
static bool buffer_filled = false;

// IBSC 안정화를 위한 게인 및 필터 계수 추가
#define IBSC_DELTA_GAIN  0.1
static double Est_Acc_Body_Filtered[6] = { 0.0, };

// =========================================================
// [헬퍼 함수 정의]
// =========================================================

// Heading 각도를 [-pi, pi] 범위로 정규화하는 헬퍼 함수
static double wrapToPi(double angle) {
    while (angle > M_PI)  angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double Estimate_Accel_Hermite(int axis, double current_val, double dt) {
    state_buffer[axis][2] = state_buffer[axis][1];
    state_buffer[axis][1] = state_buffer[axis][0];
    state_buffer[axis][0] = current_val;

    if (!buffer_filled) {
        static int init_count = 0;
        if (axis == 3) init_count++;
        if (init_count > BUFFER_SIZE + 5) buffer_filled = true;
        return 0.0;
    }

    double deriv = (3.0 * state_buffer[axis][0]
        - 4.0 * state_buffer[axis][1]
        + 1.0 * state_buffer[axis][2]) / (2.0 * dt);
    return deriv;
}

double Lat2m(double lat) {
    double lat_rad = lat * D2R;
    return 111132.92
        - 559.82 * cos(2.0 * lat_rad)
        + 1.175 * cos(4.0 * lat_rad)
        - 0.0023 * cos(6.0 * lat_rad);
}

double Lon2m(double lat) {
    double lat_rad = lat * D2R;
    return 111412.84 * cos(lat_rad)
        - 93.5 * cos(3.0 * lat_rad)
        + 0.118 * cos(5.0 * lat_rad);
}

/* Block signals (default storage) */
B_CLAW_T CLAW_B;

/* Block states (default storage) */
DW_CLAW_T CLAW_DW;

/* External inputs (root inport signals with default storage) */
ExtU_CLAW_T CLAW_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_CLAW_T CLAW_Y;

static RT_MODEL_CLAW_T CLAW_M_;
RT_MODEL_CLAW_T* const CLAW_M = &CLAW_M_;

double Target_X = 0.0, Target_Y = 0.0, Target_Z = 0.0;
double Home_Lat = 0.0, Home_Lon = 0.0, Home_Alt = 0.0, Home_Yaw = 0.0;
static bool home_init = false;
double Err_N, Err_E, Err_D;

/* Model step function */
void CLAW_step(void)
{
    // Current Position (Latitude, Longitude, Altitude) 
    double Cur_Lat = (double)CLAW_U.Cur_Pos.x;
    double Cur_Lon = (double)CLAW_U.Cur_Pos.y;
    double Cur_Alt = -(double)CLAW_U.Cur_Pos.z;

    if (Arming == 0) {
        CLAW_Y.v_cmd.cmd_height = 0.0f;
        CLAW_Y.v_cmd.cmd_roll = 0.0f;
        CLAW_Y.v_cmd.cmd_pitch = 0.0f;
        CLAW_Y.v_cmd.cmd_yaw = 0.0f;

        ps_cmd = STV[8];

        for (int i = 0; i < 6; i++) Est_Acc_Body_Filtered[i] = 0.0;
        buffer_filled = false;
        return;
    }

    if (!home_init) {
        if (fabs(Cur_Lat) > 1.0 && fabs(Cur_Lon) > 1.0) {
            Home_Lat = Cur_Lat;
            Home_Lon = Cur_Lon;
            Home_Alt = Cur_Alt;
            Home_Yaw = (double)CLAW_U.Ship_heading;

            CLAW_U.Dest_poti_i.y = (real32_T)Cur_Lat;
            CLAW_U.Dest_poti_i.x = (real32_T)Cur_Lon;
            CLAW_U.Dest_poti_i.z = -(real32_T)Cur_Alt;

            CLAW_DW.UD_DSTATE = 0.0;
            CLAW_DW.UD_DSTATE_i = 0.0;
            CLAW_DW.UD_DSTATE_p = 0.0;

            buffer_filled = false;
            for (int k = 0; k < BUFFER_SIZE; k++) {
                state_buffer[1][k] = STV[3];
                state_buffer[2][k] = STV[4];
                state_buffer[3][k] = STV[5];
            }
            for (int k = 0; k < 6; k++) Est_Acc_Body_Filtered[k] = 0.0;

            for (int i = 0; i < 4; i++) {
                TV_BSC[i] = 0.0;
                Xtraj[i] = 0.0;
                dXtraj[i] = 0.0;
                ddXtraj[i] = 0.0;
                Prev_CTRL[i] = 0.0;
            }
            home_init = true;
        }
    }

    // Target Position (Latitude, Longiutude, Altitude)
    double Dest_Lat = (double)CLAW_U.Dest_poti_i.x;
    double Dest_Lon = (double)CLAW_U.Dest_poti_i.y;
    double Dest_Alt = -(double)CLAW_U.Dest_poti_i.z;

    if (home_init) {
        STV[9] = (Cur_Lat - Home_Lat) * Lat2m(Home_Lat);
        STV[10] = (Cur_Lon - Home_Lon) * Lon2m(Home_Lon);
        STV[11] = (Cur_Alt - Home_Alt);
    }
    else {
        STV[9] = 0.0;
        STV[10] = 0.0;
        STV[11] = 0.0;
    }

    // Current EKF Velocity (NED)
    XTV[0] = 0;
    XTV[1] = 0;
    XTV[2] = 0;

    CLAW_DW.UD_DSTATE = STV[9];
    CLAW_DW.UD_DSTATE_i = STV[10];
    CLAW_DW.UD_DSTATE_p = STV[11];

    // Current IMU - Euler Angle (Roll, Pitch)
    STV[6] = (double)CLAW_U.Roll;
    STV[7] = (double)CLAW_U.Pitch;

    // Current Heading ???
    STV[8] = wrapToPi((double)CLAW_U.DR_heading_f.DR_heading);

    // Current IMU - Gyroscope
    STV[3] = (double)CLAW_U.p;
    STV[4] = (double)CLAW_U.q;
    STV[5] = (double)CLAW_U.r;

    Ph = STV[6]; Th = STV[7]; Ps = STV[8];
    Sph = sin(Ph); Cph = cos(Ph);
    Sth = sin(Th); Cth = cos(Th);
    Sps = sin(Ps); Cps = cos(Ps);

    CTML[0][0] = Cth * Cps;                       CTML[0][1] = Cth * Sps;                       CTML[0][2] = -Sth;
    CTML[1][0] = Sph * Sth * Cps - Cph * Sps;     CTML[1][1] = Sph * Sth * Sps + Cph * Cps;     CTML[1][2] = Sph * Cth;
    CTML[2][0] = Cph * Sth * Cps + Sph * Sps;     CTML[2][1] = Cph * Sth * Sps - Sph * Cps;     CTML[2][2] = Cph * Cth;

    CTML_inv[0][0] = CTML[0][0]; CTML_inv[0][1] = CTML[1][0]; CTML_inv[0][2] = CTML[2][0];
    CTML_inv[1][0] = CTML[0][1]; CTML_inv[1][1] = CTML[1][1]; CTML_inv[1][2] = CTML[2][1];
    CTML_inv[2][0] = CTML[0][2]; CTML_inv[2][1] = CTML[1][2]; CTML_inv[2][2] = CTML[2][2];

    if (fabs(Cth) < 1e-6) Cth = 1e-6;
    CTMA_inv[0][0] = 1.0; CTMA_inv[0][1] = Sph * Sth / Cth; CTMA_inv[0][2] = Cph * Sth / Cth;
    CTMA_inv[1][0] = 0.0; CTMA_inv[1][1] = Cph;         CTMA_inv[1][2] = -Sph;
    CTMA_inv[2][0] = 0.0; CTMA_inv[2][1] = Sph / Cth;     CTMA_inv[2][2] = Cph / Cth;

    STV[0] = CTML[0][0] * XTV[0] + CTML[0][1] * XTV[1] + CTML[0][2] * XTV[2];
    STV[1] = CTML[1][0] * XTV[0] + CTML[1][1] * XTV[1] + CTML[1][2] * XTV[2];
    STV[2] = CTML[2][0] * XTV[0] + CTML[2][1] * XTV[1] + CTML[2][2] * XTV[2];

    XTV[3] = CTMA_inv[0][0] * STV[3] + CTMA_inv[0][1] * STV[4] + CTMA_inv[0][2] * STV[5];
    XTV[4] = CTMA_inv[1][0] * STV[3] + CTMA_inv[1][1] * STV[4] + CTMA_inv[1][2] * STV[5];
    XTV[5] = CTMA_inv[2][0] * STV[3] + CTMA_inv[2][1] * STV[4] + CTMA_inv[2][2] * STV[5];

    // ======================================================================
    // IBSC 제어 로직
    // ======================================================================
#define ACCEL_LIMIT_LINEAR  20.0
#define ACCEL_LIMIT_ANGULAR 10.0
    double ACCEL_LPF_ALPHA = 0.5;

    double raw_acc_linear[3];
    raw_acc_linear[0] = (STV[0] - STV_OLD[0]) / DT;
    raw_acc_linear[1] = (STV[1] - STV_OLD[1]) / DT;
    raw_acc_linear[2] = (STV[2] - STV_OLD[2]) / DT;

    double raw_acc_angular[3];
    raw_acc_angular[0] = Estimate_Accel_Hermite(1, STV[3], DT);
    raw_acc_angular[1] = Estimate_Accel_Hermite(2, STV[4], DT);
    raw_acc_angular[2] = Estimate_Accel_Hermite(3, STV[5], DT);

    for (int i = 0; i < 3; i++) {
        if (raw_acc_linear[i] > ACCEL_LIMIT_LINEAR) raw_acc_linear[i] = ACCEL_LIMIT_LINEAR;
        if (raw_acc_linear[i] < -ACCEL_LIMIT_LINEAR) raw_acc_linear[i] = -ACCEL_LIMIT_LINEAR;

        if (raw_acc_angular[i] > ACCEL_LIMIT_ANGULAR) raw_acc_angular[i] = ACCEL_LIMIT_ANGULAR;
        if (raw_acc_angular[i] < -ACCEL_LIMIT_ANGULAR) raw_acc_angular[i] = -ACCEL_LIMIT_ANGULAR;

        Est_Acc_Body_Filtered[i] = (1.0 - ACCEL_LPF_ALPHA) * Est_Acc_Body_Filtered[i] + ACCEL_LPF_ALPHA * raw_acc_linear[i];
        Est_Acc_Body_Filtered[i + 3] = (1.0 - ACCEL_LPF_ALPHA) * Est_Acc_Body_Filtered[i + 3] + ACCEL_LPF_ALPHA * raw_acc_angular[i];
    }

    Acc_0[0] = CTML_inv[2][0] * Est_Acc_Body_Filtered[0] + CTML_inv[2][1] * Est_Acc_Body_Filtered[1] + CTML_inv[2][2] * Est_Acc_Body_Filtered[2];
    Acc_0[1] = CTMA_inv[0][0] * Est_Acc_Body_Filtered[3] + CTMA_inv[0][1] * Est_Acc_Body_Filtered[4] + CTMA_inv[0][2] * Est_Acc_Body_Filtered[5];
    Acc_0[2] = CTMA_inv[1][0] * Est_Acc_Body_Filtered[3] + CTMA_inv[1][1] * Est_Acc_Body_Filtered[4] + CTMA_inv[1][2] * Est_Acc_Body_Filtered[5];
    Acc_0[3] = CTMA_inv[2][0] * Est_Acc_Body_Filtered[3] + CTMA_inv[2][1] * Est_Acc_Body_Filtered[4] + CTMA_inv[2][2] * Est_Acc_Body_Filtered[5];

    double q_gain[6], k1_gain[6], k2_gain[6];

    q_gain[2] = 1.0 / (CLAW_P.BSC_Ome_ZZ * CLAW_P.BSC_Ome_ZZ * (1.0 - CLAW_P.BSC_Zeta_ZZ * CLAW_P.BSC_Zeta_ZZ));
    k1_gain[2] = CLAW_P.BSC_Zeta_ZZ * CLAW_P.BSC_Ome_ZZ / q_gain[2];
    k2_gain[2] = CLAW_P.BSC_Zeta_ZZ * CLAW_P.BSC_Ome_ZZ;

    q_gain[3] = 1.0 / (CLAW_P.BSC_Ome_PH * CLAW_P.BSC_Ome_PH * (1.0 - CLAW_P.BSC_Zeta_PH * CLAW_P.BSC_Zeta_PH));
    k1_gain[3] = CLAW_P.BSC_Zeta_PH * CLAW_P.BSC_Ome_PH / q_gain[3];
    k2_gain[3] = CLAW_P.BSC_Zeta_PH * CLAW_P.BSC_Ome_PH;

    q_gain[4] = 1.0 / (CLAW_P.BSC_Ome_TH * CLAW_P.BSC_Ome_TH * (1.0 - CLAW_P.BSC_Zeta_TH * CLAW_P.BSC_Zeta_TH));
    k1_gain[4] = CLAW_P.BSC_Zeta_TH * CLAW_P.BSC_Ome_TH / q_gain[4];
    k2_gain[4] = CLAW_P.BSC_Zeta_TH * CLAW_P.BSC_Ome_TH;

    q_gain[5] = 1.0 / (CLAW_P.BSC_Ome_PS * CLAW_P.BSC_Ome_PS * (1.0 - CLAW_P.BSC_Zeta_PS * CLAW_P.BSC_Zeta_PS));
    k1_gain[5] = CLAW_P.BSC_Zeta_PS * CLAW_P.BSC_Ome_PS / q_gain[5];
    k2_gain[5] = CLAW_P.BSC_Zeta_PS * CLAW_P.BSC_Ome_PS;

    if (home_init) {
        Target_X = (Dest_Lat - Home_Lat) * Lat2m(Home_Lat);
        Target_Y = (Dest_Lon - Home_Lon) * Lon2m(Home_Lon);
        Target_Z = (Dest_Alt - Home_Alt);
        

        // Target Heading
        ps_cmd = wrapToPi((double)CLAW_U.Ship_heading);
    }
    else {
        Target_X = 0.0;
        Target_Y = 0.0;
        Target_Z = 0.0;
        ps_cmd = STV[8];
    }

    Err_N = Target_X - STV[9];
    Err_E = Target_Y - STV[10];
    Err_D = Target_Z - STV[11];

    double leash_dist_1 = 25.0;
    double leash_dist_2 = 25.0;

    if (Err_N > leash_dist_1) Err_N = leash_dist_1;
    else if (Err_N < -leash_dist_1) Err_N = -leash_dist_1;

    if (Err_E > leash_dist_2) Err_E = leash_dist_2;
    else if (Err_E < -leash_dist_2) Err_E = -leash_dist_2;

    double pos_lim = CLAW_P.BSC_Int_Limit;
    TV_BSC[0] += Err_N * DT;
    if (TV_BSC[0] > pos_lim) TV_BSC[0] = pos_lim;
    else if (TV_BSC[0] < -pos_lim) TV_BSC[0] = -pos_lim;

    TV_BSC[1] += Err_E * DT;
    if (TV_BSC[1] > pos_lim) TV_BSC[1] = pos_lim;
    else if (TV_BSC[1] < -pos_lim) TV_BSC[1] = -pos_lim;

    pos_des_dot_trim[0] = CTML_inv[0][0] * STV_trim[0] + CTML_inv[0][1] * STV_trim[1] + CTML_inv[0][2] * STV_trim[2];
    pos_des_dot_trim[1] = CTML_inv[1][0] * STV_trim[0] + CTML_inv[1][1] * STV_trim[1] + CTML_inv[1][2] * STV_trim[2];
    pos_des_dot_trim[2] = CTML_inv[2][0] * STV_trim[0] + CTML_inv[2][1] * STV_trim[1] + CTML_inv[2][2] * STV_trim[2];

    pos_dot_des[0] = pos_des_dot_trim[0] + CLAW_P.BSC_K_POS_P * Err_N + CLAW_P.BSC_K_POS_I * TV_BSC[0];
    pos_dot_des[1] = pos_des_dot_trim[1] + CLAW_P.BSC_K_POS_P * Err_E + CLAW_P.BSC_K_POS_I * TV_BSC[1];
    pos_dot_des[2] = 0.0;
    
    uv_des[0] = CTML_inv[0][0] * pos_dot_des[0] + CTML_inv[0][1] * pos_dot_des[1] + CTML_inv[0][2] * pos_dot_des[2];
    uv_des[1] = CTML_inv[1][0] * pos_dot_des[0] + CTML_inv[1][1] * pos_dot_des[1] + CTML_inv[1][2] * pos_dot_des[2];
    uv_des[2] = CTML_inv[2][0] * pos_dot_des[0] + CTML_inv[2][1] * pos_dot_des[1] + CTML_inv[2][2] * pos_dot_des[2];

    double max_vel = 5.0;
    if (uv_des[0] > max_vel) uv_des[0] = max_vel;
    if (uv_des[0] < -max_vel) uv_des[0] = -max_vel;
    if (uv_des[1] > max_vel) uv_des[1] = max_vel;
    if (uv_des[1] < -max_vel) uv_des[1] = -max_vel;

    TV_BSC[2] += (STV[1] - uv_des[1]) * DT;
    TV_BSC[3] += (STV[0] - uv_des[0]) * DT;

    double lim = CLAW_P.BSC_Int_Limit;
    if (TV_BSC[2] > lim) TV_BSC[2] = lim; else if (TV_BSC[2] < -lim) TV_BSC[2] = -lim;
    if (TV_BSC[3] > lim) TV_BSC[3] = lim; else if (TV_BSC[3] < -lim) TV_BSC[3] = -lim;

    double ph_cmd = STV_trim[6] - CLAW_P.BSC_K_VEL_P * (STV[1] - uv_des[1]) - CLAW_P.BSC_K_VEL_I * TV_BSC[2];
    double th_cmd = STV_trim[7] + CLAW_P.BSC_K_VEL_P * (STV[0] - uv_des[0]) + CLAW_P.BSC_K_VEL_I * TV_BSC[3];
    double zz_cmd = Target_Z;

    // Trajectory Generator 입력용 최단 거리 오차 산출
    double ps_err = wrapToPi(ps_cmd - Xtraj[3]);

    ddXtraj[0] = (zz_cmd - Xtraj[0]) * CLAW_P.BSC_Ome_ZZ * CLAW_P.BSC_Ome_ZZ - 2.0 * CLAW_P.BSC_Zeta_ZZ * CLAW_P.BSC_Ome_ZZ * dXtraj[0];
    ddXtraj[1] = (ph_cmd - Xtraj[1]) * CLAW_P.BSC_Ome_PH * CLAW_P.BSC_Ome_PH - 2.0 * CLAW_P.BSC_Zeta_PH * CLAW_P.BSC_Ome_PH * dXtraj[1];
    ddXtraj[2] = (th_cmd - Xtraj[2]) * CLAW_P.BSC_Ome_TH * CLAW_P.BSC_Ome_TH - 2.0 * CLAW_P.BSC_Zeta_TH * CLAW_P.BSC_Ome_TH * dXtraj[2];
    ddXtraj[3] = ps_err * CLAW_P.BSC_Ome_PS * CLAW_P.BSC_Ome_PS - 2.0 * CLAW_P.BSC_Zeta_PS * CLAW_P.BSC_Ome_PS * dXtraj[3];

    dXtraj[0] += DT * ddXtraj[0]; Xtraj[0] += DT * dXtraj[0];
    dXtraj[1] += DT * ddXtraj[1]; Xtraj[1] += DT * dXtraj[1];
    dXtraj[2] += DT * ddXtraj[2]; Xtraj[2] += DT * dXtraj[2];
    
    // Yaw 필터 상태량적분 및 Bounds Wrap
    dXtraj[3] += DT * ddXtraj[3]; 
    Xtraj[3]   = wrapToPi(Xtraj[3] + DT * dXtraj[3]);

    int i, j;
    for (i = 0; i < 4; i++) for (j = 0; j < 4; j++) G_mat[i][j] = 0.0;

    for (j = 0; j < 4; j++) {
        G_mat[0][j] = CTML_inv[2][0] * CLAW_P.BSC_B_mat[0 * 4 + j] +
            CTML_inv[2][1] * CLAW_P.BSC_B_mat[1 * 4 + j] +
            CTML_inv[2][2] * CLAW_P.BSC_B_mat[2 * 4 + j];
    }

    for (i = 1; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            G_mat[i][j] = CTMA_inv[i - 1][0] * CLAW_P.BSC_B_mat[3 * 4 + j] +
                CTMA_inv[i - 1][1] * CLAW_P.BSC_B_mat[4 * 4 + j] +
                CTMA_inv[i - 1][2] * CLAW_P.BSC_B_mat[5 * 4 + j];
        }
    }

    for (i = 0; i < 4; i++) {
        if (fabs(G_mat[i][i]) < 1e-6) {
            G_mat[i][i] = (G_mat[i][i] >= 0) ? 1e-6 : -1e-6;
        }
        G_mat_inv[i][i] = 1.0 / G_mat[i][i];
    }

    z1[0] = STV[11] - Xtraj[0]; 
    z1[1] = STV[6] - Xtraj[1]; 
    z1[2] = STV[7] - Xtraj[2]; 
    z1[3] = wrapToPi(STV[8] - Xtraj[3]); // Heading 1차 제어 오차 Unwrap

    if (fabs(z1[3]) < 0.035) z1[3] = 0.0;

    double z1_dot[4];
    z1_dot[0] = XTV[2] - dXtraj[0];
    z1_dot[1] = XTV[3] - dXtraj[1];
    z1_dot[2] = XTV[4] - dXtraj[2];
    z1_dot[3] = XTV[5] - dXtraj[3];

    alpha[0] = -1.0 * q_gain[2] * k1_gain[2] * z1[0] + dXtraj[0];
    alpha[1] = -1.0 * q_gain[3] * k1_gain[3] * z1[1] + dXtraj[1];
    alpha[2] = -1.0 * q_gain[4] * k1_gain[4] * z1[2] + dXtraj[2];
    alpha[3] = -1.0 * q_gain[5] * k1_gain[5] * z1[3] + dXtraj[3];

    z2[0] = XTV[2] - alpha[0]; z2[1] = XTV[3] - alpha[1]; z2[2] = XTV[4] - alpha[2]; z2[3] = XTV[5] - alpha[3];

    alpha_dot[0] = -1.0 * q_gain[2] * k1_gain[2] * z1_dot[0] + ddXtraj[0];
    alpha_dot[1] = -1.0 * q_gain[3] * k1_gain[3] * z1_dot[1] + ddXtraj[1];
    alpha_dot[2] = -1.0 * q_gain[4] * k1_gain[4] * z1_dot[2] + ddXtraj[2];
    alpha_dot[3] = -1.0 * q_gain[5] * k1_gain[5] * z1_dot[3] + ddXtraj[3];

    f[0] = 0.0; f[1] = 0.0; f[2] = 0.0; f[3] = 0.0;

    double u_temp[4];
    u_temp[0] = k2_gain[2] * z2[0] + z1[0] / q_gain[2] - alpha_dot[0] - Acc_0[0];
    u_temp[1] = k2_gain[3] * z2[1] + z1[1] / q_gain[3] - alpha_dot[1] - Acc_0[1];
    u_temp[2] = k2_gain[4] * z2[2] + z1[2] / q_gain[4] - alpha_dot[2] - Acc_0[2];
    u_temp[3] = k2_gain[5] * z2[3] + z1[3] / q_gain[5] - alpha_dot[3] - Acc_0[3];

    Del_Control[0] = -1.0 * (G_mat_inv[0][0] * u_temp[0] + G_mat_inv[0][1] * u_temp[1] + G_mat_inv[0][2] * u_temp[2] + G_mat_inv[0][3] * u_temp[3]);
    Del_Control[1] = -1.0 * (G_mat_inv[1][0] * u_temp[0] + G_mat_inv[1][1] * u_temp[1] + G_mat_inv[1][2] * u_temp[2] + G_mat_inv[1][3] * u_temp[3]);
    Del_Control[2] = -1.0 * (G_mat_inv[2][0] * u_temp[0] + G_mat_inv[2][1] * u_temp[1] + G_mat_inv[2][2] * u_temp[2] + G_mat_inv[2][3] * u_temp[3]);
    Del_Control[3] = -1.0 * (G_mat_inv[3][0] * u_temp[0] + G_mat_inv[3][1] * u_temp[1] + G_mat_inv[3][2] * u_temp[2] + G_mat_inv[3][3] * u_temp[3]);

    for (i = 0; i < 3; i++) STV_OLD[i] = STV[i];

    CLAW_Y.v_cmd.cmd_height = (real32_T)(Del_Control[0] * CLAW_P.BSC_Scale_Thrust);
    CLAW_Y.v_cmd.cmd_roll = (real32_T)(Del_Control[1] * CLAW_P.BSC_Scale_Roll * 1.0);
    CLAW_Y.v_cmd.cmd_pitch = (real32_T)(Del_Control[2] * CLAW_P.BSC_Scale_Pitch * -1.0);
    CLAW_Y.v_cmd.cmd_yaw = (real32_T)(Del_Control[3] * CLAW_P.BSC_Scale_Yaw * 1.0);

    if (CLAW_Y.v_cmd.cmd_roll > 1.0F) CLAW_Y.v_cmd.cmd_roll = 1.0F;
    else if (CLAW_Y.v_cmd.cmd_roll < -1.0F) CLAW_Y.v_cmd.cmd_roll = -1.0F;

    if (CLAW_Y.v_cmd.cmd_pitch > 1.0F) CLAW_Y.v_cmd.cmd_pitch = 1.0F;
    else if (CLAW_Y.v_cmd.cmd_pitch < -1.0F) CLAW_Y.v_cmd.cmd_pitch = -1.0F;

    if (CLAW_Y.v_cmd.cmd_yaw > 1.0F) CLAW_Y.v_cmd.cmd_yaw = 1.0F;
    else if (CLAW_Y.v_cmd.cmd_yaw < -1.0F) CLAW_Y.v_cmd.cmd_yaw = -1.0F;

    if (CLAW_Y.v_cmd.cmd_height > 1.0F) CLAW_Y.v_cmd.cmd_height = 1.0F;
    else if (CLAW_Y.v_cmd.cmd_height < -1.0F) CLAW_Y.v_cmd.cmd_height = -1.0F;

    Prev_CTRL[0] = (double)(CLAW_Y.v_cmd.cmd_height / CLAW_P.BSC_Scale_Thrust);
    Prev_CTRL[1] = (double)(CLAW_Y.v_cmd.cmd_roll / (CLAW_P.BSC_Scale_Roll * 1.0));
    Prev_CTRL[2] = (double)(CLAW_Y.v_cmd.cmd_pitch / (CLAW_P.BSC_Scale_Pitch * -1.0));
    Prev_CTRL[3] = (double)(CLAW_Y.v_cmd.cmd_yaw / CLAW_P.BSC_Scale_Yaw * 1.0);

    // Logging Position (NED)
    CLAW_Y.cur_poti.x = (real32_T)STV[9];
    CLAW_Y.cur_poti.y = (real32_T)STV[10];
    CLAW_Y.cur_poti.z = -(real32_T)STV[11];
}

/* Model initialize function */
void CLAW_initialize(void)
{
    CLAW_MedianFilter_Init(&CLAW_DW.MedianFilter);
    CLAW_MedianFilter_Init(&CLAW_DW.MedianFilter1);
    CLAW_MedianFilter_Init(&CLAW_DW.MedianFilter2);

    for (int i = 0; i < 4; i++) {
        TV_BSC[i] = 0.0;
    }

    CLAW_DW.obj.matlabCodegenIsDeleted = false;
    CLAW_DW.obj.isInitialized = 1L;
    CLAW_DW.obj.NumChannels = 1L;
    CLAW_DW.obj.pMID.isInitialized = 0L;
    CLAW_DW.obj.isSetupComplete = true;
}

/* Model terminate function */
void CLAW_terminate(void)
{
    CLAW_MedianFilter_Term(&CLAW_DW.MedianFilter);
    CLAW_MedianFilter_Term(&CLAW_DW.MedianFilter1);
    CLAW_MedianFilter_Term(&CLAW_DW.MedianFilter2);

    if (!CLAW_DW.obj.matlabCodegenIsDeleted) {
        CLAW_DW.obj.matlabCodegenIsDeleted = true;
        if ((CLAW_DW.obj.isInitialized == 1L) && CLAW_DW.obj.isSetupComplete) {
            CLAW_DW.obj.NumChannels = -1L;
            if (CLAW_DW.obj.pMID.isInitialized == 1L) {
                CLAW_DW.obj.pMID.isInitialized = 2L;
            }
        }
    }
}