/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: CLAW.h
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


#ifndef RTW_HEADER_CLAW_h_
#define RTW_HEADER_CLAW_h_
#ifndef CLAW_COMMON_INCLUDES_
#define CLAW_COMMON_INCLUDES_
#include "mode_tdcn_rtwtypes.h"
#endif                                 /* CLAW_COMMON_INCLUDES_ */

#include "mode_tdcn_CLAW_types.h"



/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block signals for system '<S1>/Median Filter' */
typedef struct {
  real_T MedianFilter;                 /* '<S1>/Median Filter' */
  real_T vprev;
  real_T p;
} B_MedianFilter_CLAW_T;

/* Block states (default storage) for system '<S1>/Median Filter' */
typedef struct {
  dsp_simulink_MedianFilter_CLA_T obj; /* '<S1>/Median Filter' */
  boolean_T objisempty;                /* '<S1>/Median Filter' */
} DW_MedianFilter_CLAW_T;

/* Block signals (default storage) */
typedef struct {
  real_T J_data[12];
  real_T B_data[12];
  real_T y[9];
  real_T F_data[4];
  real_T d_a;
  real_T e_a;
  real_T f_a;
  real_T g_a;
  real_T h_a;
  real_T i_a;
  real_T j_a;
  real_T k_a;
  real_T l_a;
  real_T m_a;
  real_T bkj;
  real_T a21;
  real_T rtb_DataTypeConversion_idx_0;
  real_T rtb_DataTypeConversion_idx_1;
  real_T rtb_DataTypeConversion_idx_2;
  real_T rtb_DataTypeConversion_idx_3;
  real_T rtb_DataTypeConversion4_idx_0;
  real_T rtb_DataTypeConversion4_idx_1;
  real_T rtb_DataTypeConversion4_idx_2;
  real_T rtb_Y_idx_0;
  real_T rtb_Y_idx_1;
  real_T rtb_Y_idx_2;
  real_T rtb_DataTypeConversion5_idx_0;
  real_T rtb_DataTypeConversion5_idx_1;
  real_T rtb_DataTypeConversion5_idx_2;
  real_T rtb_DataTypeConversion6_idx_0;
  real_T rtb_DataTypeConversion6_idx_1;
  real_T rtb_DataTypeConversion6_idx_2;
  real_T rtb_DataTypeConversion7_idx_0;
  real_T rtb_DataTypeConversion7_idx_1;
  real_T rtb_DataTypeConversion7_idx_2;
  real_T ind1;
  real_T ind2;
  real_T d;
  real_T d1;
  real_T ind1_m;
  real_T ind2_c;
  real_T d2;
  real_T d3;
  real_T cnt1;
  real_T cnt2;
  int8_T ii_data[4];
  boolean_T b_b_data[4];
  boolean_T x_tmp[4];
  int16_T idx;
  int16_T b_ii;
  int16_T nrows;
  int16_T i;
  int16_T c_i;
  int16_T coffset;
  int16_T J_size_idx_0;
  int16_T i_k;
  boolean_T flag;
  B_MedianFilter_CLAW_T MedianFilter2; /* [추가] Z축 필터용 */
  B_MedianFilter_CLAW_T MedianFilter1; /* '<S1>/Median Filter' */
  B_MedianFilter_CLAW_T MedianFilter;  /* '<S1>/Median Filter' */
} B_CLAW_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  dsp_simulink_MedianFilter_C_b_T obj; /* '<S1>/Median Filter2' */
  real_T UD_DSTATE;                    /* '<S4>/UD' */
  real_T Integrator_DSTATE;            /* '<S44>/Integrator' */
  real_T Filter_DSTATE;                /* '<S39>/Filter' */
  real_T UD_DSTATE_i;                  /* '<S5>/UD' */
  real_T Integrator_DSTATE_p;          /* '<S140>/Integrator' */
  real_T Filter_DSTATE_m;              /* '<S135>/Filter' */
  real_T UD_DSTATE_l;                  /* '<S6>/UD' */
  real_T Integrator_DSTATE_k;          /* '<S92>/Integrator' */
  real_T Filter_DSTATE_o;              /* '<S87>/Filter' */
  real_T UD_DSTATE_p;                  /* '<S7>/UD' */
  real_T Integrator_DSTATE_g;          /* '<S188>/Integrator' */
  real_T Filter_DSTATE_g;              /* '<S183>/Filter' */
  DW_MedianFilter_CLAW_T MedianFilter2; /* [추가] Z축 필터용 */
  DW_MedianFilter_CLAW_T MedianFilter1;/* '<S1>/Median Filter' */
  DW_MedianFilter_CLAW_T MedianFilter; /* '<S1>/Median Filter' */
} DW_CLAW_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  //Anchor_point Anchor_point_g;         /* '<Root>/Anchor_point' */
  //Anchort2Tag_dist dist;               /* '<Root>/Dist' */
  Dest_poti Cur_Pos;     /* '<Root>/Cur_Pos' : ���� ��ġ (x,y,z) */
  Dest_poti Dest_poti_i;               /* '<Root>/Dest_poti' */
  DR_heading DR_heading_f;             /* '<Root>/DR_heading' */
  // --- [추가] 선박 목표 Heading 입력 정의 ---
    real32_T Ship_heading;
//�߰� ----------------------------------------------------------------
/* IMU ���� ������ (Roll, Pitch, p, q, r) */
    real32_T Roll;  // Roll ���� (rad)
    real32_T Pitch; // Pitch ���� (rad)
    real32_T p;     // Roll ���ӵ� (rad/s)
    real32_T q;     // Pitch ���ӵ� (rad/s)
    real32_T r;     // Yaw ���ӵ� (rad/s)
} ExtU_CLAW_T;
//----------------------------------------------------------------

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  V_cmd v_cmd;                         /* '<Root>/v_cmd' */
  //Anchor_point AP_Location;            /* '<Root>/AP_Location' */
  //Anchort2Tag_dist AP_distant;         /* '<Root>/AP_distant' */
  Dest_poti cur_poti;                  /* '<Root>/cur_poti' */
} ExtY_CLAW_T;

/* Parameters (default storage) */
typedef struct P_CLAW_T_ {
    /* --- [1] Scaling Factors (��� ������) --- */
    real_T BSC_Scale_Thrust;
    real_T BSC_Scale_Roll;
    real_T BSC_Scale_Pitch;
    real_T BSC_Scale_Yaw;

    /* --- [2] BSC Control Gains (Ʃ�� ����) --- */
    // Outer Loop Gains(Position& Velocity)* /
    real_T BSC_K_POS_P;  // ���� K_POS_P (dpp_kapa_x, y)
    real_T BSC_K_POS_I;  // ���� K_POS_I (dpp_kapa_xintg, yintg)
    real_T BSC_K_VEL_P;  // ���� K_VEL_P (dpp_kapa_u, v)
    real_T BSC_K_VEL_I;  // ���� K_VEL_I (dpp_kapa_uintg, vintg)
    /* [�߰�] Anti-Windup Limit (���� ���Ѱ�) */
    real_T BSC_Int_Limit;

    /* Omega (Natural Frequency) */
    real_T BSC_Ome_XX;
    real_T BSC_Ome_YY;
    real_T BSC_Ome_ZZ;
    real_T BSC_Ome_PH;
    real_T BSC_Ome_TH;
    real_T BSC_Ome_PS;

    /* Zeta (Damping Ratio) */
    real_T BSC_Zeta_XX;
    real_T BSC_Zeta_YY;
    real_T BSC_Zeta_ZZ;
    real_T BSC_Zeta_PH;
    real_T BSC_Zeta_TH;
    real_T BSC_Zeta_PS;

    /* Time Constants */
    real_T BSC_Tau_hdot;
    real_T BSC_Tau_r;

    /* --- [3] System Matrix (B Matrix) --- */
    /* Controller_Var.c�� B_mat[12][4]�� 1���� �迭[48]�� ���� */
    real_T BSC_B_mat[48];

 // real_T Kd_height;                    /* Variable: Kd_height
 //                                       * Referenced by: '<S182>/Derivative Gain'
 //                                       */
 // real_T Kd_pitch;                     /* Variable: Kd_pitch
 //                                       * Referenced by: '<S134>/Derivative Gain'
 //                                       */
 // real_T Kd_roll;                      /* Variable: Kd_roll
 //                                       * Referenced by: '<S38>/Derivative Gain'
 //                                       */
 // real_T Kd_yaw;                       /* Variable: Kd_yaw
 //                                       * Referenced by: '<S86>/Derivative Gain'
 //                                       */
 // real_T Ki_height;                    /* Variable: Ki_height
 //                                       * Referenced by: '<S185>/Integral Gain'
 //                                       */
 // real_T Ki_pitch;                     /* Variable: Ki_pitch
 //                                       * Referenced by: '<S137>/Integral Gain'
 //                                       */
 // real_T Ki_roll;                      /* Variable: Ki_roll
 //                                       * Referenced by: '<S41>/Integral Gain'
 //                                       */
 // real_T Ki_yaw;                       /* Variable: Ki_yaw
 //                                       * Referenced by: '<S89>/Integral Gain'
 //                                       */
 // real_T Kp_height;                    /* Variable: Kp_height
 //                                       * Referenced by: '<S193>/Proportional Gain'
 //                                       */
 // real_T Kp_pitch;                     /* Variable: Kp_pitch
 //                                       * Referenced by: '<S145>/Proportional Gain'
 //                                       */
 // real_T Kp_roll;                      /* Variable: Kp_roll
 //                                       * Referenced by: '<S49>/Proportional Gain'
 //                                       */
 // real_T Kp_yaw;                       /* Variable: Kp_yaw
 //                                       * Referenced by: '<S97>/Proportional Gain'
 //                                       */
 // real_T height_scale;                 /* Variable: height_scale
 //                                       * Referenced by: '<S1>/height_scale'
 //                                       */
 // real_T pitch_scale;                  /* Variable: pitch_scale
 //                                       * Referenced by: '<S1>/pitch_scale'
 //                                       */
 // real_T roll_scale;                   /* Variable: roll_scale
 //                                       * Referenced by: '<S1>/roll_scale'
 //                                       */
 // real_T yaw_scale;                    /* Variable: yaw_scale
 //                                       * Referenced by: '<S1>/yaw_scale'
 //                                       */
 } P_CLAW_T;

/* Real-time Model Data Structure */
struct tag_RTM_CLAW_T {
  const char_T * volatile errorStatus;
};

/* Block parameters (default storage) */
extern P_CLAW_T CLAW_P;

/* Block signals (default storage) */
extern B_CLAW_T CLAW_B;

/* Block states (default storage) */
extern DW_CLAW_T CLAW_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_CLAW_T CLAW_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_CLAW_T CLAW_Y;

/* Model entry point functions */
extern void CLAW_initialize(void);
extern void CLAW_step(void);
extern void CLAW_terminate(void);

/* Real-time Model object */
extern RT_MODEL_CLAW_T *const CLAW_M;

extern double Target_X, Target_Y, Target_Z;
extern double Err_N, Err_E, Err_D;
extern double Home_Lat, Home_Lon, Home_Alt, Home_Yaw;
extern double STV[12];

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S4>/Data Type Duplicate' : Unused code path elimination
 * Block '<S5>/Data Type Duplicate' : Unused code path elimination
 * Block '<S6>/Data Type Duplicate' : Unused code path elimination
 * Block '<S7>/Data Type Duplicate' : Unused code path elimination
 * Block '<S47>/Filter Coefficient' : Eliminated nontunable gain of 1
 * Block '<S95>/Filter Coefficient' : Eliminated nontunable gain of 1
 * Block '<S143>/Filter Coefficient' : Eliminated nontunable gain of 1
 * Block '<S191>/Filter Coefficient' : Eliminated nontunable gain of 1
 * Block '<S1>/Reshape' : Reshape block reduction
 * Block '<S12>/Reshape (9) to [3x3] column-major' : Reshape block reduction
 * Block '<S1>/roll_cmd_direction' : Eliminated nontunable gain of 1
 * Block '<S1>/yaw_cmd_direction' : Eliminated nontunable gain of 1
 * Block '<S2>/Reshape3' : Reshape block reduction
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'CLAW'
 * '<S1>'   : 'CLAW/PID controller'
 * '<S2>'   : 'CLAW/Transform_AP_to_NED'
 * '<S3>'   : 'CLAW/PID controller/Degrees to Radians'
 * '<S4>'   : 'CLAW/PID controller/Discrete Derivative'
 * '<S5>'   : 'CLAW/PID controller/Discrete Derivative1'
 * '<S6>'   : 'CLAW/PID controller/Discrete Derivative2'
 * '<S7>'   : 'CLAW/PID controller/Discrete Derivative3'
 * '<S8>'   : 'CLAW/PID controller/PID Controller1'
 * '<S9>'   : 'CLAW/PID controller/PID Controller2'
 * '<S10>'  : 'CLAW/PID controller/PID Controller3'
 * '<S11>'  : 'CLAW/PID controller/PID Controller4'
 * '<S12>'  : 'CLAW/PID controller/Subsystem3'
 * '<S13>'  : 'CLAW/PID controller/PID Controller1/Anti-windup'
 * '<S14>'  : 'CLAW/PID controller/PID Controller1/D Gain'
 * '<S15>'  : 'CLAW/PID controller/PID Controller1/Filter'
 * '<S16>'  : 'CLAW/PID controller/PID Controller1/Filter ICs'
 * '<S17>'  : 'CLAW/PID controller/PID Controller1/I Gain'
 * '<S18>'  : 'CLAW/PID controller/PID Controller1/Ideal P Gain'
 * '<S19>'  : 'CLAW/PID controller/PID Controller1/Ideal P Gain Fdbk'
 * '<S20>'  : 'CLAW/PID controller/PID Controller1/Integrator'
 * '<S21>'  : 'CLAW/PID controller/PID Controller1/Integrator ICs'
 * '<S22>'  : 'CLAW/PID controller/PID Controller1/N Copy'
 * '<S23>'  : 'CLAW/PID controller/PID Controller1/N Gain'
 * '<S24>'  : 'CLAW/PID controller/PID Controller1/P Copy'
 * '<S25>'  : 'CLAW/PID controller/PID Controller1/Parallel P Gain'
 * '<S26>'  : 'CLAW/PID controller/PID Controller1/Reset Signal'
 * '<S27>'  : 'CLAW/PID controller/PID Controller1/Saturation'
 * '<S28>'  : 'CLAW/PID controller/PID Controller1/Saturation Fdbk'
 * '<S29>'  : 'CLAW/PID controller/PID Controller1/Sum'
 * '<S30>'  : 'CLAW/PID controller/PID Controller1/Sum Fdbk'
 * '<S31>'  : 'CLAW/PID controller/PID Controller1/Tracking Mode'
 * '<S32>'  : 'CLAW/PID controller/PID Controller1/Tracking Mode Sum'
 * '<S33>'  : 'CLAW/PID controller/PID Controller1/Tsamp - Integral'
 * '<S34>'  : 'CLAW/PID controller/PID Controller1/Tsamp - Ngain'
 * '<S35>'  : 'CLAW/PID controller/PID Controller1/postSat Signal'
 * '<S36>'  : 'CLAW/PID controller/PID Controller1/preSat Signal'
 * '<S37>'  : 'CLAW/PID controller/PID Controller1/Anti-windup/Passthrough'
 * '<S38>'  : 'CLAW/PID controller/PID Controller1/D Gain/Internal Parameters'
 * '<S39>'  : 'CLAW/PID controller/PID Controller1/Filter/Disc. Forward Euler Filter'
 * '<S40>'  : 'CLAW/PID controller/PID Controller1/Filter ICs/Internal IC - Filter'
 * '<S41>'  : 'CLAW/PID controller/PID Controller1/I Gain/Internal Parameters'
 * '<S42>'  : 'CLAW/PID controller/PID Controller1/Ideal P Gain/Passthrough'
 * '<S43>'  : 'CLAW/PID controller/PID Controller1/Ideal P Gain Fdbk/Disabled'
 * '<S44>'  : 'CLAW/PID controller/PID Controller1/Integrator/Discrete'
 * '<S45>'  : 'CLAW/PID controller/PID Controller1/Integrator ICs/Internal IC'
 * '<S46>'  : 'CLAW/PID controller/PID Controller1/N Copy/Disabled'
 * '<S47>'  : 'CLAW/PID controller/PID Controller1/N Gain/Internal Parameters'
 * '<S48>'  : 'CLAW/PID controller/PID Controller1/P Copy/Disabled'
 * '<S49>'  : 'CLAW/PID controller/PID Controller1/Parallel P Gain/Internal Parameters'
 * '<S50>'  : 'CLAW/PID controller/PID Controller1/Reset Signal/Disabled'
 * '<S51>'  : 'CLAW/PID controller/PID Controller1/Saturation/Passthrough'
 * '<S52>'  : 'CLAW/PID controller/PID Controller1/Saturation Fdbk/Disabled'
 * '<S53>'  : 'CLAW/PID controller/PID Controller1/Sum/Sum_PID'
 * '<S54>'  : 'CLAW/PID controller/PID Controller1/Sum Fdbk/Disabled'
 * '<S55>'  : 'CLAW/PID controller/PID Controller1/Tracking Mode/Disabled'
 * '<S56>'  : 'CLAW/PID controller/PID Controller1/Tracking Mode Sum/Passthrough'
 * '<S57>'  : 'CLAW/PID controller/PID Controller1/Tsamp - Integral/TsSignalSpecification'
 * '<S58>'  : 'CLAW/PID controller/PID Controller1/Tsamp - Ngain/Passthrough'
 * '<S59>'  : 'CLAW/PID controller/PID Controller1/postSat Signal/Forward_Path'
 * '<S60>'  : 'CLAW/PID controller/PID Controller1/preSat Signal/Forward_Path'
 * '<S61>'  : 'CLAW/PID controller/PID Controller2/Anti-windup'
 * '<S62>'  : 'CLAW/PID controller/PID Controller2/D Gain'
 * '<S63>'  : 'CLAW/PID controller/PID Controller2/Filter'
 * '<S64>'  : 'CLAW/PID controller/PID Controller2/Filter ICs'
 * '<S65>'  : 'CLAW/PID controller/PID Controller2/I Gain'
 * '<S66>'  : 'CLAW/PID controller/PID Controller2/Ideal P Gain'
 * '<S67>'  : 'CLAW/PID controller/PID Controller2/Ideal P Gain Fdbk'
 * '<S68>'  : 'CLAW/PID controller/PID Controller2/Integrator'
 * '<S69>'  : 'CLAW/PID controller/PID Controller2/Integrator ICs'
 * '<S70>'  : 'CLAW/PID controller/PID Controller2/N Copy'
 * '<S71>'  : 'CLAW/PID controller/PID Controller2/N Gain'
 * '<S72>'  : 'CLAW/PID controller/PID Controller2/P Copy'
 * '<S73>'  : 'CLAW/PID controller/PID Controller2/Parallel P Gain'
 * '<S74>'  : 'CLAW/PID controller/PID Controller2/Reset Signal'
 * '<S75>'  : 'CLAW/PID controller/PID Controller2/Saturation'
 * '<S76>'  : 'CLAW/PID controller/PID Controller2/Saturation Fdbk'
 * '<S77>'  : 'CLAW/PID controller/PID Controller2/Sum'
 * '<S78>'  : 'CLAW/PID controller/PID Controller2/Sum Fdbk'
 * '<S79>'  : 'CLAW/PID controller/PID Controller2/Tracking Mode'
 * '<S80>'  : 'CLAW/PID controller/PID Controller2/Tracking Mode Sum'
 * '<S81>'  : 'CLAW/PID controller/PID Controller2/Tsamp - Integral'
 * '<S82>'  : 'CLAW/PID controller/PID Controller2/Tsamp - Ngain'
 * '<S83>'  : 'CLAW/PID controller/PID Controller2/postSat Signal'
 * '<S84>'  : 'CLAW/PID controller/PID Controller2/preSat Signal'
 * '<S85>'  : 'CLAW/PID controller/PID Controller2/Anti-windup/Passthrough'
 * '<S86>'  : 'CLAW/PID controller/PID Controller2/D Gain/Internal Parameters'
 * '<S87>'  : 'CLAW/PID controller/PID Controller2/Filter/Disc. Forward Euler Filter'
 * '<S88>'  : 'CLAW/PID controller/PID Controller2/Filter ICs/Internal IC - Filter'
 * '<S89>'  : 'CLAW/PID controller/PID Controller2/I Gain/Internal Parameters'
 * '<S90>'  : 'CLAW/PID controller/PID Controller2/Ideal P Gain/Passthrough'
 * '<S91>'  : 'CLAW/PID controller/PID Controller2/Ideal P Gain Fdbk/Disabled'
 * '<S92>'  : 'CLAW/PID controller/PID Controller2/Integrator/Discrete'
 * '<S93>'  : 'CLAW/PID controller/PID Controller2/Integrator ICs/Internal IC'
 * '<S94>'  : 'CLAW/PID controller/PID Controller2/N Copy/Disabled'
 * '<S95>'  : 'CLAW/PID controller/PID Controller2/N Gain/Internal Parameters'
 * '<S96>'  : 'CLAW/PID controller/PID Controller2/P Copy/Disabled'
 * '<S97>'  : 'CLAW/PID controller/PID Controller2/Parallel P Gain/Internal Parameters'
 * '<S98>'  : 'CLAW/PID controller/PID Controller2/Reset Signal/Disabled'
 * '<S99>'  : 'CLAW/PID controller/PID Controller2/Saturation/Passthrough'
 * '<S100>' : 'CLAW/PID controller/PID Controller2/Saturation Fdbk/Disabled'
 * '<S101>' : 'CLAW/PID controller/PID Controller2/Sum/Sum_PID'
 * '<S102>' : 'CLAW/PID controller/PID Controller2/Sum Fdbk/Disabled'
 * '<S103>' : 'CLAW/PID controller/PID Controller2/Tracking Mode/Disabled'
 * '<S104>' : 'CLAW/PID controller/PID Controller2/Tracking Mode Sum/Passthrough'
 * '<S105>' : 'CLAW/PID controller/PID Controller2/Tsamp - Integral/TsSignalSpecification'
 * '<S106>' : 'CLAW/PID controller/PID Controller2/Tsamp - Ngain/Passthrough'
 * '<S107>' : 'CLAW/PID controller/PID Controller2/postSat Signal/Forward_Path'
 * '<S108>' : 'CLAW/PID controller/PID Controller2/preSat Signal/Forward_Path'
 * '<S109>' : 'CLAW/PID controller/PID Controller3/Anti-windup'
 * '<S110>' : 'CLAW/PID controller/PID Controller3/D Gain'
 * '<S111>' : 'CLAW/PID controller/PID Controller3/Filter'
 * '<S112>' : 'CLAW/PID controller/PID Controller3/Filter ICs'
 * '<S113>' : 'CLAW/PID controller/PID Controller3/I Gain'
 * '<S114>' : 'CLAW/PID controller/PID Controller3/Ideal P Gain'
 * '<S115>' : 'CLAW/PID controller/PID Controller3/Ideal P Gain Fdbk'
 * '<S116>' : 'CLAW/PID controller/PID Controller3/Integrator'
 * '<S117>' : 'CLAW/PID controller/PID Controller3/Integrator ICs'
 * '<S118>' : 'CLAW/PID controller/PID Controller3/N Copy'
 * '<S119>' : 'CLAW/PID controller/PID Controller3/N Gain'
 * '<S120>' : 'CLAW/PID controller/PID Controller3/P Copy'
 * '<S121>' : 'CLAW/PID controller/PID Controller3/Parallel P Gain'
 * '<S122>' : 'CLAW/PID controller/PID Controller3/Reset Signal'
 * '<S123>' : 'CLAW/PID controller/PID Controller3/Saturation'
 * '<S124>' : 'CLAW/PID controller/PID Controller3/Saturation Fdbk'
 * '<S125>' : 'CLAW/PID controller/PID Controller3/Sum'
 * '<S126>' : 'CLAW/PID controller/PID Controller3/Sum Fdbk'
 * '<S127>' : 'CLAW/PID controller/PID Controller3/Tracking Mode'
 * '<S128>' : 'CLAW/PID controller/PID Controller3/Tracking Mode Sum'
 * '<S129>' : 'CLAW/PID controller/PID Controller3/Tsamp - Integral'
 * '<S130>' : 'CLAW/PID controller/PID Controller3/Tsamp - Ngain'
 * '<S131>' : 'CLAW/PID controller/PID Controller3/postSat Signal'
 * '<S132>' : 'CLAW/PID controller/PID Controller3/preSat Signal'
 * '<S133>' : 'CLAW/PID controller/PID Controller3/Anti-windup/Passthrough'
 * '<S134>' : 'CLAW/PID controller/PID Controller3/D Gain/Internal Parameters'
 * '<S135>' : 'CLAW/PID controller/PID Controller3/Filter/Disc. Forward Euler Filter'
 * '<S136>' : 'CLAW/PID controller/PID Controller3/Filter ICs/Internal IC - Filter'
 * '<S137>' : 'CLAW/PID controller/PID Controller3/I Gain/Internal Parameters'
 * '<S138>' : 'CLAW/PID controller/PID Controller3/Ideal P Gain/Passthrough'
 * '<S139>' : 'CLAW/PID controller/PID Controller3/Ideal P Gain Fdbk/Disabled'
 * '<S140>' : 'CLAW/PID controller/PID Controller3/Integrator/Discrete'
 * '<S141>' : 'CLAW/PID controller/PID Controller3/Integrator ICs/Internal IC'
 * '<S142>' : 'CLAW/PID controller/PID Controller3/N Copy/Disabled'
 * '<S143>' : 'CLAW/PID controller/PID Controller3/N Gain/Internal Parameters'
 * '<S144>' : 'CLAW/PID controller/PID Controller3/P Copy/Disabled'
 * '<S145>' : 'CLAW/PID controller/PID Controller3/Parallel P Gain/Internal Parameters'
 * '<S146>' : 'CLAW/PID controller/PID Controller3/Reset Signal/Disabled'
 * '<S147>' : 'CLAW/PID controller/PID Controller3/Saturation/Passthrough'
 * '<S148>' : 'CLAW/PID controller/PID Controller3/Saturation Fdbk/Disabled'
 * '<S149>' : 'CLAW/PID controller/PID Controller3/Sum/Sum_PID'
 * '<S150>' : 'CLAW/PID controller/PID Controller3/Sum Fdbk/Disabled'
 * '<S151>' : 'CLAW/PID controller/PID Controller3/Tracking Mode/Disabled'
 * '<S152>' : 'CLAW/PID controller/PID Controller3/Tracking Mode Sum/Passthrough'
 * '<S153>' : 'CLAW/PID controller/PID Controller3/Tsamp - Integral/TsSignalSpecification'
 * '<S154>' : 'CLAW/PID controller/PID Controller3/Tsamp - Ngain/Passthrough'
 * '<S155>' : 'CLAW/PID controller/PID Controller3/postSat Signal/Forward_Path'
 * '<S156>' : 'CLAW/PID controller/PID Controller3/preSat Signal/Forward_Path'
 * '<S157>' : 'CLAW/PID controller/PID Controller4/Anti-windup'
 * '<S158>' : 'CLAW/PID controller/PID Controller4/D Gain'
 * '<S159>' : 'CLAW/PID controller/PID Controller4/Filter'
 * '<S160>' : 'CLAW/PID controller/PID Controller4/Filter ICs'
 * '<S161>' : 'CLAW/PID controller/PID Controller4/I Gain'
 * '<S162>' : 'CLAW/PID controller/PID Controller4/Ideal P Gain'
 * '<S163>' : 'CLAW/PID controller/PID Controller4/Ideal P Gain Fdbk'
 * '<S164>' : 'CLAW/PID controller/PID Controller4/Integrator'
 * '<S165>' : 'CLAW/PID controller/PID Controller4/Integrator ICs'
 * '<S166>' : 'CLAW/PID controller/PID Controller4/N Copy'
 * '<S167>' : 'CLAW/PID controller/PID Controller4/N Gain'
 * '<S168>' : 'CLAW/PID controller/PID Controller4/P Copy'
 * '<S169>' : 'CLAW/PID controller/PID Controller4/Parallel P Gain'
 * '<S170>' : 'CLAW/PID controller/PID Controller4/Reset Signal'
 * '<S171>' : 'CLAW/PID controller/PID Controller4/Saturation'
 * '<S172>' : 'CLAW/PID controller/PID Controller4/Saturation Fdbk'
 * '<S173>' : 'CLAW/PID controller/PID Controller4/Sum'
 * '<S174>' : 'CLAW/PID controller/PID Controller4/Sum Fdbk'
 * '<S175>' : 'CLAW/PID controller/PID Controller4/Tracking Mode'
 * '<S176>' : 'CLAW/PID controller/PID Controller4/Tracking Mode Sum'
 * '<S177>' : 'CLAW/PID controller/PID Controller4/Tsamp - Integral'
 * '<S178>' : 'CLAW/PID controller/PID Controller4/Tsamp - Ngain'
 * '<S179>' : 'CLAW/PID controller/PID Controller4/postSat Signal'
 * '<S180>' : 'CLAW/PID controller/PID Controller4/preSat Signal'
 * '<S181>' : 'CLAW/PID controller/PID Controller4/Anti-windup/Passthrough'
 * '<S182>' : 'CLAW/PID controller/PID Controller4/D Gain/Internal Parameters'
 * '<S183>' : 'CLAW/PID controller/PID Controller4/Filter/Disc. Forward Euler Filter'
 * '<S184>' : 'CLAW/PID controller/PID Controller4/Filter ICs/Internal IC - Filter'
 * '<S185>' : 'CLAW/PID controller/PID Controller4/I Gain/Internal Parameters'
 * '<S186>' : 'CLAW/PID controller/PID Controller4/Ideal P Gain/Passthrough'
 * '<S187>' : 'CLAW/PID controller/PID Controller4/Ideal P Gain Fdbk/Disabled'
 * '<S188>' : 'CLAW/PID controller/PID Controller4/Integrator/Discrete'
 * '<S189>' : 'CLAW/PID controller/PID Controller4/Integrator ICs/Internal IC'
 * '<S190>' : 'CLAW/PID controller/PID Controller4/N Copy/Disabled'
 * '<S191>' : 'CLAW/PID controller/PID Controller4/N Gain/Internal Parameters'
 * '<S192>' : 'CLAW/PID controller/PID Controller4/P Copy/Disabled'
 * '<S193>' : 'CLAW/PID controller/PID Controller4/Parallel P Gain/Internal Parameters'
 * '<S194>' : 'CLAW/PID controller/PID Controller4/Reset Signal/Disabled'
 * '<S195>' : 'CLAW/PID controller/PID Controller4/Saturation/Passthrough'
 * '<S196>' : 'CLAW/PID controller/PID Controller4/Saturation Fdbk/Disabled'
 * '<S197>' : 'CLAW/PID controller/PID Controller4/Sum/Sum_PID'
 * '<S198>' : 'CLAW/PID controller/PID Controller4/Sum Fdbk/Disabled'
 * '<S199>' : 'CLAW/PID controller/PID Controller4/Tracking Mode/Disabled'
 * '<S200>' : 'CLAW/PID controller/PID Controller4/Tracking Mode Sum/Passthrough'
 * '<S201>' : 'CLAW/PID controller/PID Controller4/Tsamp - Integral/TsSignalSpecification'
 * '<S202>' : 'CLAW/PID controller/PID Controller4/Tsamp - Ngain/Passthrough'
 * '<S203>' : 'CLAW/PID controller/PID Controller4/postSat Signal/Forward_Path'
 * '<S204>' : 'CLAW/PID controller/PID Controller4/preSat Signal/Forward_Path'
 * '<S205>' : 'CLAW/Transform_AP_to_NED/GaussNewton_NED'
 */
#endif                                 /* RTW_HEADER_CLAW_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
