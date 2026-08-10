/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: CLAW_types.h
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

#ifndef RTW_HEADER_CLAW_types_h_
#define RTW_HEADER_CLAW_types_h_
#include "mode_tdcn_rtwtypes.h"
// #ifndef DEFINED_TYPEDEF_FOR_Anchor_point_
// #define DEFINED_TYPEDEF_FOR_Anchor_point_
// 
// typedef struct {
//   real32_T AP1[3];
//   real32_T AP2[3];
//   real32_T AP3[3];
//   real32_T AP4[3];
// } Anchor_point;
// 
// #endif
// 
// #ifndef DEFINED_TYPEDEF_FOR_Anchort2Tag_dist_
// #define DEFINED_TYPEDEF_FOR_Anchort2Tag_dist_
// 
// typedef struct {
//   real32_T A1;
//   real32_T A2;
//   real32_T A3;
//   real32_T A4;
// } Anchort2Tag_dist;
// 
// #endif

#ifndef DEFINED_TYPEDEF_FOR_Dest_poti_
#define DEFINED_TYPEDEF_FOR_Dest_poti_

/* Sejong: real32_T -> real_T (float -> double).
   이 구조체는 Cur_Pos / Dest_poti_i / cur_poti 세 곳에 쓰이는데, 앞의 둘은
   x=위도, y=경도 를 도(deg) 단위로 담는다.  float32 는 유효자리가 7자리뿐이라
   위경도 1도 근처에서 ULP 가 커진다 (위도 0.42 m, 경도 127도에서 0.675 m,
   149도에서 1.39 m).  cm 급 선박 착륙에서 그대로 위치 오차가 된다.
   double 로 올리면 ULP 가 나노미터 수준이 돼 사라진다.
   원본:
       real32_T x;
       real32_T y;
       real32_T z;                                                          */
typedef struct {
  real_T x;
  real_T y;
  real_T z;
} Dest_poti;

#endif

#ifndef DEFINED_TYPEDEF_FOR_DR_heading_
#define DEFINED_TYPEDEF_FOR_DR_heading_

typedef struct {
  real32_T DR_heading;
} DR_heading;

#endif

#ifndef DEFINED_TYPEDEF_FOR_V_cmd_
#define DEFINED_TYPEDEF_FOR_V_cmd_

typedef struct {
  real32_T cmd_roll;
  real32_T cmd_pitch;
  real32_T cmd_yaw;
  real32_T cmd_height;
} V_cmd;

#endif

#ifndef struct_tag_je9TdSm2yYb751inV09C2
#define struct_tag_je9TdSm2yYb751inV09C2

struct tag_je9TdSm2yYb751inV09C2
{
  int32_T isInitialized;
  boolean_T isSetupComplete;
  real_T pWinLen;
  real_T pBuf[5];
  real_T pHeap[5];
  real_T pMidHeap;
  real_T pIdx;
  real_T pPos[5];
  real_T pMinHeapLength;
  real_T pMaxHeapLength;
};

#endif                                 /* struct_tag_je9TdSm2yYb751inV09C2 */

#ifndef typedef_c_dsp_internal_MedianFilterCG_T
#define typedef_c_dsp_internal_MedianFilterCG_T

typedef struct tag_je9TdSm2yYb751inV09C2 c_dsp_internal_MedianFilterCG_T;

#endif                             /* typedef_c_dsp_internal_MedianFilterCG_T */

#ifndef struct_tag_BlgwLpgj2bjudmbmVKWwDE
#define struct_tag_BlgwLpgj2bjudmbmVKWwDE

struct tag_BlgwLpgj2bjudmbmVKWwDE
{
  uint32_T f1[8];
};

#endif                                 /* struct_tag_BlgwLpgj2bjudmbmVKWwDE */

#ifndef typedef_cell_wrap_CLAW_T
#define typedef_cell_wrap_CLAW_T

typedef struct tag_BlgwLpgj2bjudmbmVKWwDE cell_wrap_CLAW_T;

#endif                                 /* typedef_cell_wrap_CLAW_T */

#ifndef struct_tag_4kpgE3lCKM2mKMlNBIa8oB
#define struct_tag_4kpgE3lCKM2mKMlNBIa8oB

struct tag_4kpgE3lCKM2mKMlNBIa8oB
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  cell_wrap_CLAW_T inputVarSize;
  int32_T NumChannels;
  c_dsp_internal_MedianFilterCG_T pMID;
};

#endif                                 /* struct_tag_4kpgE3lCKM2mKMlNBIa8oB */

#ifndef typedef_dsp_simulink_MedianFilter_CLA_T
#define typedef_dsp_simulink_MedianFilter_CLA_T

typedef struct tag_4kpgE3lCKM2mKMlNBIa8oB dsp_simulink_MedianFilter_CLA_T;

#endif                             /* typedef_dsp_simulink_MedianFilter_CLA_T */

#ifndef struct_tag_UeG7yjL5UU0cwdA60Vt2WE
#define struct_tag_UeG7yjL5UU0cwdA60Vt2WE

struct tag_UeG7yjL5UU0cwdA60Vt2WE
{
  int32_T isInitialized;
  boolean_T isSetupComplete;
  real_T pWinLen;
  real_T pBuf[10];
  real_T pHeap[10];
  real_T pMidHeap;
  real_T pIdx;
  real_T pPos[10];
  real_T pMinHeapLength;
  real_T pMaxHeapLength;
};

#endif                                 /* struct_tag_UeG7yjL5UU0cwdA60Vt2WE */

#ifndef typedef_c_dsp_internal_MedianFilter_b_T
#define typedef_c_dsp_internal_MedianFilter_b_T

typedef struct tag_UeG7yjL5UU0cwdA60Vt2WE c_dsp_internal_MedianFilter_b_T;

#endif                             /* typedef_c_dsp_internal_MedianFilter_b_T */

#ifndef struct_tag_Bo0KMfG7xFU2NDU162cS9B
#define struct_tag_Bo0KMfG7xFU2NDU162cS9B

struct tag_Bo0KMfG7xFU2NDU162cS9B
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  cell_wrap_CLAW_T inputVarSize;
  int32_T NumChannels;
  c_dsp_internal_MedianFilter_b_T pMID;
};

#endif                                 /* struct_tag_Bo0KMfG7xFU2NDU162cS9B */

#ifndef typedef_dsp_simulink_MedianFilter_C_b_T
#define typedef_dsp_simulink_MedianFilter_C_b_T

typedef struct tag_Bo0KMfG7xFU2NDU162cS9B dsp_simulink_MedianFilter_C_b_T;

#endif                             /* typedef_dsp_simulink_MedianFilter_C_b_T */

/* Parameters (default storage) */
typedef struct P_CLAW_T_ P_CLAW_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_CLAW_T RT_MODEL_CLAW_T;

#endif                                 /* RTW_HEADER_CLAW_types_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
