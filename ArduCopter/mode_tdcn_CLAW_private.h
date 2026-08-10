/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: CLAW_private.h
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

#ifndef RTW_HEADER_CLAW_private_h_
#define RTW_HEADER_CLAW_private_h_
#include "mode_tdcn_rtwtypes.h"
#include "mode_tdcn_CLAW.h"
#include "mode_tdcn_CLAW_types.h"

extern void CLAW_MedianFilter_Init(DW_MedianFilter_CLAW_T *localDW);
extern void CLAW_MedianFilter(real_T rtu_0, B_MedianFilter_CLAW_T *localB,
  DW_MedianFilter_CLAW_T *localDW);
extern void CLAW_MedianFilter_Term(DW_MedianFilter_CLAW_T *localDW);

#endif                                 /* RTW_HEADER_CLAW_private_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
