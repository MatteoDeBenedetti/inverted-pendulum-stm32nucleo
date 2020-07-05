/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: velpos_controller.c
 *
 * Code generated for Simulink model 'velpos_controller'.
 *
 * Model version                  : 1.61
 * Simulink Coder version         : 8.14 (R2018a) 06-Feb-2018
 * C/C++ source code generated on : Tue Jun 12 19:26:17 2018
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "velpos_controller.h"
#include "velpos_controller_private.h"

/* Exported block signals */
real_T V_in;                           /* '<Root>/Saturation3' */

/* Block states (default storage) */
DW_velpos_controller_T velpos_controller_DW;

/* External inputs (root inport signals with default storage) */
ExtU_velpos_controller_T velpos_controller_U;

/* Real-time model */
RT_MODEL_velpos_controller_T velpos_controller_M_;
RT_MODEL_velpos_controller_T *const velpos_controller_M = &velpos_controller_M_;

/* Model step function */
void velpos_controller_step(void)
{
  real_T denAccum;
  real_T numAccum;
  real_T denAccum_0;
  real_T rtb_Sum5;
  real_T rtb_Gain1;
  real_T rtb_IntegralGain;
  real_T Integrator;

  /* Gain: '<S1>/Gain1' incorporates:
   *  Inport: '<Root>/In2'
   */
  rtb_Gain1 = 0.017453292519943295 * velpos_controller_U.In2;

  /* DiscreteTransferFcn: '<S4>/Discrete Transfer Fcn' */
  denAccum = rtb_Gain1 - -0.77880078307140488 *
    velpos_controller_DW.DiscreteTransferFcn_states;

  /* Gain: '<Root>/Gain1' incorporates:
   *  Gain: '<S2>/Gain1'
   *  Inport: '<Root>/In1'
   *  Sum: '<Root>/Sum4'
   */
  rtb_Sum5 = (0.017453292519943295 * velpos_controller_U.In1 - rtb_Gain1) * 0.5;

  /* Saturate: '<Root>/Saturation1' */
  if (rtb_Sum5 > 0.1) {
    rtb_Sum5 = 0.1;
  } else {
    if (rtb_Sum5 < -0.1) {
      rtb_Sum5 = -0.1;
    }
  }

  /* End of Saturate: '<Root>/Saturation1' */

  /* Sum: '<Root>/Sum5' incorporates:
   *  DiscreteTransferFcn: '<S4>/Discrete Transfer Fcn'
   */
  rtb_Sum5 -= 25.0 * denAccum + -25.0 *
    velpos_controller_DW.DiscreteTransferFcn_states;

  /* DiscreteTransferFcn: '<S7>/Filter Differentiator TF' incorporates:
   *  Gain: '<S6>/Derivative Gain'
   */
  denAccum_0 = 0.0 * rtb_Sum5 - -0.04712041884816752 *
    velpos_controller_DW.FilterDifferentiatorTF_states;
  numAccum = denAccum_0 + -velpos_controller_DW.FilterDifferentiatorTF_states;

  /* Gain: '<S6>/Integral Gain' */
  rtb_IntegralGain = 0.7 * rtb_Sum5;

  /* DiscreteIntegrator: '<S6>/Integrator' */
  Integrator = 0.005 * rtb_IntegralGain + velpos_controller_DW.Integrator_DSTATE;

  /* DiscreteTransferFcn: '<S5>/Discrete Transfer Fcn' */
  V_in = 0.048770575499285984 *
    velpos_controller_DW.DiscreteTransferFcn_states_c;

  /* Saturate: '<Root>/Saturation3' incorporates:
   *  DiscreteTransferFcn: '<S5>/Discrete Transfer Fcn'
   */
  if (V_in > 10.0) {
    V_in = 10.0;
  } else {
    if (V_in < -10.0) {
      V_in = -10.0;
    }
  }

  /* End of Saturate: '<Root>/Saturation3' */

  /* Update for DiscreteTransferFcn: '<S4>/Discrete Transfer Fcn' */
  velpos_controller_DW.DiscreteTransferFcn_states = denAccum;

  /* Update for DiscreteTransferFcn: '<S7>/Filter Differentiator TF' */
  velpos_controller_DW.FilterDifferentiatorTF_states = denAccum_0;

  /* Update for DiscreteIntegrator: '<S6>/Integrator' */
  velpos_controller_DW.Integrator_DSTATE = 0.005 * rtb_IntegralGain + Integrator;

  /* Update for DiscreteTransferFcn: '<S5>/Discrete Transfer Fcn' incorporates:
   *  DiscreteTransferFcn: '<S7>/Filter Differentiator TF'
   *  Gain: '<S3>/tau_grav'
   *  Gain: '<S6>/Proportional Gain'
   *  Gain: '<S7>/Filter Coefficient'
   *  Product: '<S7>/DenCoefOut'
   *  Sum: '<Root>/Sum3'
   *  Sum: '<S6>/Sum'
   *  Trigonometry: '<S3>/Sin'
   */
  velpos_controller_DW.DiscreteTransferFcn_states_c = (((5.0 * rtb_Sum5 +
    Integrator) + numAccum * 0.52356020942408377 * 182.0) + 1.1721611721611722 *
    sin(rtb_Gain1)) - -0.951229424500714 *
    velpos_controller_DW.DiscreteTransferFcn_states_c;
}

/* Model initialize function */
void velpos_controller_initialize(void)
{
  /* Registration code */

  /* initialize error status */
  rtmSetErrorStatus(velpos_controller_M, (NULL));

  /* block I/O */

  /* exported global signals */
  V_in = 0.0;

  /* states (dwork) */
  (void) memset((void *)&velpos_controller_DW, 0,
                sizeof(DW_velpos_controller_T));

  /* external inputs */
  (void)memset((void *)&velpos_controller_U, 0, sizeof(ExtU_velpos_controller_T));

  /* InitializeConditions for DiscreteTransferFcn: '<S7>/Filter Differentiator TF' */
  velpos_controller_DW.FilterDifferentiatorTF_states = 0.0;
}

/* Model terminate function */
void velpos_controller_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
