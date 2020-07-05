/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: velpos_controller.h
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

#ifndef RTW_HEADER_velpos_controller_h_
#define RTW_HEADER_velpos_controller_h_
#include <math.h>
#include <string.h>
#include <stddef.h>
#ifndef velpos_controller_COMMON_INCLUDES_
# define velpos_controller_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* velpos_controller_COMMON_INCLUDES_ */

#include "velpos_controller_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T DiscreteTransferFcn_states;   /* '<S4>/Discrete Transfer Fcn' */
  real_T FilterDifferentiatorTF_states;/* '<S7>/Filter Differentiator TF' */
  real_T Integrator_DSTATE;            /* '<S6>/Integrator' */
  real_T DiscreteTransferFcn_states_c; /* '<S5>/Discrete Transfer Fcn' */
} DW_velpos_controller_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T In1;                          /* '<Root>/In1' */
  real_T In2;                          /* '<Root>/In2' */
} ExtU_velpos_controller_T;

/* Real-time Model Data Structure */
struct tag_RTM_velpos_controller_T {
  const char_T * volatile errorStatus;
};

/* Block states (default storage) */
extern DW_velpos_controller_T velpos_controller_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_velpos_controller_T velpos_controller_U;

/*
 * Exported Global Signals
 *
 * Note: Exported global signals are block signals with an exported global
 * storage class designation.  Code generation will declare the memory for
 * these signals and export their symbols.
 *
 */
extern real_T V_in;                    /* '<Root>/Saturation3' */

/* Model entry point functions */
extern void velpos_controller_initialize(void);
extern void velpos_controller_step(void);
extern void velpos_controller_terminate(void);

/* Real-time Model object */
extern RT_MODEL_velpos_controller_T *const velpos_controller_M;

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
 * '<Root>' : 'velpos_controller'
 * '<S1>'   : 'velpos_controller/Degrees to Radians2'
 * '<S2>'   : 'velpos_controller/Degrees to Radians3'
 * '<S3>'   : 'velpos_controller/GravityController1'
 * '<S4>'   : 'velpos_controller/Transfer Fcn2'
 * '<S5>'   : 'velpos_controller/Transfer Fcn3'
 * '<S6>'   : 'velpos_controller/VelController1'
 * '<S7>'   : 'velpos_controller/VelController1/Filter Differentiator'
 */
#endif                                 /* RTW_HEADER_velpos_controller_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
