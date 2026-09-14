/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: FlightMechanicsModelForMRS.h
 *
 * Code generated for Simulink model 'FlightMechanicsModelForMRS'.
 *
 * Model version                  : 5.30
 * Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
 * C/C++ source code generated on : Fri Aug  1 17:18:14 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef FlightMechanicsModelForMRS_h_
#define FlightMechanicsModelForMRS_h_
#ifndef FlightMechanicsModelForMRS_COMMON_INCLUDES_
#define FlightMechanicsModelForMRS_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "math.h"
#endif                         /* FlightMechanicsModelForMRS_COMMON_INCLUDES_ */

#include "FlightMechanicsModelForMRS_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T SpeedState_DSTATE;            /* '<Root>/SpeedState' */
  real_T RollAngleState_DSTATE;        /* '<Root>/RollAngleState' */
  real_T PitchAngleState_DSTATE;       /* '<Root>/PitchAngleState' */
  real_T FlypathAngleState_DSTATE;     /* '<Root>/FlypathAngleState' */
  real_T HeadingState_DSTATE;          /* '<Root>/HeadingState' */
  real_T AltitudeState_DSTATE;         /* '<Root>/AltitudeState' */
  real_T XPositionState_DSTATE;        /* '<Root>/XPositionState' */
  real_T YPositionState_DSTATE;        /* '<Root>/YPositionState' */
} DW_FlightMechanicsModelForMRS_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T Commands_PitchRef;            /* '<Root>/Commands_PitchRef' */
  real_T Commands_RollRef;             /* '<Root>/Commands_RollRef' */
  real_T Commands_VinfRef;             /* '<Root>/Commands_VinfRef' */
} ExtU_FlightMechanicsModelForM_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T States_X;                     /* '<Root>/States_X' */
  real_T States_Y;                     /* '<Root>/States_Y' */
  real_T States_Z;                     /* '<Root>/States_Z' */
  real_T States_VX;                    /* '<Root>/States_VX' */
  real_T States_VY;                    /* '<Root>/States_VY' */
  real_T States_VZ;                    /* '<Root>/States_VZ' */
  real_T States_Roll;                  /* '<Root>/States_Roll' */
  real_T States_Pitch;                 /* '<Root>/States_Pitch' */
  real_T States_Heading;               /* '<Root>/States_Heading' */
} ExtY_FlightMechanicsModelForM_T;

/* Type definition for custom storage class: Struct */
typedef struct AircraftParameters_tag {
  real_T AoA_max;                      /* Referenced by:
                                        * '<Root>/Constant'
                                        * '<S1>/Constant1'
                                        */
  real_T AoA_min;                      /* Referenced by: '<Root>/Constant1' */
  real_T wing_area;                    /* Referenced by:
                                        * '<Root>/Gain1'
                                        * '<S1>/Gain'
                                        */
  real_T T2W_ratio;                    /* Referenced by: '<Root>/Constant6' */

  /* Thrust to weight ratio */
  real_T cla;                          /* Referenced by:
                                        * '<Root>/Gain1'
                                        * '<S1>/Gain'
                                        */
  real_T g;                            /* Referenced by: '<S1>/Gain1' */
  real_T m;                            /* Referenced by:
                                        * '<Root>/Gain1'
                                        * '<S1>/Gain1'
                                        */
  real_T pitch_tau;                    /* Referenced by: '<Root>/Gain3' */
  real_T rho;                          /* Referenced by:
                                        * '<Root>/Constant2'
                                        * '<Root>/Constant3'
                                        * '<S1>/Gain'
                                        */
  real_T roll_tau;                     /* Referenced by: '<Root>/Gain4' */
  real_T speed_tau;                    /* Referenced by: '<Root>/Gain2' */
  real_T v_max;                        /* Referenced by: '<Root>/Saturation2' */
  real_T v_stall;                      /* Referenced by:
                                        * '<Root>/Constant3'
                                        * '<Root>/Saturation2'
                                        * '<S1>/Constant'
                                        * '<S1>/Gain'
                                        * '<S2>/Constant'
                                        */
} AircraftParameters_type;

/* Real-time Model Data Structure */
struct tag_RTM_FlightMechanicsModelF_T {
  const char_T * volatile errorStatus;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    boolean_T stopRequestedFlag;
  } Timing;
};

/* Block states (default storage) */
extern DW_FlightMechanicsModelForMRS_T FlightMechanicsModelForMRS_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_FlightMechanicsModelForM_T FlightMechanicsModelForMRS_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_FlightMechanicsModelForM_T FlightMechanicsModelForMRS_Y;

/* Model entry point functions */
extern void FlightMechanicsModelForMRS_initialize(void);
extern void FlightMechanicsModelForMRS_step(void);
extern void FlightMechanicsModelForMRS_terminate(void);

/* Exported data declaration */

/* Declaration for custom storage class: Struct */
extern AircraftParameters_type AircraftParameters;

/* Real-time Model object */
extern RT_MODEL_FlightMechanicsModel_T *const FlightMechanicsModelForMRS_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S5>/Data Type Duplicate' : Unused code path elimination
 * Block '<S5>/Data Type Propagation' : Unused code path elimination
 * Block '<S10>/Data Type Duplicate' : Unused code path elimination
 * Block '<S10>/Data Type Propagation' : Unused code path elimination
 * Block '<S11>/Data Type Duplicate' : Unused code path elimination
 * Block '<S11>/Data Type Propagation' : Unused code path elimination
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
 * '<Root>' : 'FlightMechanicsModelForMRS'
 * '<S1>'   : 'FlightMechanicsModelForMRS/Alpha_trim'
 * '<S2>'   : 'FlightMechanicsModelForMRS/Compare To Constant'
 * '<S3>'   : 'FlightMechanicsModelForMRS/Degrees to Radians'
 * '<S4>'   : 'FlightMechanicsModelForMRS/Degrees to Radians1'
 * '<S5>'   : 'FlightMechanicsModelForMRS/PitchSaturationDynamic'
 * '<S6>'   : 'FlightMechanicsModelForMRS/Radians to Degrees'
 * '<S7>'   : 'FlightMechanicsModelForMRS/Radians to Degrees1'
 * '<S8>'   : 'FlightMechanicsModelForMRS/Radians to Degrees3'
 * '<S9>'   : 'FlightMechanicsModelForMRS/Radians to Degrees4'
 * '<S10>'  : 'FlightMechanicsModelForMRS/Saturation Dynamic'
 * '<S11>'  : 'FlightMechanicsModelForMRS/Saturation Dynamic1'
 * '<S12>'  : 'FlightMechanicsModelForMRS/Alpha_trim/Degrees to Radians'
 */
#endif                                 /* FlightMechanicsModelForMRS_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
