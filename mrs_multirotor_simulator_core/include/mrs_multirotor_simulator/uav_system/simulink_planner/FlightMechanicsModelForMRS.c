/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: FlightMechanicsModelForMRS.c
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

#include "FlightMechanicsModelForMRS.h"
#include <math.h>
#include "rtwtypes.h"

/* Exported data definition */

/* Definition for custom storage class: Struct */
AircraftParameters_type AircraftParameters = {
  /* AoA_max */
  10.0,

  /* AoA_min */
  -5.0,

  /* wing_area */
  4.0,

  /* T2W_ratio */
  0.5,

  /* cla */
  3.0,

  /* g */
  9.81,

  /* m */
  50.0,

  /* pitch_tau */
  0.25,

  /* rho */
  1.225,

  /* roll_tau */
  0.25,

  /* speed_tau */
  5.0,

  /* v_max */
  50.0,

  /* v_stall */
  20.0
};

/* Block states (default storage) */
DW_FlightMechanicsModelForMRS_T FlightMechanicsModelForMRS_DW;

/* External inputs (root inport signals with default storage) */
ExtU_FlightMechanicsModelForM_T FlightMechanicsModelForMRS_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_FlightMechanicsModelForM_T FlightMechanicsModelForMRS_Y;

/* Real-time model */
static RT_MODEL_FlightMechanicsModel_T FlightMechanicsModelForMRS_M_;
RT_MODEL_FlightMechanicsModel_T *const FlightMechanicsModelForMRS_M =
  &FlightMechanicsModelForMRS_M_;

/* Model step function */
void FlightMechanicsModelForMRS_step(void)
{
  real_T rtb_Gain1;
  real_T rtb_Gain2;
  real_T rtb_Gain_pl;
  real_T rtb_MaximalRoll;
  real_T rtb_MinimalPitch;
  real_T rtb_SpeedX;
  real_T rtb_SpeedY;
  real_T rtb_VerticalSpeed;
  real_T rtb_VerticalSpeed_tmp;

  /* Stop: '<Root>/Stop Simulation' incorporates:
   *  Constant: '<S2>/Constant'
   *  DiscreteIntegrator: '<Root>/SpeedState'
   *  RelationalOperator: '<S2>/Compare'
   */
  if (FlightMechanicsModelForMRS_DW.SpeedState_DSTATE <
      AircraftParameters.v_stall) {
    rtmSetStopRequested(FlightMechanicsModelForMRS_M, 1);
  }

  /* End of Stop: '<Root>/Stop Simulation' */

  /* Math: '<Root>/Square' incorporates:
   *  DiscreteIntegrator: '<Root>/SpeedState'
   *  Math: '<S1>/Square'
   */
  rtb_SpeedX = FlightMechanicsModelForMRS_DW.SpeedState_DSTATE *
    FlightMechanicsModelForMRS_DW.SpeedState_DSTATE;

  /* Product: '<Root>/Product5' incorporates:
   *  Constant: '<Root>/Constant2'
   *  Math: '<Root>/Square'
   */
  rtb_MinimalPitch = AircraftParameters.rho * 0.5 * rtb_SpeedX;

  /* Trigonometry: '<Root>/MaximalRoll' incorporates:
   *  Constant: '<Root>/Constant3'
   *  Constant: '<Root>/Constant4'
   *  Product: '<Root>/Divide1'
   *  Sum: '<Root>/Sum6'
   */
  rtb_MaximalRoll = atan(rtb_MinimalPitch / (0.5 * AircraftParameters.rho *
    AircraftParameters.v_stall * AircraftParameters.v_stall) - 1.0);

  /* Gain: '<S4>/Gain1' incorporates:
   *  Inport generated from: '<Root>/BusElementIn'
   */
  rtb_Gain1 = 0.017453292519943295 *
    FlightMechanicsModelForMRS_U.Commands_RollRef;

  /* Switch: '<S11>/Switch2' incorporates:
   *  Gain: '<Root>/MinimalRoll'
   *  RelationalOperator: '<S11>/LowerRelop1'
   *  RelationalOperator: '<S11>/UpperRelop'
   *  Switch: '<S11>/Switch'
   */
  if (rtb_Gain1 > rtb_MaximalRoll) {
    rtb_Gain1 = rtb_MaximalRoll;
  } else if (rtb_Gain1 < -rtb_MaximalRoll) {
    /* Switch: '<S11>/Switch' incorporates:
     *  Gain: '<Root>/MinimalRoll'
     */
    rtb_Gain1 = -rtb_MaximalRoll;
  }

  /* End of Switch: '<S11>/Switch2' */

  /* Gain: '<Root>/Gain4' incorporates:
   *  DiscreteIntegrator: '<Root>/RollAngleState'
   *  Sum: '<Root>/Sum5'
   */
  rtb_MaximalRoll = 1.0 / AircraftParameters.roll_tau * (rtb_Gain1 -
    FlightMechanicsModelForMRS_DW.RollAngleState_DSTATE);

  /* Gain: '<Root>/Gain1' incorporates:
   *  Constant: '<S1>/Constant'
   *  Constant: '<S1>/Constant1'
   *  DiscreteIntegrator: '<Root>/FlypathAngleState'
   *  DiscreteIntegrator: '<Root>/PitchAngleState'
   *  DiscreteIntegrator: '<Root>/RollAngleState'
   *  Gain: '<S12>/Gain1'
   *  Gain: '<S1>/Gain'
   *  Gain: '<S1>/Gain1'
   *  Product: '<Root>/Product4'
   *  Product: '<S1>/Divide'
   *  Product: '<S1>/Divide1'
   *  Sum: '<Root>/Sum2'
   *  Sum: '<Root>/Sum7'
   *  Sum: '<S1>/Sum'
   *  Sum: '<S1>/Sum1'
   *  Trigonometry: '<S1>/Cos'
   */
  rtb_Gain1 = ((FlightMechanicsModelForMRS_DW.PitchAngleState_DSTATE -
                (0.017453292519943295 * AircraftParameters.AoA_max - (rtb_SpeedX
    - AircraftParameters.v_stall * AircraftParameters.v_stall) *
                 (AircraftParameters.g * AircraftParameters.m) / (0.5 *
    AircraftParameters.rho * AircraftParameters.wing_area *
    AircraftParameters.cla * AircraftParameters.v_stall *
    AircraftParameters.v_stall * (rtb_SpeedX / cos
    (FlightMechanicsModelForMRS_DW.RollAngleState_DSTATE))))) -
               FlightMechanicsModelForMRS_DW.FlypathAngleState_DSTATE) *
    rtb_MinimalPitch * (AircraftParameters.cla * AircraftParameters.wing_area /
                        AircraftParameters.m);

  /* Product: '<Root>/HorizontalSpeed' incorporates:
   *  DiscreteIntegrator: '<Root>/FlypathAngleState'
   *  DiscreteIntegrator: '<Root>/SpeedState'
   *  Trigonometry: '<Root>/Trigonometric Function1'
   */
  rtb_MinimalPitch = cos(FlightMechanicsModelForMRS_DW.FlypathAngleState_DSTATE)
    * FlightMechanicsModelForMRS_DW.SpeedState_DSTATE;

  /* Product: '<Root>/SpeedX' incorporates:
   *  DiscreteIntegrator: '<Root>/HeadingState'
   *  Trigonometry: '<Root>/Trigonometric Function4'
   */
  rtb_SpeedX = rtb_MinimalPitch * cos
    (FlightMechanicsModelForMRS_DW.HeadingState_DSTATE);

  /* Product: '<Root>/SpeedY' incorporates:
   *  DiscreteIntegrator: '<Root>/HeadingState'
   *  Trigonometry: '<Root>/Trigonometric Function3'
   */
  rtb_SpeedY = rtb_MinimalPitch * sin
    (FlightMechanicsModelForMRS_DW.HeadingState_DSTATE);

  /* Trigonometry: '<Root>/Trigonometric Function5' incorporates:
   *  DiscreteIntegrator: '<Root>/FlypathAngleState'
   *  Trigonometry: '<Root>/Sin'
   */
  rtb_VerticalSpeed_tmp = sin
    (FlightMechanicsModelForMRS_DW.FlypathAngleState_DSTATE);

  /* Product: '<Root>/VerticalSpeed' incorporates:
   *  DiscreteIntegrator: '<Root>/SpeedState'
   *  Trigonometry: '<Root>/Trigonometric Function5'
   */
  rtb_VerticalSpeed = rtb_VerticalSpeed_tmp *
    FlightMechanicsModelForMRS_DW.SpeedState_DSTATE;

  /* Gain: '<S6>/Gain' incorporates:
   *  DiscreteIntegrator: '<Root>/FlypathAngleState'
   */
  rtb_Gain_pl = 57.295779513082323 *
    FlightMechanicsModelForMRS_DW.FlypathAngleState_DSTATE;

  /* Sum: '<Root>/MaximalPitch' incorporates:
   *  Constant: '<Root>/Constant'
   */
  rtb_MinimalPitch = AircraftParameters.AoA_max + rtb_Gain_pl;

  /* Switch: '<S5>/Switch2' incorporates:
   *  Inport generated from: '<Root>/Bus Element In'
   *  RelationalOperator: '<S5>/LowerRelop1'
   */
  if (!(FlightMechanicsModelForMRS_U.Commands_PitchRef > rtb_MinimalPitch)) {
    /* Sum: '<Root>/MinimalPitch' incorporates:
     *  Constant: '<Root>/Constant1'
     */
    rtb_MinimalPitch = rtb_Gain_pl + AircraftParameters.AoA_min;

    /* Switch: '<S5>/Switch' incorporates:
     *  RelationalOperator: '<S5>/UpperRelop'
     */
    if (!(FlightMechanicsModelForMRS_U.Commands_PitchRef < rtb_MinimalPitch)) {
      rtb_MinimalPitch = FlightMechanicsModelForMRS_U.Commands_PitchRef;
    }

    /* End of Switch: '<S5>/Switch' */
  }

  /* End of Switch: '<S5>/Switch2' */

  /* Gain: '<Root>/Gain3' incorporates:
   *  DiscreteIntegrator: '<Root>/PitchAngleState'
   *  Gain: '<S3>/Gain1'
   *  Sum: '<Root>/Sum4'
   */
  rtb_Gain_pl = (0.017453292519943295 * rtb_MinimalPitch -
                 FlightMechanicsModelForMRS_DW.PitchAngleState_DSTATE) * (1.0 /
    AircraftParameters.pitch_tau);

  /* Sum: '<Root>/Sum' incorporates:
   *  Constant: '<Root>/Constant6'
   */
  rtb_MinimalPitch = AircraftParameters.T2W_ratio - rtb_VerticalSpeed_tmp;

  /* Saturate: '<Root>/Saturation2' incorporates:
   *  Inport generated from: '<Root>/BusElementIn1'
   */
  if (FlightMechanicsModelForMRS_U.Commands_VinfRef > AircraftParameters.v_max)
  {
    rtb_Gain2 = AircraftParameters.v_max;
  } else if (FlightMechanicsModelForMRS_U.Commands_VinfRef <
             AircraftParameters.v_stall) {
    rtb_Gain2 = AircraftParameters.v_stall;
  } else {
    rtb_Gain2 = FlightMechanicsModelForMRS_U.Commands_VinfRef;
  }

  /* Gain: '<Root>/Gain2' incorporates:
   *  DiscreteIntegrator: '<Root>/SpeedState'
   *  Saturate: '<Root>/Saturation2'
   *  Sum: '<Root>/Sum3'
   */
  rtb_Gain2 = 1.0 / AircraftParameters.speed_tau * (rtb_Gain2 -
    FlightMechanicsModelForMRS_DW.SpeedState_DSTATE);

  /* Switch: '<S10>/Switch2' incorporates:
   *  RelationalOperator: '<S10>/LowerRelop1'
   */
  if (rtb_Gain2 > rtb_MinimalPitch) {
    /* Switch: '<S10>/Switch2' */
    rtb_Gain2 = rtb_MinimalPitch;
  } else {
    /* Sum: '<Root>/Sum1' incorporates:
     *  Constant: '<Root>/Constant6'
     */
    rtb_MinimalPitch = (0.0 - AircraftParameters.T2W_ratio) -
      rtb_VerticalSpeed_tmp;

    /* Switch: '<S10>/Switch' incorporates:
     *  RelationalOperator: '<S10>/UpperRelop'
     */
    if (rtb_Gain2 < rtb_MinimalPitch) {
      /* Switch: '<S10>/Switch2' */
      rtb_Gain2 = rtb_MinimalPitch;
    }

    /* End of Switch: '<S10>/Switch' */
  }

  /* End of Switch: '<S10>/Switch2' */

  /* Gain: '<Root>/Gain' incorporates:
   *  DiscreteIntegrator: '<Root>/RollAngleState'
   *  DiscreteIntegrator: '<Root>/SpeedState'
   *  Product: '<Root>/Divide'
   *  Trigonometry: '<Root>/Trigonometric Function'
   */
  rtb_MinimalPitch = tan(FlightMechanicsModelForMRS_DW.RollAngleState_DSTATE) /
    FlightMechanicsModelForMRS_DW.SpeedState_DSTATE * 9.81;

  /* Outport generated from: '<Root>/Bus Element Out' incorporates:
   *  DiscreteIntegrator: '<Root>/XPositionState'
   */
  FlightMechanicsModelForMRS_Y.States_X =
    FlightMechanicsModelForMRS_DW.XPositionState_DSTATE;

  /* Outport generated from: '<Root>/Bus Element Out1' incorporates:
   *  DiscreteIntegrator: '<Root>/YPositionState'
   */
  FlightMechanicsModelForMRS_Y.States_Y =
    FlightMechanicsModelForMRS_DW.YPositionState_DSTATE;

  /* Outport generated from: '<Root>/Bus Element Out2' incorporates:
   *  DiscreteIntegrator: '<Root>/AltitudeState'
   */
  FlightMechanicsModelForMRS_Y.States_Z =
    FlightMechanicsModelForMRS_DW.AltitudeState_DSTATE;

  /* Outport generated from: '<Root>/Bus Element Out3' */
  FlightMechanicsModelForMRS_Y.States_VX = rtb_SpeedX;

  /* Outport generated from: '<Root>/Bus Element Out4' */
  FlightMechanicsModelForMRS_Y.States_VY = rtb_SpeedY;

  /* Outport generated from: '<Root>/Bus Element Out5' */
  FlightMechanicsModelForMRS_Y.States_VZ = rtb_VerticalSpeed;

  /* Outport generated from: '<Root>/Bus Element Out7' incorporates:
   *  DiscreteIntegrator: '<Root>/RollAngleState'
   *  Gain: '<S9>/Gain'
   */
  FlightMechanicsModelForMRS_Y.States_Roll = 57.295779513082323 *
    FlightMechanicsModelForMRS_DW.RollAngleState_DSTATE;

  /* Outport generated from: '<Root>/Bus Element Out6' incorporates:
   *  DiscreteIntegrator: '<Root>/PitchAngleState'
   *  Gain: '<S8>/Gain'
   */
  FlightMechanicsModelForMRS_Y.States_Pitch = 57.295779513082323 *
    FlightMechanicsModelForMRS_DW.PitchAngleState_DSTATE;

  /* Outport generated from: '<Root>/Bus Element Out8' incorporates:
   *  DiscreteIntegrator: '<Root>/HeadingState'
   *  Gain: '<S7>/Gain'
   */
  FlightMechanicsModelForMRS_Y.States_Heading = 57.295779513082323 *
    FlightMechanicsModelForMRS_DW.HeadingState_DSTATE;

  /* Update for DiscreteIntegrator: '<Root>/SpeedState' */
  FlightMechanicsModelForMRS_DW.SpeedState_DSTATE += 0.001 * rtb_Gain2;

  /* Saturate: '<Root>/Saturation1' */
  if (rtb_MaximalRoll > 0.52359877559829882) {
    rtb_MaximalRoll = 0.52359877559829882;
  } else if (rtb_MaximalRoll < -0.52359877559829882) {
    rtb_MaximalRoll = -0.52359877559829882;
  }

  /* Update for DiscreteIntegrator: '<Root>/RollAngleState' incorporates:
   *  Saturate: '<Root>/Saturation1'
   */
  FlightMechanicsModelForMRS_DW.RollAngleState_DSTATE += 0.001 * rtb_MaximalRoll;

  /* Saturate: '<Root>/Saturation' */
  if (rtb_Gain_pl > 0.52359877559829882) {
    rtb_Gain_pl = 0.52359877559829882;
  } else if (rtb_Gain_pl < -0.52359877559829882) {
    rtb_Gain_pl = -0.52359877559829882;
  }

  /* Update for DiscreteIntegrator: '<Root>/PitchAngleState' incorporates:
   *  Saturate: '<Root>/Saturation'
   */
  FlightMechanicsModelForMRS_DW.PitchAngleState_DSTATE += 0.001 * rtb_Gain_pl;

  /* Update for DiscreteIntegrator: '<Root>/FlypathAngleState' */
  FlightMechanicsModelForMRS_DW.FlypathAngleState_DSTATE += 0.001 * rtb_Gain1;

  /* Update for DiscreteIntegrator: '<Root>/HeadingState' */
  FlightMechanicsModelForMRS_DW.HeadingState_DSTATE += 0.001 * rtb_MinimalPitch;

  /* Update for DiscreteIntegrator: '<Root>/AltitudeState' */
  FlightMechanicsModelForMRS_DW.AltitudeState_DSTATE += 0.001 *
    rtb_VerticalSpeed;

  /* Update for DiscreteIntegrator: '<Root>/XPositionState' */
  FlightMechanicsModelForMRS_DW.XPositionState_DSTATE += 0.001 * rtb_SpeedX;

  /* Update for DiscreteIntegrator: '<Root>/YPositionState' */
  FlightMechanicsModelForMRS_DW.YPositionState_DSTATE += 0.001 * rtb_SpeedY;
}

/* Model initialize function */
void FlightMechanicsModelForMRS_initialize(void)
{
  /* InitializeConditions for DiscreteIntegrator: '<Root>/SpeedState' */
  FlightMechanicsModelForMRS_DW.SpeedState_DSTATE = 30.0;
}

/* Model terminate function */
void FlightMechanicsModelForMRS_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
