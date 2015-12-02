/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mc_att_control_params.c
 * Parameters for multicopter attitude controller.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Anton Babushkin <anton@px4.io>
 */

/**
 * Roll P gain
 *
 * Roll proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @min 0.0
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ROLL_P, 6.5f);

/**
 * Roll rate P gain
 *
 * Roll rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.0
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_P, 0.15f);

/**
 * Roll rate I gain
 *
 * Roll rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_I, 0.05f);

/**
 * Roll rate D gain
 *
 * Roll rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_D, 0.003f);

/**
 * Roll rate feedforward
 *
 * Improves tracking performance.
 *
 * @min 0.0
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_FF, 0.0f);

/**
 * Pitch P gain
 *
 * Pitch proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @unit 1/s
 * @min 0.0
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_PITCH_P, 6.5f);

/**
 * Pitch rate P gain
 *
 * Pitch rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.0
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_P, 0.15f);

/**
 * Pitch rate I gain
 *
 * Pitch rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_I, 0.05f);

/**
 * Pitch rate D gain
 *
 * Pitch rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_D, 0.003f);

/**
 * Pitch rate feedforward
 *
 * Improves tracking performance.
 *
 * @min 0.0
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_FF, 0.0f);

/**
 * Yaw P gain
 *
 * Yaw proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @unit 1/s
 * @min 0.0
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_YAW_P, 2.8f);

/**
 * Yaw rate P gain
 *
 * Yaw rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.0
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_P, 0.2f);

/**
 * Yaw rate I gain
 *
 * Yaw rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_I, 0.1f);

/**
 * Yaw rate D gain
 *
 * Yaw rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_D, 0.0f);

/**
 * Yaw rate feedforward
 *
 * Improves tracking performance.
 *
 * @min 0.0
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_FF, 0.0f);

/**
 * Yaw feed forward
 *
 * Feed forward weight for manual yaw control. 0 will give slow responce and no overshot, 1 - fast responce and big overshot.
 *
 * @min 0.0
 * @max 1.0
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_YAW_FF, 0.5f);

/**
 * Max roll rate
 *
 * Limit for roll rate, has effect for large rotations in autonomous mode, to avoid large control output and mixer saturation.
 *
 * @unit deg/s
 * @min 0.0
 * @max 360.0
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_MAX, 220.0f);

/**
 * Max pitch rate
 *
 * Limit for pitch rate, has effect for large rotations in autonomous mode, to avoid large control output and mixer saturation.
 *
 * @unit deg/s
 * @min 0.0
 * @max 360.0
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_MAX, 220.0f);

/**
 * Max yaw rate
 *
 * Limit for yaw rate, has effect for large rotations in autonomous mode,
 * to avoid large control output and mixer saturation. A value of significantly
 * over 60 degrees per second can already lead to mixer saturation.
 *
 * @unit deg/s
 * @min 0.0
 * @max 360.0
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_MAX, 60.0f);

/**
 * Max acro roll rate
 *
 * @unit deg/s
 * @min 0.0
 * @max 360.0
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ACRO_R_MAX, 360.0f);

/**
 * Max acro pitch rate
 *
 * @unit deg/s
 * @min 0.0
 * @max 360.0
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ACRO_P_MAX, 360.0f);

/**
 * Max acro yaw rate
 *
 * @unit deg/s
 * @min 0.0
 * @group Multicopter Attitude Control
 */
PARAM_DEFINE_FLOAT(MC_ACRO_Y_MAX, 360.0f);

/**
 * Threshold for Rattitude mode
 * 
 * Manual input needed in order to override attitude control rate setpoints
 * and instead pass manual stick inputs as rate setpoints
 * 
 * @unit 
 * @min 0.0
 * @max 1.0
 * @group Multicopter Attitude Control
 */
 PARAM_DEFINE_FLOAT(MC_RATT_TH, 1.0f);

/**
* Parameter to enable L1 adaptive control
*
* This performs L1 adaptive estimation of the disturbance of the system
* and compensates for it leading to better attitude hold
*
* @min 0
* @max 1
* @group Multicoptor Attitude Control
*/
PARAM_DEFINE_INT32(MC_ENABLE_L1, 0);

/**
* Parameter to enable L1 adaptive control debug information
*
* @min 0
* @max 1
* @group Multicoptor Attitude Control
*/
PARAM_DEFINE_INT32(L1_ENABLE_DEBUG, 0);

/**
* Moment of Inertia about XX
*
* @unit N m^2
* @min 0.0
* @max 10.0
* @group Multicoptor Attitude Control
*/
PARAM_DEFINE_FLOAT(L1_INERTIA_XX, 0.0f);

/**
* Moment of Inertia about XY
*
* @unit N m^2
* @min 0.0
* @max 10.0
* @group Multicoptor Attitude Control
*/
PARAM_DEFINE_FLOAT(L1_INERTIA_XY, 0.0f);

/**
* Moment of Inertia about XZ
*
* @unit N m^2
* @min 0.0
* @max 10.0
* @group Multicoptor Attitude Control
*/
PARAM_DEFINE_FLOAT(L1_INERTIA_XZ, 0.0f);

/**
* Moment of Inertia about YY
*
* @unit N m^2
* @min 0.0
* @max 10.0
* @group Multicoptor Attitude Control
*/
PARAM_DEFINE_FLOAT(L1_INERTIA_YY, 0.0f);

/**
* Moment of Inertia about YZ
*
* @unit N m^2
* @min 0.0
* @max 10.0
* @group Multicoptor Attitude Control
*/
PARAM_DEFINE_FLOAT(L1_INERTIA_YZ, 0.0f);

/**
* Moment of Inertia about ZZ
*
* @unit N m^2
* @min 0.0
* @max 10.0
* @group Multicoptor Attitude Control
*/
PARAM_DEFINE_FLOAT(L1_INERTIA_ZZ, 0.0f);

/**
* Control Bandwith in X
*
* @unit ??
* @min 0.0
* @max 1000.0
* @group Multicoptor Attitude Control
*/
PARAM_DEFINE_FLOAT(L1_BANDWIDTH_X, 0.0f);

/**
* Control Bandwidth in Y
*
* @unit ??
* @min 0.0
* @max 1000.0
* @group Multicoptor Attitude Control
*/
PARAM_DEFINE_FLOAT(L1_BANDWIDTH_Y, 0.0f);

/**
* Control Bandwidth in Z
*
* @unit ??
* @min 0.0
* @max 1000.0
* @group Multicoptor Attitude Control
*/
PARAM_DEFINE_FLOAT(L1_BANDWIDTH_Z, 0.0f);

/**
* Gain on state predictor in X
*
* @min 0.0
* @max 1000.0
* @group Multicoptor Attitude Control
*/
PARAM_DEFINE_FLOAT(L1_KSP_X, 400.0f);

/**
* Gain on state predictor in Y
*
* @min 0.0
* @max 1000.0
* @group Multicoptor Attitude Control
*/
PARAM_DEFINE_FLOAT(L1_KSP_Y, 400.0f);

/**
* Gain on state predictor in Z
*
* @unit ??
* @min 0.0
* @max 1000.0
* @group Multicoptor Attitude Control
*/
PARAM_DEFINE_FLOAT(L1_KSP_Z, 400.0f);

/**
* Adaptation gain
*
* @min 0.0
* @max 10000.0
* @group Multicoptor Attitude Control
*/
PARAM_DEFINE_FLOAT(L1_GAMMA, 5000.0f);

/**
* Initial disturbance estimate in X
*
* @unit N m
* @min 0.0
* @max 100.0
* @group Multicoptor Attitude Control
*/
PARAM_DEFINE_FLOAT(L1_INIT_DISTX, 0.0f);

/**
* Initial disturbance estimate in Y
*
* @unit N m
* @min 0.0
* @max 100.0
* @group Multicoptor Attitude Control
*/
PARAM_DEFINE_FLOAT(L1_INIT_DISTY, 0.0f);

/**
* Initial disturbance estimate in Z
*
* @unit N m
* @min 0.0
* @max 100.0
* @group Multicoptor Attitude Control
*/
PARAM_DEFINE_FLOAT(L1_INIT_DISTZ, 0.0f);

/**
* Threshold on total force before which L1 is initialized
*
* This is a fraction of the weight of the vehicle
*
* @min 0.0
* @max 1.0
* @group Multicoptor Attitude Control
*/
PARAM_DEFINE_FLOAT(L1_ENGAGE_LEVEL, 0.3f);
