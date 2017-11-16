
#include "stabilizer.h"
#include "stabilizer_types.h"

#include "sensfusion6.h"

#include "motors.h"

#include "log.h"
#include "param.h"
#include "num.h"

#include "FreeRTOS.h"
#include "arm_math.h"

#include "adaptive_controller.h"

#define DEG_TO_RAD (PI/180.0f)
#define RAD_TO_DEG (180.0f/PI)
#define limitThrust(VAL) limitUint16(VAL)

typedef struct motorPower_s{
  uint16_t m1;
  uint16_t m2;
  uint16_t m3;
  uint16_t m4;
} motorPower_t;

static bool motorSetEnable = false;

// LQR gains
static float lqr_kp     = 5258.05006548232f;
static float lqr_kq     = 5258.05006548232f;
static float lqr_kr     = 8920.34704715925f;
static float lqr_kroll  = 15811.38830084192f;
static float lqr_kpitch = 15811.38830084192f;
static float lqr_kyaw   = 15811.38830084192f;

// Command gain
static float command_kroll	=  15811.38830084192f;
static float command_kpitch	=  15811.38830084192f;
static float command_kyaw	=  15811.38830084192f;

// START LOGGING
static float lqr[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // LQR Controller Outputs
static float fdfwd[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // Feedforward Controller Outputs
static float adp[4] = {0.0f, 0.0f, 0.0f, 0.0f};
altitude_commands_t altitude_commands;
lqr_states_t lqr_states;
// END LOGGING

static motorPower_t motorPower;
static motorPower_t motorPowerSet;

/**
 * Supporting and utility functions
 */
static void setMotors(const motorPower_t *motorPower)
{
	motorsSetRatio(MOTOR_M1, motorPower->m1);
	motorsSetRatio(MOTOR_M2, motorPower->m2);
	motorsSetRatio(MOTOR_M3, motorPower->m3);
	motorsSetRatio(MOTOR_M4, motorPower->m4);
}

static void updateLqrStates(lqr_states_t *lqr_x, const sensorData_t *sensors, const state_t *state, const uint32_t tick)
{
	// sensors and states are in deg
	lqr_x->attitudeRate.roll 	=  sensors->gyro.x * DEG_TO_RAD;
	lqr_x->attitudeRate.pitch	= -sensors->gyro.y * DEG_TO_RAD;
	lqr_x->attitudeRate.yaw 	= -sensors->gyro.z * DEG_TO_RAD; // model (z down), crazyflie (z up)
	lqr_x->attitude.roll 		=  state->attitude.roll * DEG_TO_RAD;
	lqr_x->attitude.pitch		=  state->attitude.pitch * DEG_TO_RAD;
	lqr_x->attitude.yaw 		= -state->attitude.yaw * DEG_TO_RAD; // model (z down), crazyflie (z up)

	lqr_x->attitudeRate.timestamp = tick;
	lqr_x->attitude.timestamp = tick;
}
static void updateLqrCommands(altitude_commands_t *alt_cmds, const setpoint_t *setpoint, const uint32_t tick)
{
	// Commands in Deg
	alt_cmds->attitude.timestamp = tick;
	alt_cmds->attitude.roll		=  setpoint->attitude.roll * DEG_TO_RAD;
	alt_cmds->attitude.pitch	= -setpoint->attitude.pitch * DEG_TO_RAD; // inverted
	alt_cmds->attitude.yaw		=  setpoint->attitudeRate.yaw * DEG_TO_RAD;
	alt_cmds->thrust			=  setpoint->thrust;
}

/**
 * stateController API
 */

void stateControllerInit(void)
{
	motorsInit(motorMapDefaultBrushed);
	adaptiveControllerInit();
}

bool stateControllerTest(void)
{
	bool pass = true;

	pass &= motorsTest();
	pass &= adaptiveControllerTest();

	return pass;
}

void stateController(control_t *control, setpoint_t *setpoint,
					 const sensorData_t *sensors,
					 const state_t *state,
					 const uint32_t tick)
{
	if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
		updateLqrStates(&lqr_states, sensors, state, tick);
		updateLqrCommands(&altitude_commands, setpoint, tick);

		if (altitude_commands.thrust < 10)
		{
			adaptiveControllerReset();
			motorPower.m1 = limitThrust(0);
			motorPower.m2 = limitThrust(0);
			motorPower.m3 = limitThrust(0);
			motorPower.m4 = limitThrust(0);
		}
		else
		{
			float p = lqr_states.attitudeRate.roll;
			float q = lqr_states.attitudeRate.pitch;
			float r = lqr_states.attitudeRate.yaw;
			float roll = lqr_states.attitude.roll;
			float pitch = lqr_states.attitude.pitch;
			float yaw = lqr_states.attitude.yaw;

			lqr[0] = -1.0f*(-lqr_kp*p + lqr_kq*q + lqr_kr*r - lqr_kroll*roll + lqr_kpitch*pitch + lqr_kyaw*yaw);
			lqr[1] = -1.0f*(-lqr_kp*p - lqr_kq*q - lqr_kr*r - lqr_kroll*roll - lqr_kpitch*pitch - lqr_kyaw*yaw);
			lqr[2] = -1.0f*( lqr_kp*p - lqr_kq*q + lqr_kr*r + lqr_kroll*roll - lqr_kpitch*pitch + lqr_kyaw*yaw);
			lqr[3] = -1.0f*( lqr_kp*p + lqr_kq*q - lqr_kr*r + lqr_kroll*roll + lqr_kpitch*pitch - lqr_kyaw*yaw);

			// ROLL, PITCH, YAW, THROTTLE (Feed forward)
			fdfwd[0] = altitude_commands.attitude.roll  * command_kroll;
			fdfwd[1] = altitude_commands.attitude.pitch * command_kpitch;
			fdfwd[2] = altitude_commands.attitude.yaw   * command_kyaw;
			fdfwd[3] = altitude_commands.thrust;

			adaptiveControllerUpdateModel(&lqr_states, &altitude_commands, tick);
			adaptiveControllerUpdateGains(&lqr_states, &altitude_commands, tick);
			adaptiveControllerComputeOutputs(adp);

			motorPower.m1 = limitThrust( (int32_t) ( lqr[0] - fdfwd[0] + fdfwd[1] + fdfwd[2] + fdfwd[3] + adp[0] ));
			motorPower.m2 = limitThrust( (int32_t) ( lqr[1] - fdfwd[0] - fdfwd[1] - fdfwd[2] + fdfwd[3] + adp[1] ));
			motorPower.m3 = limitThrust( (int32_t) ( lqr[2] + fdfwd[0] - fdfwd[1] + fdfwd[2] + fdfwd[3] + adp[2] ));
			motorPower.m4 = limitThrust( (int32_t) ( lqr[3] + fdfwd[0] + fdfwd[1] - fdfwd[2] + fdfwd[3] + adp[3] ));
		}

		// skip power_distribution module
		if(motorSetEnable)
			setMotors(&motorPowerSet);
		else
			setMotors(&motorPower);
	}
}

PARAM_GROUP_START(lqrGain)
PARAM_ADD(PARAM_FLOAT, kp, &lqr_kp)
PARAM_ADD(PARAM_FLOAT, kq, &lqr_kq)
PARAM_ADD(PARAM_FLOAT, kr, &lqr_kr)
PARAM_ADD(PARAM_FLOAT, kroll, &lqr_kroll)
PARAM_ADD(PARAM_FLOAT, kpitch, &lqr_kpitch)
PARAM_ADD(PARAM_FLOAT, kyaw, &lqr_kyaw)
PARAM_GROUP_STOP(ring)

PARAM_GROUP_START(lqrCommandGain)
PARAM_ADD(PARAM_FLOAT, roll, &command_kroll)
PARAM_ADD(PARAM_FLOAT, pitch, &command_kpitch)
PARAM_ADD(PARAM_FLOAT, yaw, &command_kyaw)
PARAM_GROUP_STOP(ring)

PARAM_GROUP_START(motorPowerSet)
PARAM_ADD(PARAM_UINT8, enable, &motorSetEnable)
PARAM_ADD(PARAM_UINT16, m1, &motorPowerSet.m1)
PARAM_ADD(PARAM_UINT16, m2, &motorPowerSet.m2)
PARAM_ADD(PARAM_UINT16, m3, &motorPowerSet.m3)
PARAM_ADD(PARAM_UINT16, m4, &motorPowerSet.m4)
PARAM_GROUP_STOP(ring)

LOG_GROUP_START(motor)
LOG_ADD(LOG_INT32, m1, &motorPower.m1)
LOG_ADD(LOG_INT32, m2, &motorPower.m2)
LOG_ADD(LOG_INT32, m3, &motorPower.m3)
LOG_ADD(LOG_INT32, m4, &motorPower.m4)
LOG_GROUP_STOP(motor)

LOG_GROUP_START(lqrOutputs)
LOG_ADD(LOG_FLOAT, m1, lqr+0)
LOG_ADD(LOG_FLOAT, m2, lqr+1)
LOG_ADD(LOG_FLOAT, m3, lqr+2)
LOG_ADD(LOG_FLOAT, m4, lqr+3)
LOG_GROUP_STOP(lqrOutputs)

LOG_GROUP_START(fdfwdOutputs)
LOG_ADD(LOG_FLOAT, m1, fdfwd+0)
LOG_ADD(LOG_FLOAT, m2, fdfwd+1)
LOG_ADD(LOG_FLOAT, m3, fdfwd+2)
LOG_ADD(LOG_FLOAT, m4, fdfwd+3)
LOG_GROUP_STOP(lqrOutputs)

LOG_GROUP_START(adpOutputs)
LOG_ADD(LOG_FLOAT, m1, adp+0)
LOG_ADD(LOG_FLOAT, m2, adp+1)
LOG_ADD(LOG_FLOAT, m3, adp+2)
LOG_ADD(LOG_FLOAT, m4, adp+3)
LOG_GROUP_STOP(adpOutputs)

LOG_GROUP_START(lqrCommands)
LOG_ADD(LOG_FLOAT, roll, &altitude_commands.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &altitude_commands.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &altitude_commands.attitude.yaw)
LOG_ADD(LOG_FLOAT, thrust, &altitude_commands.thrust)
LOG_GROUP_STOP(lqrCommands)

LOG_GROUP_START(lqrStates)
LOG_ADD(LOG_FLOAT, p, &lqr_states.attitudeRate.roll)
LOG_ADD(LOG_FLOAT, q, &lqr_states.attitudeRate.pitch)
LOG_ADD(LOG_FLOAT, r, &lqr_states.attitudeRate.yaw)
LOG_ADD(LOG_FLOAT, roll, &lqr_states.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &lqr_states.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &lqr_states.attitude.yaw)
LOG_GROUP_STOP(lqrStates)
