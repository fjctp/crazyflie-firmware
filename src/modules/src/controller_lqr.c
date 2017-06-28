
#include "stabilizer.h"
#include "stabilizer_types.h"

#include "sensfusion6.h"

#include "motors.h"

#include "log.h"
#include "param.h"
#include "num.h"

#include "math.h"
#include "arm_math.h"

#define DEG_TO_RAD (PI/180.0f)
#define RAD_TO_DEG (180.0f/PI)
#define limitThrust(VAL) limitUint16(VAL)

typedef struct lowPassFilter_s {
	Axis3f sensors_last;
	Axis3f filtered;
} lowPassFilter_t;

static bool motorSetEnable = false;

// LQR gains
static float lqr_kp     = 1446.116972692153f;
static float lqr_kq     = 1479.600141759485f;
static float lqr_kr     = 5429.071423864209f;
static float lqr_kroll  = 8338.333333333342f;
static float lqr_kpitch = 8338.333333333342f;
static float lqr_kyaw   = 8338.333333333344f;

// Command gain
static float command_kroll	=  8338.333333333251f;
static float command_kpitch	=  8338.333333333251f;
static float command_kyaw	=  8338.333333333303f;

// Gyro low pass filter
static float alpha = 1/PI/7.0f*ATTITUDE_RATE;

// START LOGGING
static int32_t lqr[4] = {0, 0, 0, 0}; // LQR Controller Outputs
static float roll_deg  = 0;  // Euler angles from States estimator
static float pitch_deg = 0;
static float yaw_deg   = 0;
lowPassFilter_t gyroLowPassFilter;
// END LOGGING


static struct {
  float thrust;
  float roll;
  float pitch;
  float yaw;
} command;

static struct {
  uint32_t m1;
  uint32_t m2;
  uint32_t m3;
  uint32_t m4;
} motorPower;

static struct {
  uint16_t m1;
  uint16_t m2;
  uint16_t m3;
  uint16_t m4;
} motorPowerSet;


static void setMotors();
static void updateLowPassFilter(lowPassFilter_t *lowPassFilter, const Axis3f sensor, const float alpha);

void stateControllerInit(void)
{
	  motorsInit(motorMapDefaultBrushed);
}

bool stateControllerTest(void)
{
  bool pass = true;

  pass &= motorsTest();

  return pass;
}

void stateController(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
	if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
		Axis3f rate_gyro = {
				.x =  sensors->gyro.x,
				.y = -sensors->gyro.y,  // pitch rate is inverted
				.z = -sensors->gyro.z}; // model (z down), crazyflie (z up)

		updateLowPassFilter(&gyroLowPassFilter, rate_gyro, alpha);

		float p = gyroLowPassFilter.filtered.x * DEG_TO_RAD; // gyro is in deg/sec
		float q = gyroLowPassFilter.filtered.y * DEG_TO_RAD;
		float r = gyroLowPassFilter.filtered.z * DEG_TO_RAD;

		roll_deg  =  state->attitude.roll;    // euler angles is in deg
		pitch_deg =  state->attitude.pitch;
		yaw_deg   = -state->attitude.yaw;     // model (z down), crazyflie (z up)

		float roll = roll_deg * DEG_TO_RAD;
		float pitch = pitch_deg * DEG_TO_RAD;
		float yaw = yaw_deg * DEG_TO_RAD;

		command.roll   =  setpoint->attitude.roll;
		command.pitch  = -setpoint->attitude.pitch; // inverted
		command.yaw    =  setpoint->attitudeRate.yaw;
		command.thrust =  setpoint->thrust;

		// ROLL, PITCH, YAW, THROTTLE
		float cmds[4] = {command.roll * DEG_TO_RAD * command_kroll,
						 command.pitch * DEG_TO_RAD * command_kpitch,
						 command.yaw * DEG_TO_RAD * command_kyaw,
						 command.thrust};

		
		if (command.thrust < 10)
		{
			motorPower.m1 = limitThrust(0);
			motorPower.m2 = limitThrust(0);
			motorPower.m3 = limitThrust(0);
			motorPower.m4 = limitThrust(0);
		}
		else
		{
			
#ifndef QUAD_FORMATION_X
			lqr[0] = (int32_t) -1.0f*( lqr_kq*q + lqr_kr*r + lqr_kpitch*pitch + lqr_kyaw*yaw);
			lqr[1] = (int32_t) -1.0f*(-lqr_kp*p - lqr_kr*r - lqr_kroll*roll   - lqr_kyaw*yaw);
			lqr[2] = (int32_t) -1.0f*(-lqr_kq*q + lqr_kr*r - lqr_kpitch*pitch + lqr_kyaw*yaw);
			lqr[3] = (int32_t) -1.0f*( lqr_kp*p - lqr_kr*r + lqr_kroll*roll   - lqr_kyaw*yaw);

			motorPower.m1 = limitThrust(lqr[0] + cmds[3]
			                                   + cmds[1]
			                                   + cmds[2]);
			motorPower.m2 = limitThrust(lqr[1] + cmds[3]
			                                   - cmds[0]
			                                   - cmds[2]);
			motorPower.m3 = limitThrust(lqr[2] + cmds[3]
			                                   - cmds[1]
			                                   + cmds[2]);
			motorPower.m4 = limitThrust(lqr[3] + cmds[3]
			                                   + cmds[0]
			                                   - cmds[2]);
#else
			lqr[0] = (int32_t) -1.0f*(-lqr_kp*p + lqr_kq*q + lqr_kr*r - lqr_kroll*roll + lqr_kpitch*pitch + lqr_kyaw*yaw);
			lqr[1] = (int32_t) -1.0f*(-lqr_kp*p - lqr_kq*q - lqr_kr*r - lqr_kroll*roll - lqr_kpitch*pitch - lqr_kyaw*yaw);
			lqr[2] = (int32_t) -1.0f*( lqr_kp*p - lqr_kq*q + lqr_kr*r + lqr_kroll*roll - lqr_kpitch*pitch + lqr_kyaw*yaw);
			lqr[3] = (int32_t) -1.0f*( lqr_kp*p + lqr_kq*q - lqr_kr*r + lqr_kroll*roll + lqr_kpitch*pitch - lqr_kyaw*yaw);

			motorPower.m1 = limitThrust(lqr[0] - cmds[0] + cmds[1] + cmds[2] + cmds[3]);
			motorPower.m2 = limitThrust(lqr[1] - cmds[0] - cmds[1] - cmds[2] + cmds[3]);
			motorPower.m3 = limitThrust(lqr[2] + cmds[0] - cmds[1] + cmds[2] + cmds[3]);
			motorPower.m4 = limitThrust(lqr[3] + cmds[0] + cmds[1] - cmds[2] + cmds[3]);
#endif
		}

		setMotors(); // skip power_distribution module
	}
}

static void setMotors()
{
	if (motorSetEnable)
	{
		motorsSetRatio(MOTOR_M1, motorPowerSet.m1);
		motorsSetRatio(MOTOR_M2, motorPowerSet.m2);
		motorsSetRatio(MOTOR_M3, motorPowerSet.m3);
		motorsSetRatio(MOTOR_M4, motorPowerSet.m4);
	}
	else
	{
		motorsSetRatio(MOTOR_M1, motorPower.m1);
		motorsSetRatio(MOTOR_M2, motorPower.m2);
		motorsSetRatio(MOTOR_M3, motorPower.m3);
		motorsSetRatio(MOTOR_M4, motorPower.m4);
	}
}

static void updateLowPassFilter(lowPassFilter_t *lowPassFilter, const Axis3f sensor, const float alpha)
{
	lowPassFilter->filtered.x = ((alpha-1)*lowPassFilter->filtered.x +
								(sensor.x+lowPassFilter->filtered.x))/(alpha+1);
	lowPassFilter->filtered.y = ((alpha-1)*lowPassFilter->filtered.y +
								(sensor.y+lowPassFilter->filtered.y))/(alpha+1);
	lowPassFilter->filtered.z = ((alpha-1)*lowPassFilter->filtered.z +
								(sensor.z+lowPassFilter->filtered.z))/(alpha+1);

	lowPassFilter->sensors_last = sensor;
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
LOG_ADD(LOG_INT32, m4, &motorPower.m4)
LOG_ADD(LOG_INT32, m1, &motorPower.m1)
LOG_ADD(LOG_INT32, m2, &motorPower.m2)
LOG_ADD(LOG_INT32, m3, &motorPower.m3)
LOG_GROUP_STOP(motor)

LOG_GROUP_START(lqrOutputs)
LOG_ADD(LOG_INT32, m1, lqr+0)
LOG_ADD(LOG_INT32, m2, lqr+1)
LOG_ADD(LOG_INT32, m3, lqr+2)
LOG_ADD(LOG_INT32, m4, lqr+3)
LOG_GROUP_STOP(lqrOutputs)

LOG_GROUP_START(lqrCommands)
LOG_ADD(LOG_FLOAT, thrust, &command.thrust)
LOG_ADD(LOG_FLOAT, roll, &command.roll)
LOG_ADD(LOG_FLOAT, pitch, &command.pitch)
LOG_ADD(LOG_FLOAT, yaw, &command.yaw)
LOG_GROUP_STOP(lqrCommands)

LOG_GROUP_START(lqrStates)
LOG_ADD(LOG_FLOAT, gyro_roll_deg, &gyroLowPassFilter.filtered.x)
LOG_ADD(LOG_FLOAT, gyro_pitch_deg, &gyroLowPassFilter.filtered.y)
LOG_ADD(LOG_FLOAT, gyro_yaw_deg, &gyroLowPassFilter.filtered.z)
LOG_ADD(LOG_FLOAT, euler_roll_deg, &roll_deg)
LOG_ADD(LOG_FLOAT, euler_pitch_deg, &pitch_deg)
LOG_ADD(LOG_FLOAT, euler_yaw_deg, &yaw_deg)
LOG_GROUP_STOP(lqrStates)
