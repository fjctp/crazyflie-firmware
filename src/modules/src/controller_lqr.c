
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

// Gains
static float k_all = 1.0f;
static float k1 = 56.0698f;
static float k2 =  5.0f;
static float k3 =  7.0711f;
static float k4 =  0.0f;

// Gyro low pass filter
static float alpha = 1/PI/7.0f*ATTITUDE_RATE;

// START DEBUG
static int32_t lqr[4] = {0, 0, 0, 0}; // LQR Controller Outputs
static float roll_deg  = 0;  // Euler angles from States estimator
static float pitch_deg = 0;
static float yaw_deg   = 0;
// END DEBUG

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

lowPassFilter_t gyroLowPassFilter;


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
		updateLowPassFilter(&gyroLowPassFilter, sensors->gyro, alpha);

		float p = gyroLowPassFilter.filtered.x * DEG_TO_RAD; // gyro is in deg/sec
		float q = gyroLowPassFilter.filtered.y * DEG_TO_RAD;
		float r = gyroLowPassFilter.filtered.z * DEG_TO_RAD;

		roll_deg = state->attitude.roll;   // euler angles is in deg
		pitch_deg = state->attitude.pitch; // euler angles is in deg
		yaw_deg = state->attitude.yaw;     // euler angles is in deg

		float roll_rad = roll_deg * DEG_TO_RAD;
		float pitch_rad = pitch_deg * DEG_TO_RAD;
		float yaw_rad = yaw_deg * DEG_TO_RAD;

		/*command.pitch = -setpoint->velocity.x;
		command.roll = setpoint->velocity.y;
		command.yaw = setpoint->velocity.z;*/

		/*command.roll = setpoint->attitude.roll / (30.0f*100.0f);
		command.pitch = setpoint->attitude.pitch / (30.0f*100.0f);
		command.yaw = setpoint->attitude.yaw / (30.0f*100.0f);*/

		command.thrust = setpoint->thrust;
		command.roll = 0;
		command.pitch = 0;
		command.yaw = 0;

		if (command.thrust < 500)
		{
			motorPower.m1 = limitThrust(0);
			motorPower.m2 = limitThrust(0);
			motorPower.m3 = limitThrust(0);
			motorPower.m4 = limitThrust(0);
		}
		else
		{
			lqr[0] = (int32_t) -1.0f*k_all*( k1*q - k2*r + k3*pitch_rad - k4*yaw_rad);
			lqr[1] = (int32_t) -1.0f*k_all*(-k1*p + k2*r - k3*roll_rad  + k4*yaw_rad);
			lqr[2] = (int32_t) -1.0f*k_all*(-k1*q - k2*r - k3*pitch_rad - k4*yaw_rad);
			lqr[3] = (int32_t) -1.0f*k_all*( k1*p + k2*r + k3*roll_rad  + k4*yaw_rad);

			motorPower.m1 = limitThrust(lqr[0] + setpoint->thrust
			                                   - command.pitch
			                                   + command.yaw);
			motorPower.m2 = limitThrust(lqr[1] + setpoint->thrust
			                                   - command.roll
			                                   - command.yaw);
			motorPower.m3 = limitThrust(lqr[2] + setpoint->thrust
			                                   + command.pitch
			                                   + command.yaw);
			motorPower.m4 = limitThrust(lqr[3] + setpoint->thrust
			                                   + command.roll
			                                   - command.yaw);
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
PARAM_ADD(PARAM_FLOAT, kAll, &k_all)
PARAM_ADD(PARAM_FLOAT, k1, &k1)
PARAM_ADD(PARAM_FLOAT, k2, &k2)
PARAM_ADD(PARAM_FLOAT, k3, &k3)
PARAM_ADD(PARAM_FLOAT, k4, &k4)
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
