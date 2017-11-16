#include "adaptive_controller.h"
#include "stabilizer_types.h"

#include "FreeRTOS.h"
#include <math.h>
#include "arm_math.h"
#include "num.h"

#include "log.h"
#include "param.h"

#define ADAPTIVE_MODEL_RATE					ATTITUDE_RATE
#define ADAPTIVE_LAW_RATE					ATTITUDE_RATE
#define ADAPTIVE_MODEL_UPDATE_DT			(float)(1.0f/ADAPTIVE_MODEL_RATE)
#define ADAPTIVE_LAW_UPDATE_DT				(float)(1.0f/ADAPTIVE_LAW_RATE)

#define ADAPTIVE_LAW_INTEGRATOR_THRESHOLD	1e-4f
#define ADAPTIVE_GAIN_MAX					20000.0f
#define ADAPTIVE_MODEL_STATES_MAX			 1000.0f
#define ADAPTIVE_OUTPUT_MAX					40000.0f

#define ADAPTIVE_STATES_DIM		6
#define ADAPTIVE_INPUTS_DIM		4
#define ADAPTIVE_COMMANDS_DIM	4

#define ADAPTIVE_PHI_FUNC_DIM	10
#define NRBF_PER_STATE			3

#define DEG_TO_RAD (PI/180.0f)
#define RAD_TO_DEG (180.0f/PI)

#define M_PI_F 	(float)M_PI

/**
 * General Variables
 */
static bool adaptiveSetEnable = false;

/**
 * Reference Model
 */
static float Aref[ADAPTIVE_STATES_DIM * ADAPTIVE_STATES_DIM] = {
   -62.8139246236921f,                -0.0f,                 0.0f,  -188.8866291793110f,    -0.0000000000001f,                 0.0f,
				-0.0f,   -58.8918704979545f,                 0.0f,                 0.0f,  -176.5007348711485f,     0.0000000000001f,
				 0.0f,                 0.0f,    -5.1690076360555f,                 0.0f,                 0.0f,    -9.1621084282497f,
				 1.0f,                 0.0f,                 0.0f,                 0.0f,                 0.0f,                 0.0f,
				 0.0f,                 1.0f,                 0.0f,                 0.0f,                 0.0f,                 0.0f,
				 0.0f,                 0.0f,                 1.0f,                 0.0f,                 0.0f,                 0.0f

};
static float Bref[ADAPTIVE_STATES_DIM * ADAPTIVE_INPUTS_DIM] = {
   188.8866291793108f,                 0.0f,               0.0f,                0.0f,
				 0.0f,   176.5007348711482f,               0.0f,                0.0f,
				 0.0f,                 0.0f,   9.1621084282498f,                0.0f,
				 0.0f,                 0.0f,               0.0f,                0.0f,
				 0.0f,                 0.0f,               0.0f,                0.0f,
				 0.0f,                 0.0f,               0.0f,                0.0f
};
static float B[ADAPTIVE_STATES_DIM * ADAPTIVE_INPUTS_DIM] = {
	-0.002986559838791f,  -0.002986559838791f,   0.002986559838791f,   0.002986559838791f,
	 0.002790721654432f,  -0.002790721654432f,  -0.002790721654432f,   0.002790721654432f,
	 0.000144865654013f,  -0.000144865654013f,   0.000144865654013f,  -0.000144865654013f,
	               0.0f,                 0.0f,                 0.0f,                 0.0f,
	               0.0f,                 0.0f,                 0.0f,                 0.0f,
	               0.0f,                 0.0f,                 0.0f,                 0.0f
};
static float x_ref[ADAPTIVE_STATES_DIM];
static float x_cmd[ADAPTIVE_COMMANDS_DIM];
static float x_err[ADAPTIVE_STATES_DIM];

/**
 * Adaptive Control
 */
static float inv_pv[ADAPTIVE_STATES_DIM * ADAPTIVE_STATES_DIM] = {
   2.781703018791624f,  -0.000000000000000f,   0.000000000000000f,   6.419341126073790f,   0.000000000000003f,   0.000000000000000f,
  -0.000000000000000f,   2.683366970116205f,  -0.000000000000002f,  -0.000000000000004f,   6.088362059947713f,  -0.000000000000004f,
   0.000000000000000f,  -0.000000000000002f,   1.505534727907421f,   0.000000000000000f,  -0.000000000000004f,   0.770586454127876f,
   6.419341126073790f,  -0.000000000000004f,   0.000000000000000f,  17.495304799786688f,   0.000000000000000f,   0.000000000000000f,
   0.000000000000003f,   6.088362059947713f,  -0.000000000000004f,   0.000000000000000f,  16.456538811035010f,  -0.000000000000011f,
   0.000000000000000f,  -0.000000000000004f,   0.770586454127876f,   0.000000000000000f,  -0.000000000000011f,   1.510145163747154f
};
static float big_gamma[ADAPTIVE_PHI_FUNC_DIM * ADAPTIVE_PHI_FUNC_DIM] = {
   20000.0f,           0.0f,           0.0f,           0.0f,           0.0f,           0.0f,           0.0f,           0.0f,           0.0f,           0.0f,
	   0.0f,       20000.0f,           0.0f,           0.0f,           0.0f,           0.0f,           0.0f,           0.0f,           0.0f,           0.0f,
	   0.0f,           0.0f,       20000.0f,           0.0f,           0.0f,           0.0f,           0.0f,           0.0f,           0.0f,           0.0f,
	   0.0f,           0.0f,           0.0f,       20000.0f,           0.0f,           0.0f,           0.0f,           0.0f,           0.0f,           0.0f,
	   0.0f,           0.0f,           0.0f,           0.0f,       20000.0f,           0.0f,           0.0f,           0.0f,           0.0f,           0.0f,
	   0.0f,           0.0f,           0.0f,           0.0f,           0.0f,       20000.0f,           0.0f,           0.0f,           0.0f,           0.0f,
	   0.0f,           0.0f,           0.0f,           0.0f,           0.0f,           0.0f,       20000.0f,           0.0f,           0.0f,           0.0f,
	   0.0f,           0.0f,           0.0f,           0.0f,           0.0f,           0.0f,           0.0f,       20000.0f,           0.0f,           0.0f,
	   0.0f,           0.0f,           0.0f,           0.0f,           0.0f,           0.0f,           0.0f,           0.0f,       20000.0f,           0.0f,
	   0.0f,           0.0f,           0.0f,           0.0f,           0.0f,           0.0f,           0.0f,           0.0f,           0.0f,       20000.0f
};
static float Lv[ADAPTIVE_STATES_DIM * ADAPTIVE_STATES_DIM] = {
  14.073551390048442f,  -0.000000000000004f,  -0.000000000000000f,  -5.163838427619302f,  -0.000000000000001f,   0.000000000000000f,
  -0.000000000000004f,  13.925016890306281f,   0.000000000000001f,   0.000000000000005f,  -5.151784679183076f,   0.000000000000003f,
  -0.000000000000000f,   0.000000000000001f,   5.394106020879559f,   0.000000000000000f,  -0.000000000000001f,  -2.752467200905034f,
  -5.163838427619302f,   0.000000000000005f,   0.000000000000000f,   2.237654092616559f,  -0.000000000000001f,  -0.000000000000000f,
  -0.000000000000001f,  -5.151784679183076f,  -0.000000000000001f,  -0.000000000000001f,   2.270582581842923f,   0.000000000000002f,
   0.000000000000000f,   0.000000000000003f,  -2.752467200905034f,  -0.000000000000000f,   0.000000000000002f,   5.377637948591545f
};
static float big_theta[ADAPTIVE_PHI_FUNC_DIM * ADAPTIVE_INPUTS_DIM];
static float big_theta_dot[ADAPTIVE_PHI_FUNC_DIM * ADAPTIVE_INPUTS_DIM];
static float phi_function[ADAPTIVE_PHI_FUNC_DIM];

/**
 * Prepare for matrix operations using arm_math.h
 */
static arm_matrix_instance_f32 Aref_mat = {ADAPTIVE_STATES_DIM, ADAPTIVE_STATES_DIM, Aref};
static arm_matrix_instance_f32 Bref_mat = {ADAPTIVE_STATES_DIM, ADAPTIVE_INPUTS_DIM, Bref};
static arm_matrix_instance_f32 B_mat = {ADAPTIVE_STATES_DIM, ADAPTIVE_INPUTS_DIM, B};
static arm_matrix_instance_f32 Xref_mat = {ADAPTIVE_STATES_DIM, 1, x_ref};
static arm_matrix_instance_f32 Xcmd_mat = {ADAPTIVE_COMMANDS_DIM, 1, x_cmd};
static arm_matrix_instance_f32 Xerr_mat = {ADAPTIVE_STATES_DIM, 1, x_err};

static arm_matrix_instance_f32 inv_Pv_mat = {ADAPTIVE_STATES_DIM, ADAPTIVE_STATES_DIM, inv_pv};
static arm_matrix_instance_f32 big_gamma_mat = {ADAPTIVE_PHI_FUNC_DIM, ADAPTIVE_PHI_FUNC_DIM, big_gamma};
static arm_matrix_instance_f32 Lv_mat = {ADAPTIVE_STATES_DIM, ADAPTIVE_STATES_DIM, Lv};
static arm_matrix_instance_f32 big_theta_mat = {ADAPTIVE_PHI_FUNC_DIM, ADAPTIVE_INPUTS_DIM, big_theta};
static arm_matrix_instance_f32 big_theta_dot_mat = {ADAPTIVE_PHI_FUNC_DIM, ADAPTIVE_INPUTS_DIM, big_theta_dot};
static arm_matrix_instance_f32 phi_function_mat = {ADAPTIVE_PHI_FUNC_DIM, 1, phi_function};

/**
 * RBF
 */
static float rbf_center_p[NRBF_PER_STATE] = {-40.0f/180.0f*M_PI_F, 0.0f/180.0f*M_PI_F, 40.0f/180.0f*M_PI_F};
static float rbf_width_p = 100.0f/180.0f*M_PI_F;
static float rbf_center_q[NRBF_PER_STATE] = {-40.0f/180.0f*M_PI_F, 0.0f/180.0f*M_PI_F, 40.0f/180.0f*M_PI_F};
static float rbf_width_q = 100.0f/180.0f*M_PI_F;
static float rbf_center_r[NRBF_PER_STATE] = {-40.0f/180.0f*M_PI_F, 0.0f/180.0f*M_PI_F, 40.0f/180.0f*M_PI_F};
static float rbf_width_r = 100.0f/180.0f*M_PI_F;

/**
 * Worker functions
 */
static inline void mat_add(const arm_matrix_instance_f32 * pSrcA, const arm_matrix_instance_f32 * pSrcB, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_add_f32(pSrcA, pSrcB, pDst)); }
static inline void mat_sub(const arm_matrix_instance_f32 * pSrcA, const arm_matrix_instance_f32 * pSrcB, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_sub_f32(pSrcA, pSrcB, pDst)); }
static inline void mat_trans(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_trans_f32(pSrc, pDst)); }
static inline void mat_mult(const arm_matrix_instance_f32 * pSrcA, const arm_matrix_instance_f32 * pSrcB, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_mult_f32(pSrcA, pSrcB, pDst)); }

float rbf(float _x, float _center, float _width)
{
	// Gaussian radial basis function
	return expf(-powf(((_x-_center)/(_width/4.0f)), 2.0f));
}

void computeXDot(arm_matrix_instance_f32 *_xdot,
				 const arm_matrix_instance_f32 *_A, const arm_matrix_instance_f32 *_B, const arm_matrix_instance_f32 *_Lv,
				 const arm_matrix_instance_f32 *_x, const arm_matrix_instance_f32 *_u, const arm_matrix_instance_f32 *_x_err)
{
	float tmp1[ADAPTIVE_STATES_DIM];
	float tmp2[ADAPTIVE_STATES_DIM];
	arm_matrix_instance_f32 tmp1_mat = {ADAPTIVE_STATES_DIM, 1, tmp1};
	arm_matrix_instance_f32 tmp2_mat = {ADAPTIVE_STATES_DIM, 1, tmp2};

	/*
	// x_dot = A * x + B * x_cmd
	mat_mult(_A, _x, &tmp1_mat);
	mat_mult(_B, _u, &tmp2_mat);
	mat_add(&tmp1_mat, &tmp2_mat, _xdot);
	*/
	
	// x_dot = A * x + Lv * ( x - x_ref ) + B * x_cmd
	float tmp3[ADAPTIVE_STATES_DIM];
	float tmp4[ADAPTIVE_STATES_DIM];
	arm_matrix_instance_f32 tmp3_mat = {ADAPTIVE_STATES_DIM, 1, tmp3};
	arm_matrix_instance_f32 tmp4_mat = {ADAPTIVE_STATES_DIM, 1, tmp4};

	mat_mult(_A, _x, &tmp1_mat);
	mat_mult(_B, _u, &tmp2_mat);
	mat_mult(_Lv, _x_err, &tmp3_mat);
	mat_add(&tmp1_mat, &tmp2_mat, &tmp4_mat);
	mat_add(&tmp4_mat, &tmp3_mat, _xdot);
	
}
void computeBigThetaDot(arm_matrix_instance_f32 *_big_theta_dot,
						const arm_matrix_instance_f32 *_gamma, const arm_matrix_instance_f32 *_x_error_transpose,
						const arm_matrix_instance_f32 *_pv_inv, const arm_matrix_instance_f32 *_B,
						const arm_matrix_instance_f32 *_phi_functions)
{
	// Big_Theta_dot = gamma * Phi * X_err_transpose * Pv_inv * B

	float tmp1[ADAPTIVE_PHI_FUNC_DIM];
	float tmp2[ADAPTIVE_PHI_FUNC_DIM * ADAPTIVE_STATES_DIM];
	float tmp3[ADAPTIVE_PHI_FUNC_DIM * ADAPTIVE_STATES_DIM];

	arm_matrix_instance_f32 tmp1_mat = {ADAPTIVE_PHI_FUNC_DIM, 1, tmp1};
	arm_matrix_instance_f32 tmp2_mat = {ADAPTIVE_PHI_FUNC_DIM, ADAPTIVE_STATES_DIM, tmp2};
	arm_matrix_instance_f32 tmp3_mat = {ADAPTIVE_PHI_FUNC_DIM, ADAPTIVE_STATES_DIM, tmp3};

	mat_mult(_gamma, _phi_functions, &tmp1_mat);
	mat_mult(&tmp1_mat, _x_error_transpose, &tmp2_mat);
	mat_mult(&tmp2_mat, _pv_inv, &tmp3_mat);
	mat_mult(&tmp3_mat, _B, _big_theta_dot);
}
void computePhiFunctions(float *_phi, const lqr_states_t *_lqr_states )
{
	/*
	_phi[0] = _lqr_states->attitudeRate.roll;
	_phi[1] = _lqr_states->attitudeRate.pitch;
	_phi[2] = _lqr_states->attitudeRate.yaw;
	_phi[3] = 1;
	*/

	/*
	_phi[1] = rbf(_lqr_states->attitudeRate.roll, rbf_center_p[1], rbf_width_p);
	_phi[4] = rbf(_lqr_states->attitudeRate.pitch, rbf_center_q[1], rbf_width_q);
	_phi[7] = rbf(_lqr_states->attitudeRate.yaw, rbf_center_r[1], rbf_width_r);
	_phi[3] = 1;
	*/

	_phi[0] = rbf(_lqr_states->attitudeRate.roll, rbf_center_p[0], rbf_width_p);
	_phi[1] = rbf(_lqr_states->attitudeRate.roll, rbf_center_p[1], rbf_width_p);
	_phi[2] = rbf(_lqr_states->attitudeRate.roll, rbf_center_p[2], rbf_width_p);
	_phi[3] = rbf(_lqr_states->attitudeRate.pitch, rbf_center_q[0], rbf_width_q);
	_phi[4] = rbf(_lqr_states->attitudeRate.pitch, rbf_center_q[1], rbf_width_q);
	_phi[5] = rbf(_lqr_states->attitudeRate.pitch, rbf_center_q[2], rbf_width_q);
	_phi[6] = rbf(_lqr_states->attitudeRate.yaw, rbf_center_r[0], rbf_width_r);
	_phi[7] = rbf(_lqr_states->attitudeRate.yaw, rbf_center_r[1], rbf_width_r);
	_phi[8] = rbf(_lqr_states->attitudeRate.yaw, rbf_center_r[2], rbf_width_r);
	_phi[9] = 1;

}

/**
 * Main functions
 */
void adaptiveControllerInit(void)
{
	adaptiveControllerReset();
}
bool adaptiveControllerTest(void)
{
	return true;
}

void adaptiveControllerReset(void)
{
	for(int i=0; i<ADAPTIVE_STATES_DIM; i++) {
		x_ref[i] = 0.0f;
		x_err[i] = 0.0f;
	}
	for(int i=0; i<ADAPTIVE_COMMANDS_DIM; i++) {
		x_cmd[i] = 0.0f;
	}

	for(int i=0; i<(ADAPTIVE_PHI_FUNC_DIM * ADAPTIVE_INPUTS_DIM); i++) {
		big_theta[i] = 0.0f;
	}
}
void adaptiveControllerUpdateModel(	const lqr_states_t *_lqr_states, const altitude_commands_t *_command, const uint32_t _tick)
{
	if (adaptiveSetEnable) {
		x_cmd[0] = _command->attitude.roll;
		x_cmd[1] = _command->attitude.pitch;
		x_cmd[2] = _command->attitude.yaw;
		x_cmd[3] = _command->thrust;

		// update state error vector for computeXDot()
		float x[ADAPTIVE_STATES_DIM];
		arm_matrix_instance_f32 X_mat = {ADAPTIVE_STATES_DIM, 1, x};
		x[0] = _lqr_states->attitudeRate.roll;
		x[1] = _lqr_states->attitudeRate.pitch;
		x[2] = _lqr_states->attitudeRate.yaw;
		x[3] = _lqr_states->attitude.roll;
		x[4] = _lqr_states->attitude.pitch;
		x[5] = _lqr_states->attitude.yaw;
		mat_sub(&X_mat, &Xref_mat, &Xerr_mat);

		// update model
		float xref_dot[ADAPTIVE_STATES_DIM];
		arm_matrix_instance_f32 Xref_dot_mat = {ADAPTIVE_STATES_DIM, 1, xref_dot};
		computeXDot(&Xref_dot_mat, &Aref_mat, &Bref_mat, &Lv_mat, &Xref_mat, &Xcmd_mat, &Xerr_mat);
		for(int i=0; i<ADAPTIVE_STATES_DIM; i++) {
			if (fabsf(xref_dot[i])>ADAPTIVE_LAW_INTEGRATOR_THRESHOLD) {
				// if xdot is less than the threshold, no change to x
				// euler integration, may switch to predictor-corrector method
				x_ref[i] = x_ref[i] + ADAPTIVE_MODEL_UPDATE_DT*xref_dot[i];
				x_ref[i] = max(min(x_ref[i], ADAPTIVE_MODEL_STATES_MAX), -ADAPTIVE_MODEL_STATES_MAX);
			}
		}

		// update state error vector for adaptiveControllerUpdateGains()
		//mat_sub(&X_mat, &Xref_mat, &Xerr_mat);
	}
}

void adaptiveControllerUpdateGains(	const lqr_states_t *_lqr_states, const altitude_commands_t *_command, const uint32_t _tick)
{
	if (adaptiveSetEnable) {
		// Get transpose
		//float x_err_transpose[ADAPTIVE_STATES_DIM];
		arm_matrix_instance_f32 Xerr_transpose_mat = {1, ADAPTIVE_STATES_DIM, x_err};
		//mat_trans(&Xerr_mat, &Xerr_transpose_mat);

		// Get phi function
		computePhiFunctions(phi_function, _lqr_states);

		// compute big theta
		computeBigThetaDot( &big_theta_dot_mat,
							&big_gamma_mat,
							&Xerr_transpose_mat,
							&inv_Pv_mat,
							&B_mat,
							&phi_function_mat);

		for(int i=0; i<(ADAPTIVE_PHI_FUNC_DIM*ADAPTIVE_INPUTS_DIM); i++) {
			if (fabsf(big_theta_dot[i])>ADAPTIVE_LAW_INTEGRATOR_THRESHOLD) {
				// if xdot is less than the threshold, no change to x
				// euler integration, may switch to predictor-corrector method
				big_theta[i] = big_theta[i] + ADAPTIVE_LAW_UPDATE_DT*big_theta_dot[i];
				big_theta[i] = max(min(big_theta[i], ADAPTIVE_GAIN_MAX), -ADAPTIVE_GAIN_MAX);
			}
		}
	}
}
void adaptiveControllerComputeOutputs(float* _outputs)
{
	// output = - (big_theta_transpose * phi_functions)
	arm_matrix_instance_f32 outputs_mat = {ADAPTIVE_INPUTS_DIM, 1, _outputs};

	float big_theta_transpose[ADAPTIVE_INPUTS_DIM * ADAPTIVE_PHI_FUNC_DIM];
	arm_matrix_instance_f32 big_theta_transpose_mat = {ADAPTIVE_INPUTS_DIM, ADAPTIVE_PHI_FUNC_DIM, big_theta_transpose};

	mat_trans(&big_theta_mat, &big_theta_transpose_mat);
	mat_mult(&big_theta_transpose_mat, &phi_function_mat, &outputs_mat);

	for (int i = 0; i<ADAPTIVE_INPUTS_DIM; i++) {
		_outputs[i] = -_outputs[i];
		_outputs[i] = max(min(_outputs[i], ADAPTIVE_OUTPUT_MAX), -ADAPTIVE_OUTPUT_MAX);
	}
}

/**
 * Params and Logs
 */
PARAM_GROUP_START(lqrAdaptiveControl)
PARAM_ADD(PARAM_UINT8, enable, &adaptiveSetEnable)
PARAM_GROUP_STOP(ring)

LOG_GROUP_START(adpReferenceStates)
LOG_ADD(LOG_FLOAT, p, x_ref)
LOG_ADD(LOG_FLOAT, q, x_ref+1)
LOG_ADD(LOG_FLOAT, r, x_ref+2)
LOG_ADD(LOG_FLOAT, roll, x_ref+3)
LOG_ADD(LOG_FLOAT, pitch, x_ref+4)
LOG_ADD(LOG_FLOAT, yaw, x_ref+5)
LOG_GROUP_STOP(adpReferenceStates)

LOG_GROUP_START(adpReferenceError)
LOG_ADD(LOG_FLOAT, p, x_err+0)
LOG_ADD(LOG_FLOAT, q, x_err+1)
LOG_ADD(LOG_FLOAT, r, x_err+2)
LOG_ADD(LOG_FLOAT, roll, x_err+3)
LOG_ADD(LOG_FLOAT, pitch, x_err+4)
LOG_ADD(LOG_FLOAT, yaw, x_err+5)
LOG_GROUP_STOP(adpReferenceError)

/**
 *	Adaptive control
 */
LOG_GROUP_START(adpPhiFunction)
LOG_ADD(LOG_FLOAT, p, phi_function+0)
LOG_ADD(LOG_FLOAT, q, phi_function+1)
LOG_ADD(LOG_FLOAT, r, phi_function+2)
LOG_ADD(LOG_FLOAT, constant, phi_function+3)
LOG_GROUP_STOP(adpPhiFunction)

LOG_GROUP_START(adpBigThetaR1)
LOG_ADD(LOG_FLOAT, 00, big_theta+0+0)
LOG_ADD(LOG_FLOAT, 01, big_theta+0+1)
LOG_ADD(LOG_FLOAT, 02, big_theta+0+2)
LOG_ADD(LOG_FLOAT, 03, big_theta+0+3)
LOG_GROUP_STOP(adpBigThetaR1)

LOG_GROUP_START(adpBigThetaR2)
LOG_ADD(LOG_FLOAT, 10, big_theta+1+0)
LOG_ADD(LOG_FLOAT, 11, big_theta+1+1)
LOG_ADD(LOG_FLOAT, 12, big_theta+1+2)
LOG_ADD(LOG_FLOAT, 13, big_theta+1+3)
LOG_GROUP_STOP(adpBigThetaR2)

LOG_GROUP_START(adpBigThetaR3)
LOG_ADD(LOG_FLOAT, 20, big_theta+2+0)
LOG_ADD(LOG_FLOAT, 21, big_theta+2+1)
LOG_ADD(LOG_FLOAT, 22, big_theta+2+2)
LOG_ADD(LOG_FLOAT, 23, big_theta+2+3)
LOG_GROUP_STOP(adpBigThetaR3)

LOG_GROUP_START(adpBigThetaR4)
LOG_ADD(LOG_FLOAT, 30, big_theta+3+0)
LOG_ADD(LOG_FLOAT, 31, big_theta+3+1)
LOG_ADD(LOG_FLOAT, 32, big_theta+3+2)
LOG_ADD(LOG_FLOAT, 33, big_theta+3+3)
LOG_GROUP_STOP(adpBigThetaR4)

/*
LOG_GROUP_START(adpPhiFunction)
LOG_ADD(LOG_FLOAT, p0, phi_function+0)
LOG_ADD(LOG_FLOAT, p1, phi_function+1)
LOG_ADD(LOG_FLOAT, p2, phi_function+2)
LOG_ADD(LOG_FLOAT, q0, phi_function+3)
LOG_ADD(LOG_FLOAT, q1, phi_function+4)
LOG_ADD(LOG_FLOAT, q2, phi_function+5)
LOG_ADD(LOG_FLOAT, r0, phi_function+6)
LOG_ADD(LOG_FLOAT, r1, phi_function+7)
LOG_ADD(LOG_FLOAT, r2, phi_function+8)
LOG_ADD(LOG_FLOAT, constant, phi_function+9)
LOG_GROUP_STOP(adpPhiFunction)

LOG_GROUP_START(adpBigThetaR1)
LOG_ADD(LOG_FLOAT, 00, big_theta+0+0)
LOG_ADD(LOG_FLOAT, 01, big_theta+0+1)
LOG_ADD(LOG_FLOAT, 02, big_theta+0+2)
LOG_ADD(LOG_FLOAT, 03, big_theta+0+3)
LOG_ADD(LOG_FLOAT, 04, big_theta+0+4)
LOG_ADD(LOG_FLOAT, 05, big_theta+0+5)
LOG_ADD(LOG_FLOAT, 06, big_theta+0+6)
LOG_ADD(LOG_FLOAT, 07, big_theta+0+7)
LOG_ADD(LOG_FLOAT, 08, big_theta+0+8)
LOG_ADD(LOG_FLOAT, 09, big_theta+0+9)
LOG_GROUP_STOP(adpBigThetaR1)

LOG_GROUP_START(adpBigThetaC1)
LOG_ADD(LOG_FLOAT, 00, big_theta+0+0)
LOG_ADD(LOG_FLOAT, 10, big_theta+1+0)
LOG_ADD(LOG_FLOAT, 20, big_theta+2+0)
LOG_ADD(LOG_FLOAT, 30, big_theta+3+0)
LOG_ADD(LOG_FLOAT, 40, big_theta+4+0)
LOG_ADD(LOG_FLOAT, 50, big_theta+5+0)
LOG_ADD(LOG_FLOAT, 60, big_theta+6+0)
LOG_ADD(LOG_FLOAT, 70, big_theta+7+0)
LOG_ADD(LOG_FLOAT, 80, big_theta+8+0)
LOG_ADD(LOG_FLOAT, 90, big_theta+9+0)
LOG_GROUP_STOP(adpBigThetaC1)
*/
