#ifndef ADAPTIVE_CONTROLLER_H_
#define ADAPTIVE_CONTROLLER_H_

#include "stabilizer_types.h"
#include "arm_math.h"

void adaptiveControllerInit(void);
bool adaptiveControllerTest(void);

void adaptiveControllerReset(void);
void adaptiveControllerUpdateModel(	const lqr_states_t *lqr_states, const altitude_commands_t *command, const uint32_t tick);
void adaptiveControllerUpdateGains(	const lqr_states_t *lqr_states, const altitude_commands_t *command, const uint32_t tick);
void adaptiveControllerComputeOutputs( float* outputs);

#endif /* ADAPTIVE_CONTROLLER_H_ */
