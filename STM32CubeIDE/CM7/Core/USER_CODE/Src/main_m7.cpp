/*
 * main_m7.cpp
 *
 *  Created on: March 10, 2026
 *      Author: Joerg Fischer
 */

#include "main_m7.h"
#include "MyDataTypes.hpp"
#include "CTRL_nmpc.hpp"

#ifdef __cplusplus
	// c includes go here
	extern "C" {
	}
#endif

CTRL::NMPC gNmpc;
plantState gState;
f32_t gOptimalU;


int main_m7(void)
{

	// allocate memory
	gNmpc.create();

	// fill with initial state guess (could be extended to initial control inputs)
	gState = {0.1F, 0.0F, 0.0F, 0.0F};
	gNmpc.initialize(&gState);

	while (1)
	{
		// preparation step
		gNmpc.preparationStep();

		// read sensors and estimate state: e.g.
		// gState = getStateEstimate();

		// feedback step to get u_optimal
		gNmpc.feedbackStep(&gState, &gOptimalU);

		// apply u_optimal: e.g.
		// setMotorVoltage(gOptimalU);
	}

	return 0;
}

