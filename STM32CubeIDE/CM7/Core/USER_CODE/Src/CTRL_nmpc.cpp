/* NMPC.cpp
 *
 *  Created on: 26.02.2026
 *      Author: Jörg Fischer
 */

#include "CTRL_NMPC.hpp"

namespace CTRL
{
	NMPC::Result NMPC::create()
	{
		// allocate capsule (acados internal allocation)
		capsule_ = FurutaPendulum_acados_create_capsule();
		int status = FurutaPendulum_acados_create_with_discretization(capsule_, HOR_LEN, NULL);

		if (status != ACADOS_SUCCESS) return CREATION_ERROR;

		nlp_config_ = FurutaPendulum_acados_get_nlp_config(capsule_);
		nlp_dims_   = FurutaPendulum_acados_get_nlp_dims(capsule_);
		nlp_in_     = FurutaPendulum_acados_get_nlp_in(capsule_);
		nlp_out_    = FurutaPendulum_acados_get_nlp_out(capsule_);
		nlp_solver_ = FurutaPendulum_acados_get_nlp_solver(capsule_);
		nlp_opts_   = FurutaPendulum_acados_get_nlp_opts(capsule_);

		return SUCCESS;
	}


	NMPC::Result NMPC::initialize(plantState *state)
	{
		if (!capsule_) return CREATION_ERROR;
		FurutaPendulum_acados_reset(capsule_, 1);

		// initial state is not an optimization variable:
		double lbx0[NBX0];
		double ubx0[NBX0];
		for (int i = 0; i < NBX0; ++i)
		{
			lbx0[i] = 0.0;
			ubx0[i] = 0.0;
		}
		ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "lbx", lbx0);
		ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "ubx", ubx0);

		// set initial guess -----------------------------------------
		// construct initial guess for state values
		double x_init[HOR_LEN + 1][NX];
		for (int k = 0; k <= HOR_LEN; ++k)
		{
			x_init[k][0] = state->theta1;
			x_init[k][1] = state->theta1p;
			x_init[k][2] = state->theta2;
			x_init[k][3] = state->theta2p;
		}
		// construct initial guess for control inputs
		double u0[HOR_LEN][NU];
		for (int k = 0; k < HOR_LEN; ++k)
		{
			for (int j = 0; j < NU; ++j)
			{
				u0[k][j] = 0.0;
			}
		}
		// apply initial guesses
		for (int i = 0; i < HOR_LEN; i++)
		{
			ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "x", x_init[i]);
			ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "u", u0[i]);
		}
		ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, HOR_LEN, "x", x_init[HOR_LEN]);

		return SUCCESS;
	}

	NMPC::Result NMPC::preparationStep(void)
	{
		if (!capsule_) return CREATION_ERROR;
		// preparation phase
		int rti_phase = 1;
		ocp_nlp_solver_opts_set(nlp_config_, nlp_opts_, "rti_phase", &rti_phase);
		statusSolver_ = FurutaPendulum_acados_solve(capsule_);
		if ((statusSolver_ != 5) && (statusSolver_ != 0))
		{
			return INFEASIBLE;
		}
		return SUCCESS;
	}

	NMPC::Result NMPC::feedbackStep(plantState* state, f32_t *u_optimal)
	{
		if(!u_optimal) return POINTER_ERROR;
		double x_current[NX];
		x_current[0] = state->theta1;
		x_current[1] = state->theta1p;
		x_current[2] = state->theta2;
		x_current[3] = state->theta2p;

		// feedback phase
		int rti_phase = 2;
		ocp_nlp_solver_opts_set(nlp_config_, nlp_opts_, "rti_phase", &rti_phase);
		ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "lbx", x_current);
		ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "ubx", x_current);

		statusSolver_ = FurutaPendulum_acados_solve(capsule_);
		if (statusSolver_ != 0)
		{
			*u_optimal = 0.0f;
			return INFEASIBLE;
		}
		ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 0, "u", &u_optimal_[0]);
		// only extracted for debugging; not needed for model predictive control
		ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 1, "u", &u_optimal_[1]);
		ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 2, "u", &u_optimal_[2]);
		ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 3, "u", &u_optimal_[3]);
		ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 4, "u", &u_optimal_[4]);
		ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 5, "u", &u_optimal_[5]);
		ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 6, "u", &u_optimal_[6]);
		ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 7, "u", &u_optimal_[7]);
		ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 8, "u", &u_optimal_[8]);
		ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 9, "u", &u_optimal_[9]);

		*u_optimal = (f32_t)u_optimal_[0];

		return SUCCESS;
	}
}
