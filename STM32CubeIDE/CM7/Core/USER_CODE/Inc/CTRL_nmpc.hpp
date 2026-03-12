/* NMPC.hpp
 *
 *  Created on: 26.02.2026
 *      Author: Jörg Fischer
 */

#pragma once

#include "MyDataTypes.hpp"

#define SUCCESS HPIPM_SUCCESS_TEMP
	#include "acados_solver_FurutaPendulum.h"
#undef SUCCESS

namespace CTRL
{
	class NMPC
	{
	public:
		static constexpr int NX      = FURUTAPENDULUM_NX;
		static constexpr int NU      = FURUTAPENDULUM_NU;
		static constexpr int NBX0    = FURUTAPENDULUM_NBX0;
		static constexpr int HOR_LEN = FURUTAPENDULUM_N;

		NMPC() = default;
		NMPC(const NMPC&) = delete;
		NMPC& operator=(const NMPC&) = delete;

		enum Result
		   {
			   SUCCESS = 0,
			   INFEASIBLE,
			   CREATION_ERROR,
			   POINTER_ERROR
		   };


		// 1 on success, 0 on fail
		Result create(void);

		// 1 on success, 0 on fail
		Result initialize(plantState* state);

		Result preparationStep(void);
		Result feedbackStep(plantState* state, f32_t *u_optimal);

	private:
		// acados handles
		FurutaPendulum_solver_capsule *capsule_ = nullptr;
		ocp_nlp_config *nlp_config_ = nullptr;
		ocp_nlp_dims   *nlp_dims_   = nullptr;
		ocp_nlp_in     *nlp_in_     = nullptr;
		ocp_nlp_out    *nlp_out_    = nullptr;
		ocp_nlp_solver *nlp_solver_ = nullptr;
		void           *nlp_opts_   = nullptr;

		// runtime buffers
		double u_optimal_[HOR_LEN] = {0.0};

		volatile int  statusSolver_ = 0;
	};
}
