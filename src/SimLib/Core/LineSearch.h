#pragma once

#include <Eigen/Core>

namespace LineSearch {

//!Make sure you don't change state inside eval! (state is captured by reference and will be used throughout linesearch)
template<typename State, typename EvalState>
State backtrack(const State &state, const Eigen::VectorXd &dx, double f0, int maxIter, double reductionFactor, double &alpha, double &f, EvalState eval)
{
	if (alpha > 1.0 || alpha <= 0.0)
		alpha = 1.0;
	int iter;
	for (iter = 0; iter < maxIter; iter++)
	{
		State ns = state + alpha * dx;
		f = eval(ns);
		if (f < f0)
		{
			return ns;
		}
		alpha *= reductionFactor;
	}
	//alpha = 0.0;
	f = eval(state);
	return state;


	/*alpha = 1.0;
	int iter;
	for (iter = 0; iter < maxIter; iter++)
	{
		State ns = state + alpha * dx;
		f = eval(ns);
		if (f < f0)
		{
			return ns;
		}
		alpha *= reductionFactor;
	}
	alpha = 0.0;
	f = eval(state);
	return state;*/

}

}
