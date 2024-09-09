#include "UnconstrainedLBFGS.h"

#include "LineSearch.h"

#include <Eigen/IterativeLinearSolvers>
#include <iostream>
#include <fstream>

namespace SimOpt
{

UnconstrainedLBFGS::UnconstrainedLBFGS()
	:m_gradientThreshold(1e-5),
	m_maxIter(1000),
	m_lbfgs(20),
	m_newXAcceptedCallback([](const Eigen::VectorXd &x) {})
{
	m_state = INITIALIZED;
}
void UnconstrainedLBFGS::solve()
{
	m_objectiveGradientFunctor(m_x, m_gradient);
	int iter;
	for (iter = 0; iter < m_maxIter; iter++)
	{
		double oldGradientNorm = m_gradient.norm();
		if (oldGradientNorm < m_gradientThreshold)
		{
			//std::cout << "converged after " << iter << " iterations " << std::endl;
			m_state = SOLVED;
			break;
		}
		//m_objectiveHessianFunctor(m_x, m_hessian);

		//Eigen::VectorXd diag = m_hessian.diagonal().cwiseAbs();


		m_dx = -m_gradient;

		//Eigen::IncompleteCholesky<double> precond;
		//precond.compute(m_hessian);

		//m_lbfgs.apply(m_dx, [&precond](Eigen::VectorXd &dx) {
		//	dx = precond.solve(dx);
		//});
		m_lbfgs.apply(m_dx);

		double largest_alpha = m_largestLineSearchStepFunctor(m_x, m_dx);//this will compute the largest step size
		m_dx *= largest_alpha;
		//std::cout << " reg " << reg << std::endl;

		double oldFx = m_objectiveFunctor(m_x);
		double alpha = 1.0;

		double f;
		LineSearch::backtrack(m_x, m_dx, oldFx, 100, 0.75, alpha, f, m_objectiveFunctor);
		
		m_x += alpha * m_dx;

		m_oldGradient = m_gradient;
		m_objectiveGradientFunctor(m_x, m_gradient);
		m_y = m_gradient - m_oldGradient;
		m_lbfgs.update(alpha * m_dx, m_y);

		m_newXAcceptedCallback(m_x);
	}
	if (iter == m_maxIter)
	{
		m_objectiveGradientFunctor(m_x, m_gradient);
		std::cout << " WARNING: m_maxIter reached, gradient norm = " << m_gradient.norm() << std::endl;
		m_state = SOLVE_FAILED;
	}
}

}