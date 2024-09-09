#include "UnconstrainedNewton.h"

#include "LineSearch.h"

#include <Eigen/Eigenvalues>

#include <iostream>
#include <fstream>

#include <Spectra/SymEigsSolver.h>
#include "FunctionalGenMatProd.h"

#include <spdlog/spdlog.h>

namespace SimOpt
{

Eigen::VectorXd computeLargestEigenvalueVectorSymmetric(std::function<Eigen::VectorXd(const Eigen::VectorXd&)> applyA,
	int nx)
{
	int maxIter = 100;
	double tolerance = 1e-5;

	Spectra::FunctionalGenMatProd<double> prod(applyA, nx, nx);
	//Spectra::SymEigsSolver<double, Spectra::LARGEST_ALGE, Spectra::FunctionalGenMatProd<double> > spectraSolver(&prod, 1, 5);
	Spectra::SymEigsSolver<Spectra::FunctionalGenMatProd<double>> spectraSolver(prod, 1, 5);
	spectraSolver.init();
	//int nconv = spectraSolver.compute(maxIter, tolerance);
	//std::cout << " spectra solver success : " << (spectraSolver.info() == Spectra::SUCCESSFUL) << " , info = " << spectraSolver.info() << std::endl;
	int nconv = spectraSolver.compute(Spectra::SortRule::LargestAlge, maxIter, tolerance);
	std::cout << " spectra solver success : " << (spectraSolver.info() == Spectra::CompInfo::Successful) << " , info = " << int(spectraSolver.info()) << std::endl;
	std::cout << " nconv : " << nconv << std::endl;
	Eigen::VectorXd eigenvalues = spectraSolver.eigenvalues();
	return spectraSolver.eigenvectors().col(0);
}
void debugHessian(const Eigen::SparseMatrix<double> &hessian)
{
	std::cout << "starting hessian debuging" << std::endl;

	std::cout << "checking for NANs (actually finite numbers)" << std::endl;
	bool thereIsNaN = false;
	for (int k = 0; k < hessian.outerSize(); k++)
	{
		for (Eigen::SparseMatrix<double>::InnerIterator it(hessian, k); it; ++it)
		{
			if (!std::isfinite(it.value()))
			{
				thereIsNaN = true;
			}
		}
	}

	std::cout << " result: " << std::endl;
	if (thereIsNaN)
	{
		std::cout << " NANs found! (or infinite values) " << std::endl;
		std::ofstream out("debug hessian.txt");
		out << hessian << std::endl;
		out.close();
		return;
	}

	std::cout << "no NANs found! " << std::endl;
	std::cout << "should i do an eigenvalue analysis?" << std::endl;
	char c;
	std::cin >> c;
	if (c == 'y' || c == 'Y')
	{
		std::cout << " Understood yes " << std::endl;

		std::cout << "Doing eigen analysis on upper triangular part" << std::endl;
		Eigen::VectorXd eigenvalues = hessian.toDense().selfadjointView<Eigen::Upper>().eigenvalues();

		std::ofstream out("debug eigenvalues.txt");
		out << eigenvalues << std::endl;
		out.close();
	}
	else
	{
		std::cout << " Understood no " << std::endl;
	}

	std::ofstream out("debug hessian.txt");
	out << hessian << std::endl;
	out.close();
}

UnconstrainedNewtonLinearSolverLLT::~UnconstrainedNewtonLinearSolverLLT()
{
}
void UnconstrainedNewtonLinearSolverLLT::debugHessian()
{
	::SimOpt::debugHessian(m_hessian);
}

UnconstrainedNewtonLinearSolverLLTWrapper::UnconstrainedNewtonLinearSolverLLTWrapper(
	LLT<Eigen::SparseMatrix<double>, Eigen::Upper > *solver,
	Eigen::SparseMatrix<double> *hessian,
	std::function<void(const Eigen::VectorXd& x, Eigen::SparseMatrix<double> &hessian)> objectiveHessianFunctor)
	:m_solver(solver),
	m_hessian(hessian),
	m_objectiveHessianFunctor(objectiveHessianFunctor)
{
}

void UnconstrainedNewtonLinearSolverLLTWrapper::setX(const Eigen::VectorXd &x)
{
	m_objectiveHessianFunctor(x, *m_hessian);
}
void UnconstrainedNewtonLinearSolverLLTWrapper::factorize()
{
	//m_solver->analyzePattern(*m_hessian);
	m_solver->factorize(*m_hessian);
}
void UnconstrainedNewtonLinearSolverLLTWrapper::setShift(double val)
{
	//std::cout << " solver setShift " << val << std::endl;
	m_solver->setShift(val);
}
void UnconstrainedNewtonLinearSolverLLTWrapper::solve(const Eigen::VectorXd &rhs, Eigen::VectorXd &solution)
{
	solution = m_solver->solve(rhs);
	//std::cout << " solver residual " << (m_hessian->selfadjointView<Eigen::Upper>() * solution -rhs).norm() << std::endl;
	//std::cout << " solver info " << m_solver->info() << std::endl;
}
Eigen::ComputationInfo UnconstrainedNewtonLinearSolverLLTWrapper::info() const
{
	return m_solver->info();
}
void UnconstrainedNewtonLinearSolverLLTWrapper::debugHessian()
{
	::SimOpt::debugHessian(*m_hessian);
}

UnconstrainedNewton::UnconstrainedNewton(std::unique_ptr<UnconstrainedNewtonLinearSolver> linearSolver)
	:m_gradientThreshold(1e-5),
	m_dxThreshold(1e-9),
	m_maxIter(1000),
	m_newXAcceptedCallback([](const Eigen::VectorXd &x) {}),
	m_linearSolver(std::move(linearSolver))
{
	m_state = INITIALIZED;
	m_disableWarnOutput = false;
}
void UnconstrainedNewton::solve()
{
	m_state = INITIALIZED;
	bool checkForSPDAtSolution = false;// true;
	Eigen::VectorXd gradient(m_x.size());
	for (m_iter = 0; m_iter < m_maxIter;)
	{
		m_stepBeginningFunctor(m_x, m_iter);
		m_objectiveGradientFunctor(m_x, gradient);
		double oldGradientNorm = gradient.norm();
		spdlog::debug("Gradient norm {} in iteration {}", oldGradientNorm, m_iter);
		if (oldGradientNorm != oldGradientNorm)
		{
			std::cout << " Error gradient norm " << oldGradientNorm << " in iteration " << m_iter << std::endl;
			m_state = SOLVE_FAILED;
			break;
		}
		if (!checkForSPDAtSolution && m_iter>=1)
		{
			if (oldGradientNorm < m_gradientThreshold)
			{
				//std::cout << "converged after " << iter << " iterations " << std::endl;
				m_state = SOLVED;
				break;
			}
		}
		

		m_iter++;

		m_linearSolver->setX(m_x);

		Eigen::VectorXd dx;

		double reg = 0.0;
		while (true)
		{
			m_linearSolver->setShift(reg);
			if (reg > 1e20)
			{
				spdlog::warn("WARNING: huge regularization in forward simulation necessary, hessian very indefinite, check hessian AND GRADIENT implementation");
				//m_linearSolver->debugHessian();
				m_state = SOLVE_FAILED;
				return;
			}
			m_linearSolver->factorize();
			if (m_linearSolver->info() != Eigen::Success)
			{
				//std::cout << " inc reg because of cholesky " << std::endl;
				reg = 4 * reg + 1e-6;
				continue;
			}
			if (checkForSPDAtSolution && reg == 0.0 && oldGradientNorm < m_gradientThreshold && m_iter > 1)
			{
				//std::cout << "converged after " << iter << " iterations " << std::endl;
				m_state = SOLVED;
				break;
			}

			m_linearSolver->solve(gradient, dx);
			dx = -dx;
			if (dx.dot(gradient) >= 0.0)
			{
				//std::cout << " inc reg because of descending condition  " << std::endl;
				reg = 4 * reg + 1e-7;
				continue;
			}

			break;
		}
		if (m_state == SOLVED) break;
		if (checkForSPDAtSolution && reg != 0.0 && oldGradientNorm < m_gradientThreshold)
		{
			spdlog::warn("Not stopping because of regularization = {} but gradient {} < {}", reg, oldGradientNorm, m_gradientThreshold);
		}

		spdlog::debug("Added regularization {}", reg);
		//std::cout << " reg " << reg << std::endl;

		//SimOpt::ASScalar oldFXAcc = m_objectiveFunctor(m_x);
		//double reference = oldFXAcc.operator double();
		//double oldFx = (m_objectiveFunctor(m_x) - reference).operator double();
		m_searchDirectionFilterFunctor(dx);//filter search direction
		m_updateOperatorBeforeLineSearchFunctor(m_x, dx);

		double oldFx = m_objectiveFunctor(m_x);
		
		double largest_alpha = m_largestLineSearchStepFunctor(m_x, dx);//this will compute the largest step size
		dx *= largest_alpha;
		spdlog::debug("The largest search alpha {} multiplied into dx", largest_alpha);
		double alpha = 1.0;


		double f;
		LineSearch::backtrack(m_x, dx, oldFx, 25, 0.5, alpha, f, [this](const Eigen::VectorXd& x) { return m_objectiveFunctor(x); });
		
		spdlog::debug("alpha {} step length {} with objective value {} at iteration {}", alpha, dx.norm() * alpha, f, m_iter);
		
		m_x += alpha * dx;
		m_newXAcceptedCallback(m_x); // note that this functor is allowed to change x, (i.e. to adapt simulation to configuration, i.e. incremental rotation parametrization)
	}
	if (m_iter == m_maxIter)
	{
		m_objectiveGradientFunctor(m_x, gradient);
		if(!m_disableWarnOutput)
			spdlog::warn(" WARNING: m_maxIter reached, gradient norm = {}", gradient.norm());
		m_state = SOLVE_FAILED;
	}
}
void UnconstrainedNewton::polish()
{
	assert(m_state == SOLVED);
	Eigen::VectorXd gradient(m_x.size());
	m_objectiveGradientFunctor(m_x, gradient);
	double oldGradientNorm = gradient.norm();
	std::cout << " start polishing " << oldGradientNorm << std::endl;
	while (true)
	{
		m_linearSolver->setX(m_x);
		m_linearSolver->setShift(0.0);
		m_linearSolver->factorize();
		if (m_linearSolver->info() != Eigen::Success)
		{
			std::cout << " error could not factorize in polish() " << std::endl;
			int k = 3;
		}
		Eigen::VectorXd dx;
		m_linearSolver->solve(gradient, dx);
		dx = -dx;
		Eigen::VectorXd xn = m_x + dx;
		m_objectiveGradientFunctor(xn, gradient);
		double newGradientNorm = gradient.norm();

		if (newGradientNorm < oldGradientNorm * 0.5) // we have a scale of 0.5 here for now, because of numerical problems we sometimes do polish steps which are miniscule (changing gradient norm from 2.1e-11 to 2.09e-11 for example)
		{
			std::cout << " accepted polishing " << newGradientNorm << std::endl;
			oldGradientNorm = newGradientNorm;
			m_x = xn;
			m_newXAcceptedCallback(m_x);
		}
		else
		{
			std::cout << " stopped polishing " << oldGradientNorm << std::endl;
			break;
		}
	}
}
const char* UnconstrainedNewton::toString(STATE state)
{
	switch (state)
	{
	case INITIALIZED: return "INITIALIZED";
	case SOLVE_FAILED: return "SOLVE_FAILED";
	case SOLVED: return "SOLVED";
	default: throw std::logic_error("not implemented");
	}
}

}