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
	m_lineSearchMethod(lineSearchMethod::ls_backtrack),
	m_maxIter(1000),
	m_printLevel(1),
	m_newXAcceptedCallback([](const Eigen::VectorXd &x) {}),
	m_linearSolver(std::move(linearSolver))
{
	m_state = INITIALIZED;
	m_disableWarnOutput = false;
	m_checkForSPDAtSolution = false;// true;
	m_startRegularization = 1e-6;
}

double UnconstrainedNewton::computeSearchDirection(const Eigen::VectorXd& x, const Eigen::VectorXd& gradient, Eigen::VectorXd& dx, double start_reg)
{
	m_linearSolver->setX(x);

	double reg = 0.0;
	while (true)
	{
		m_linearSolver->setShift(reg);
		if (reg > 1e20)
		{
			if(m_printLevel>0) spdlog::warn("WARNING: huge regularization in forward simulation necessary, hessian very indefinite, check hessian AND GRADIENT implementation, dx norm {}", dx.norm());
			//m_linearSolver->debugHessian();
			m_state = SOLVE_FAILED;
			return reg;
		}
		m_linearSolver->factorize();
		if (m_linearSolver->info() != Eigen::Success)
		{
			//std::cout << " inc reg because of cholesky " << std::endl;
			reg = 4 * reg + start_reg; // 1e-6;
			continue;
		}
		
		m_linearSolver->solve(gradient, dx);
		dx = -dx;
		if (dx.dot(gradient) >= 0.0)
		{
			//std::cout << " inc reg because of descending condition  " << std::endl;
			reg = 4 * reg + 0.1 * start_reg; // 1e-7;
			continue;
		}

		break;
	}
	return reg;
}

void UnconstrainedNewton::solve()
{
	m_state = INITIALIZED;
	Eigen::VectorXd gradient(m_x.size());
	for (m_iter = 0; m_iter < m_maxIter;)
	{
		bool initialized = m_stepBeginningFunctor(m_x, m_iter);
		if (!initialized)
		{
			m_state = SOLVE_FAILED;
			break;
		}
		m_objectiveGradientFunctor(m_x, gradient);
		double oldGradientNorm = gradient.norm();
		if(m_printLevel>0) spdlog::debug("Gradient norm {} in iteration {}", oldGradientNorm, m_iter);
		if (oldGradientNorm != oldGradientNorm || isnan(oldGradientNorm))
		{
			if(m_printLevel>0) spdlog::error(" Error gradient norm {} in iteration {}", oldGradientNorm, m_iter);
			m_state = SOLVE_FAILED;
			break;
		}
		if (!m_checkForSPDAtSolution && m_iter>=1)
		{
			if (oldGradientNorm < m_gradientThreshold)
			{
				//std::cout << "converged after " << iter << " iterations " << std::endl;
				m_state = SOLVED;
				break;
			}
		}
		
		m_iter++;

		Eigen::VectorXd dx;
		double reg = computeSearchDirection(m_x, gradient, dx, m_startRegularization);
		if (m_checkForSPDAtSolution && reg == 0.0 && oldGradientNorm < m_gradientThreshold && m_iter > 1)
		{
			//std::cout << "converged after " << iter << " iterations " << std::endl;
			m_state = SOLVED;
			break;
		}
		if(m_state == SOLVE_FAILED)
		{
			break;
		}
		if (m_checkForSPDAtSolution && reg != 0.0 && oldGradientNorm < m_gradientThreshold)
		{
			if(m_printLevel>0) spdlog::warn("Not stopping because of regularization = {} but gradient {} < {}", reg, oldGradientNorm, m_gradientThreshold);
		}

		if(m_printLevel>0) spdlog::debug("Added regularization {}", reg);
		//std::cout << " reg " << reg << std::endl;

		
		m_searchDirectionFilterFunctor(dx);//filter search direction
		m_updateOperatorBeforeLineSearchFunctor(m_x, dx);

		double oldFx = m_objectiveFunctor(m_x);
		
		double largest_alpha = m_largestLineSearchStepFunctor(m_x, dx);//this will compute the largest step size
		dx *= largest_alpha;
		if(m_printLevel>0) spdlog::debug("The largest search alpha {} multiplied into dx", largest_alpha);
		double alpha = 1.0;

		bool tryFullStep = false;
		if (tryFullStep)
		{

			auto computeGradientNorm = [this](const Eigen::VectorXd &x)
			{
				Eigen::VectorXd grad(x.size());
				m_objectiveGradientFunctor(x, grad);
				return grad.norm();
			};

			if(m_printLevel>0) spdlog::info(" trying full step ");

			double triedGradientNorm = computeGradientNorm(m_x + dx);
			if(m_printLevel>0) spdlog::info(" triedGradientNorm  {}", triedGradientNorm);
		}

		double f;
		switch (m_lineSearchMethod)
		{
		case lineSearchMethod::ls_backtrack:
			{
				LineSearch::backtrack(m_x, dx, oldFx, 25, 0.5, alpha, f, [this](const Eigen::VectorXd& x) { return m_objectiveFunctor(x); });
				break;
			}
		case lineSearchMethod::ls_backtrackGreedy:
			{
				LineSearch::backtrackGreedy(m_x, dx, oldFx, 50, 0.5, alpha, f, [this](const Eigen::VectorXd& x) { return m_objectiveFunctor(x); });
				break;
			}
		case lineSearchMethod::ls_fullStep:
			{
				alpha = 1.0;
				break;
			}
		default:
			{
				LineSearch::backtrack(m_x, dx, oldFx, 25, 0.5, alpha, f, [this](const Eigen::VectorXd& x) { return m_objectiveFunctor(x); });
				break;
			}
		}
		

		if(m_printLevel>0) spdlog::debug("alpha {} step length {} with objective value {} at iteration {}", alpha, dx.norm() * alpha, f, m_iter);
		m_afterLineSearchFunctor(m_x, dx, alpha, m_iter);

		if (alpha == 0.0)
		{
			if(m_printLevel>0) spdlog::warn("ERROR: alpha == 0 in line search in forward simulation, stopping");
			m_state = SOLVE_FAILED;
			break;
			//testing whether we are close to solution, often the change in the objective underflows in this case because dx is too small
			// but only allowed when no regularization was necessary, so we are pretty sure its a descending direction 
			if (reg == 0.0 && dx.norm() < m_dxThreshold)
			{
				auto computeGradientNorm = [this](const Eigen::VectorXd &x) {
					Eigen::VectorXd grad(x.size());
					m_objectiveGradientFunctor(x, grad);
					return grad.norm();
				};

				//std::cout << " trying full step " << std::endl;

				double triedGradientNorm = computeGradientNorm(m_x + dx);
				//std::cout << " triedGradientNorm  " << triedGradientNorm << std::endl;
				if (triedGradientNorm < m_gradientThreshold)
				{
					m_x += dx;
					m_newXAcceptedCallback(m_x);
					//TODO add security check on change of objective?
					//if (dx.norm() < m_dxThreshold && reg == 0.0)
					//{
					//	std::cout << "Warning accepted solution because no regularization was used and dx norm < threshold = " << m_dxThreshold << std::endl;
						//std::cout << "Trying to improve gradient norm using line search on gradient norm: " << std::endl;
						//
						//double oldGradNorm = gradient.norm();
						//double gradNormNew;
						//LineSearch::backtrackGreedy(m_x, dx, oldGradNorm, 100, 0.8, alpha, gradNormNew, computeGradientNorm);
						//m_x += alpha * dx;
						//std::cout << "alpha: " << alpha << std::endl;
						//std::cout << "f new: " << m_objectiveFunctor(m_x).operator double() << std::endl;
						//std::cout << "gradient norm : " << oldGradNorm << " -> " << gradNormNew << std::endl;
					if(m_printLevel>0) spdlog::warn("alpha is 0, but triedGradientNorm is {}", triedGradientNorm);
					m_state = SOLVE_FAILED;
					return;
				}
				else
				{
					//std::cout << " triedGradientNorm  " << triedGradientNorm << std::endl;
					if (alpha == 0.0)
					{
						if(m_printLevel>0) spdlog::warn("ERROR: alpha == 0 in line search in forward simulation, stopping");
						if(m_printLevel>0) spdlog::warn("gradient norm = {}", gradient.norm());
						if(m_printLevel>0) spdlog::warn("m_x norm = {}", m_x.norm());
						if(m_printLevel>0) spdlog::warn("dx norm = {}", dx.norm());
						if(m_printLevel>0) spdlog::warn(" regularization was : {}", reg);
						if(m_printLevel>0) spdlog::warn(" f: {}", f);
						m_state = SOLVE_FAILED;

						break;
					}
				}
			}
			else
			{
				//std::cout << " detected failed line search with regularization " << std::endl;
				if(m_printLevel>0) spdlog::warn(" detected failed line search with regularization = {}", reg);
				if(m_printLevel>0) spdlog::warn(" trying smallest eigenvector ");
				//should we move this into regularization tests or is this last resort? TODO add eigenvalue 0 check on solve?
				{
					//Eigen::VectorXd v = -gradient;
					Eigen::VectorXd v = Eigen::VectorXd::Random(gradient.size());
					v.normalize();
					Eigen::VectorXd vn;
					for (int i = 0; i < 10; i++) //TODO replace by spectra
					{
						m_linearSolver->solve(v, vn);
						v = vn.normalized();
					}

					if (v.dot(gradient) >= 0.0)
					{
						dx = -v * 1e-3;
					}
					else
					{
						dx = v * 1e-3;
					}

					m_searchDirectionFilterFunctor(dx);//with this new search direction, we filter it
					double f;
					LineSearch::backtrackGreedy(m_x, dx, oldFx, 100, 0.7, alpha, f, [this](const Eigen::VectorXd &x) { return m_objectiveFunctor(x); });
					if(m_printLevel>0) std::cout << " alpha using smallest eigenvector " << alpha << std::endl;
					if (alpha == 1.0)
					{
						if(m_printLevel>0) std::cout << " trying to increase alpha " << std::endl;
						double alphaNext = alpha;
						double nfx = f;
						double alphaLast = alpha;
						while (nfx < oldFx)
						{
							alphaLast = alphaNext;
							alphaNext *= 2.0;
							nfx = m_objectiveFunctor(m_x + alphaNext * dx);
						}
						alpha = alphaLast;
						if(m_printLevel>0) std::cout << " found alpha " << alpha << std::endl;
					}
				}

				if (alpha == 0.0)
				{
					if(m_printLevel>0) spdlog::warn("ERROR: alpha == 0 in line search in forward simulation, stopping");
					//LineSearch::backtrackGreedy(m_x, dx, oldFx, 100, 0.8, alpha, f, [this, reference](const Eigen::VectorXd &x) { return (m_objectiveFunctor(x) - reference).operator double(); });
					if(m_printLevel>0) spdlog::warn("gradient norm = {}", gradient.norm());
					if(m_printLevel>0) spdlog::warn("m_x norm = {}", m_x.norm());
					if(m_printLevel>0) spdlog::warn("dx norm = {}", dx.norm());
					if(m_printLevel>0) spdlog::warn(" regularization was : {}", reg);
					if(m_printLevel>0) spdlog::warn(" f: {}", f);
					//std::cout << " accepted residual was : " << (m_hessian.selfadjointView<Eigen::Upper>() * dx + gradient + reg * gradient).norm() << std::endl;
					m_state = SOLVE_FAILED;

					break;
				}
			}

		}

		m_x += alpha * dx;
		m_newXAcceptedCallback(m_x); // note that this functor is allowed to change x, (i.e. to adapt simulation to configuration, i.e. incremental rotation parametrization)
	}
	if (m_iter == m_maxIter)
	{
		m_objectiveGradientFunctor(m_x, gradient);
		if(!m_disableWarnOutput)
			if(m_printLevel>0) spdlog::warn(" WARNING: m_maxIter reached, gradient norm = {}", gradient.norm());
		m_state = SOLVE_FAILED;
	}
}
void UnconstrainedNewton::polish()
{
	//assert(m_state == SOLVED);
	Eigen::VectorXd gradient(m_x.size());
	m_objectiveGradientFunctor(m_x, gradient);
	double oldGradientNorm = gradient.norm();
	if(m_printLevel>0) spdlog::debug(" start polishing {}", oldGradientNorm);
	while (true)
	{
		/*m_linearSolver->setX(m_x);
		m_linearSolver->setShift(0.0);
		m_linearSolver->factorize();
		if (m_linearSolver->info() != Eigen::Success)
		{
			if(m_printLevel>0) spdlog::debug(" error could not factorize in polish() ");
			int k = 3;
			return;
		}
		Eigen::VectorXd dx;
		m_linearSolver->solve(gradient, dx);
		dx = -dx;*/
		Eigen::VectorXd dx;
		double reg = computeSearchDirection(m_x, gradient, dx, 1e-9);
		Eigen::VectorXd xn = m_x + dx;
		m_objectiveGradientFunctor(xn, gradient);
		double newGradientNorm = gradient.norm();

		if (newGradientNorm < oldGradientNorm) // we have a scale of 0.5 here for now, because of numerical problems we sometimes do polish steps which are miniscule (changing gradient norm from 2.1e-11 to 2.09e-11 for example)
		{
			if(m_printLevel>0) spdlog::debug(" accepted polishing {}", newGradientNorm );
			oldGradientNorm = newGradientNorm;
			m_x = xn;
			m_newXAcceptedCallback(m_x);
		}
		else
		{
			if(m_printLevel>0) spdlog::debug(" stopped polishing {}", oldGradientNorm);
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
