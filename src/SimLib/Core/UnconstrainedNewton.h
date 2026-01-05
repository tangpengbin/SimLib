#pragma once

#include "LLT.h"

#include <Eigen/Core>
#include <Eigen/Sparse>

#include <functional>
#include <memory>

namespace SimOpt
{

class UnconstrainedNewtonLinearSolver
{
public:
	virtual ~UnconstrainedNewtonLinearSolver(){}

	virtual Eigen::SparseMatrix<double> getHessian(const Eigen::VectorXd& x) = 0;
	virtual Eigen::SparseMatrix<double> getHessian() = 0;
	virtual Eigen::SparseMatrix<double> getShiftedHessian() = 0;
	virtual void setX(const Eigen::VectorXd &x) = 0;
	virtual void factorize() = 0;
	virtual void setShift(double val) = 0;
	virtual void solve(const Eigen::VectorXd &rhs, Eigen::VectorXd &solution) = 0;
	virtual Eigen::ComputationInfo info() const = 0;
	virtual void testHessian(const Eigen::VectorXd &x, std::function<void(const Eigen::VectorXd& x, Eigen::VectorXd &grad)> grad) { throw std::logic_error("not implemented"); }
	virtual void debugHessian() {}
};


template<int N>
class UnconstrainedNewtonLinearSolverDenseLLT : public UnconstrainedNewtonLinearSolver
{
public:
	typedef Eigen::Matrix<double, N, N> MatrixType;

	UnconstrainedNewtonLinearSolverDenseLLT(std::function<void(const Eigen::VectorXd& x, MatrixType &hessian)> objectiveHessianFunctor)
		:
		m_objectiveHessianFunctor(objectiveHessianFunctor)
	{
	}
	~UnconstrainedNewtonLinearSolverDenseLLT()
	{

	}
	Eigen::SparseMatrix<double> getHessian(const Eigen::VectorXd& x)
	{
		m_objectiveHessianFunctor(x, m_hessian);
		Eigen::MatrixXd hessianxd = m_hessian;
		return hessianxd.sparseView();
	}


	Eigen::SparseMatrix<double> getHessian()
	{
		return m_hessian.sparseView();
	}
	Eigen::SparseMatrix<double> getShiftedHessian()
	{
		return m_hessianShifted.sparseView();
	}

	MatrixType getDenseHessian(const Eigen::VectorXd& x)
	{
		m_objectiveHessianFunctor(x, *m_hessian);
		return m_hessian;
	}
	virtual void setX(const Eigen::VectorXd &x)
	{
		m_objectiveHessianFunctor(x, m_hessian);
		m_hessianShifted = m_hessian;
		m_shift = 0.0;
	}
	virtual void factorize()
	{
		m_solver.compute(m_hessianShifted);
	}
	virtual void setShift(double val)
	{
		m_shift = val;
		for (int i = 0; i < m_hessianShifted.rows(); i++)
		{
			m_hessianShifted(i, i) = m_hessian(i, i) + m_shift;
		}
	}
	virtual void solve(const Eigen::VectorXd &rhs, Eigen::VectorXd &solution)
	{
		solution = m_solver.solve(rhs);
	}
	virtual Eigen::ComputationInfo info() const
	{
		return m_solver.info();
	}

private:
	Eigen::LLT<MatrixType, Eigen::Upper > m_solver;

	MatrixType m_hessian;
	MatrixType m_hessianShifted;
	double m_shift;
	std::function<void(const Eigen::VectorXd& x, MatrixType &hessian)> m_objectiveHessianFunctor;
};


class UnconstrainedNewtonLinearSolverLLT : public UnconstrainedNewtonLinearSolver
{
public:
	UnconstrainedNewtonLinearSolverLLT(std::function<void(const Eigen::VectorXd& x, Eigen::SparseMatrix<double> &hessian)> objectiveHessianFunctor)
		:
		m_objectiveHessianFunctor(objectiveHessianFunctor),
		m_hasSparsityPattern(false)
	{
	}
	~UnconstrainedNewtonLinearSolverLLT();

	Eigen::SparseMatrix<double> getHessian(const Eigen::VectorXd& x)
	{
		m_objectiveHessianFunctor(x, m_hessian);
		return m_hessian;
	}


	Eigen::SparseMatrix<double> getHessian()
	{
		return m_hessian;
	}
	Eigen::SparseMatrix<double> getShiftedHessian()
	{
		Eigen::SparseMatrix<double> identity(m_hessian.rows(), m_hessian.cols());
		identity.setIdentity();
		
		return m_hessian + identity * m_solver.getShift();
	}

	virtual void setX(const Eigen::VectorXd &x)
	{
		m_objectiveHessianFunctor(x, m_hessian);
	}
	virtual void setSparsityPattern(Eigen::SparseMatrix<double> mat)
	{
		m_hessian = std::move(mat);
		m_solver.analyzePattern(m_hessian);
		m_hasSparsityPattern = true;
	}
	virtual void factorize()
	{
		if (!m_hasSparsityPattern)
		{
			m_solver.analyzePattern(m_hessian);
		}
		m_solver.factorize(m_hessian);
	}
	virtual void setShift(double val)
	{
		m_solver.setShift(val);
	}
	virtual void solve(const Eigen::VectorXd &rhs, Eigen::VectorXd &solution)
	{
		solution = m_solver.solve(rhs);
	}
	virtual Eigen::ComputationInfo info() const
	{
		return m_solver.info();
	}

	virtual void debugHessian() override;

private:
	LLT<Eigen::SparseMatrix<double>, Eigen::Upper > m_solver;

	Eigen::SparseMatrix<double> m_hessian;
	std::function<void(const Eigen::VectorXd& x, Eigen::SparseMatrix<double> &hessian)> m_objectiveHessianFunctor;
	bool m_hasSparsityPattern;
};

/** this classes use case is more efficiency, thus we assume that the sparsity pattern of the solver will be set before! */
class UnconstrainedNewtonLinearSolverLLTWrapper : public UnconstrainedNewtonLinearSolver
{
public:
	/** assumes solver already analyzed pattern */
	UnconstrainedNewtonLinearSolverLLTWrapper(
		LLT<Eigen::SparseMatrix<double>, Eigen::Upper > *solver,
		Eigen::SparseMatrix<double> *hessian,
		std::function<void(const Eigen::VectorXd& x, Eigen::SparseMatrix<double> &hessian)> objectiveHessianFunctor);

	Eigen::SparseMatrix<double> getHessian(const Eigen::VectorXd& x)
	{
		m_objectiveHessianFunctor(x, *m_hessian);
		return *m_hessian;
	}

	Eigen::SparseMatrix<double> getHessian()
	{
		return *m_hessian;
	}

	Eigen::SparseMatrix<double> getShiftedHessian()
	{
		Eigen::SparseMatrix<double> identity(m_hessian->rows(), m_hessian->cols());
		identity.setIdentity();
		
		return *m_hessian + identity * m_solver->getShift();
	}

	virtual void setX(const Eigen::VectorXd &x);
	virtual void factorize();
	virtual void setShift(double val);
	virtual void solve(const Eigen::VectorXd &rhs, Eigen::VectorXd &solution);
	virtual Eigen::ComputationInfo info() const;

	virtual void debugHessian() override;

private:
	LLT<Eigen::SparseMatrix<double>, Eigen::Upper > *m_solver;

	Eigen::SparseMatrix<double> *m_hessian;
	std::function<void(const Eigen::VectorXd& x, Eigen::SparseMatrix<double> &hessian)> m_objectiveHessianFunctor;

};

enum class lineSearchMethod { ls_backtrack = 0, ls_backtrackGreedy = 1, ls_fullStep = 2};

class UnconstrainedNewton
{
public:
	UnconstrainedNewton(std::unique_ptr<UnconstrainedNewtonLinearSolver> linearSolver);

	void setX(const Eigen::VectorXd& x)
	{
		m_x = x;
	}

	const Eigen::VectorXd& getX() const
	{
		return m_x;
	}

	void setObjective(std::function<double(const Eigen::VectorXd& x)> objectiveFunctor)
	{
		m_objectiveFunctor = objectiveFunctor;
	}
	void setObjectiveGradient(std::function<void(const Eigen::VectorXd& x, Eigen::VectorXd& grad)> objectiveGradientFunctor)
	{
		m_objectiveGradientFunctor = objectiveGradientFunctor;
	}
	void setLargestLineSearchStepFunctor(std::function<double(const Eigen::VectorXd& x, const Eigen::VectorXd& dx)> largestLineSearchStepFunctor)
	{
		m_largestLineSearchStepFunctor = largestLineSearchStepFunctor;
	}
	void setStepBeginningFunctor(std::function<bool(const Eigen::VectorXd& x, const int currentIteration)> stepBeginningFunctor)
	{
		m_stepBeginningFunctor = stepBeginningFunctor;
	}
	void setSearchDirectionFilter(std::function<void(Eigen::VectorXd& searchDir)> searchDirectionFilterFunctor)
	{
		m_searchDirectionFilterFunctor = searchDirectionFilterFunctor;
	}
	//usually used for langragian function
	void setUpdateOperationBeforeLineSearch(std::function<void(Eigen::VectorXd& x, Eigen::VectorXd& searchDir)> updateOperatorBeforeLineSearchFunctor)
	{
		m_updateOperatorBeforeLineSearchFunctor = updateOperatorBeforeLineSearchFunctor;
	}


	double computeSearchDirection(const Eigen::VectorXd &x, const Eigen::VectorXd& gradient, Eigen::VectorXd &dx, double start_reg = 1e-6);
	void solve();
	void polish();
	void polish_test();

	void setGradientThreshold(double gradientThreshold)
	{
		m_gradientThreshold = gradientThreshold;
	}

	/*** dx threshold is only used when no step can be taken */
	void setDxThreshold(double dxThreshold)
	{
		m_dxThreshold = dxThreshold;
	}

	void setMaxIter(int maxIter)
	{
		m_maxIter = maxIter;
	}

	void setPrintLevel(int printLevel)
	{
		m_printLevel = printLevel;
	}

	void setAfterLineSearchFunctor(std::function<void(const Eigen::VectorXd & x, const Eigen::VectorXd & dx, double& alpha, int iteration)> afterLineSearchFunctor)
	{
		m_afterLineSearchFunctor = afterLineSearchFunctor;
	}

	void setNewXAcceptedCallback(std::function<void(Eigen::VectorXd& x)> callback)
	{
		m_newXAcceptedCallback = callback;
	}

	enum STATE
	{
		INITIALIZED, SOLVE_FAILED, SOLVED
	};

	static const char* toString(STATE state);

	STATE state() const
	{
		return m_state;
	}

	void setLinearSolver(std::unique_ptr<UnconstrainedNewtonLinearSolver> linearSolver)
	{
		m_linearSolver = std::move(linearSolver);
	}
	int getIterCount() const
	{
		return m_iter;
	}
	void setDisableWarnOutput(bool disableWarnOutput)
	{
		m_disableWarnOutput = disableWarnOutput;
	}
	void setCheckForSPDAtSolution(bool check)
	{
		m_checkForSPDAtSolution = check;
	}
	void setStartRegularization(double startReg)
	{
		m_startRegularization = startReg;
	}
	void setLineSearchMethod(lineSearchMethod ls_method)
	{
		m_lineSearchMethod = ls_method;
	}

private:
	Eigen::VectorXd m_x;
	std::function<double(const Eigen::VectorXd& x)> m_objectiveFunctor;
	std::function<void(const Eigen::VectorXd& x, Eigen::VectorXd& grad)> m_objectiveGradientFunctor;
	std::function<double(const Eigen::VectorXd& x, const Eigen::VectorXd& dx)> m_largestLineSearchStepFunctor = 
		[](const Eigen::VectorXd& x, const Eigen::VectorXd& dx) {return 1.0; }; //default functor
	std::function<bool(const Eigen::VectorXd& x, const int currentIteration)> m_stepBeginningFunctor = [](const Eigen::VectorXd& x, const int currentIteration) { return true; };
	std::function<void(Eigen::VectorXd& searchDir)> m_searchDirectionFilterFunctor = [](Eigen::VectorXd& searchDir) {};
	std::function<void(Eigen::VectorXd& x, Eigen::VectorXd& searchDir)> m_updateOperatorBeforeLineSearchFunctor = [](Eigen::VectorXd& x, Eigen::VectorXd& searchDir) {};
	std::function<void(const Eigen::VectorXd & x, const Eigen::VectorXd & dx, double& alpha, int iteration)> m_afterLineSearchFunctor = [](const Eigen::VectorXd & x, const Eigen::VectorXd & dx, double& alpha, int iteration) {};
	int m_maxIter;
	double m_gradientThreshold;
	double m_dxThreshold;
	STATE m_state;
	std::function<void(Eigen::VectorXd &x)> m_newXAcceptedCallback;
	std::unique_ptr<UnconstrainedNewtonLinearSolver> m_linearSolver;
	int m_iter;
	bool m_disableWarnOutput;
	int m_printLevel;
	double m_startRegularization;
	bool m_checkForSPDAtSolution;
	lineSearchMethod m_lineSearchMethod;
};

}
