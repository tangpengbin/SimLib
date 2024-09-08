#pragma once

#include "LBFGS.h"
#include <Eigen/Core>
#include <Eigen/Sparse>

#include <functional>

namespace SimOpt
{

class UnconstrainedLBFGS
{
public:
	UnconstrainedLBFGS();

	void setX(const Eigen::VectorXd &x)
	{
		m_state = INITIALIZED;
		m_x = x;
	}

	const Eigen::VectorXd& getX() const
	{
		return m_x;
	}

	void setObjective(std::function<double(const Eigen::VectorXd&x)> objectiveFunctor)
	{
		m_objectiveFunctor = objectiveFunctor;
	}
	void setObjectiveGradient(std::function<void(const Eigen::VectorXd& x, Eigen::VectorXd &grad)> objectiveGradientFunctor)
	{
		m_objectiveGradientFunctor = objectiveGradientFunctor;
	}
	void setObjectiveHessian(std::function<void(const Eigen::VectorXd&x, Eigen::SparseMatrix<double> &hessian)> objectiveHessianFunctor)
	{
		m_objectiveHessianFunctor = objectiveHessianFunctor;
	}
	void setLargestLineSearchStepFunctor(std::function<double(const Eigen::VectorXd& x, const Eigen::VectorXd& dx)> largestLineSearchStepFunctor)
	{
		m_largestLineSearchStepFunctor = largestLineSearchStepFunctor;
	}

	void solve();

	void setGradientThreshold(double gradientThreshold)
	{
		m_gradientThreshold = gradientThreshold;
	}

	void setMaxIter(int maxIter)
	{
		m_maxIter = maxIter;
	}

	void setHistorySize(int size)
	{
		m_lbfgs.setHistorySize(size);
	}

	void setNewXAcceptedCallback(std::function<void(const Eigen::VectorXd &x)> callback)
	{
		m_newXAcceptedCallback = callback;
	}

	enum STATE
	{
		INITIALIZED, SOLVE_FAILED, SOLVED
	};

	STATE state() const
	{
		return m_state;
	}
private:
	Eigen::VectorXd m_x;
	std::function<double(const Eigen::VectorXd&x)> m_objectiveFunctor;
	std::function<void(const Eigen::VectorXd& x, Eigen::VectorXd &grad)> m_objectiveGradientFunctor;
	std::function<void(const Eigen::VectorXd&x, Eigen::SparseMatrix<double> &hessian)> m_objectiveHessianFunctor;
	std::function<double(const Eigen::VectorXd& x, const Eigen::VectorXd& dx)> m_largestLineSearchStepFunctor =
		[](const Eigen::VectorXd& x, const Eigen::VectorXd& dx) {return 1.0; }; //default functor
	int m_maxIter;
	double m_gradientThreshold;
	STATE m_state;
	Eigen::SparseMatrix<double> m_hessian;

	LBFGS<double> m_lbfgs;

	std::function<void(const Eigen::VectorXd &x)> m_newXAcceptedCallback;

	//"temporaries"
	Eigen::VectorXd m_gradient;
	Eigen::VectorXd m_dx;
	Eigen::VectorXd m_oldGradient;
	Eigen::VectorXd m_y;
};

}

