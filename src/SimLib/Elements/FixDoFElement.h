#ifndef FIX_DOF_ELEMENT_H
#define FIX_DOF_ELEMENT_H

#if _MSC_VER > 1000
#pragma once
#endif

#include "DofElementBase.h"

class FixDoFElement :
	public DofElementBase
{
public:
	FixDoFElement();
	~FixDoFElement();

	void init(int idx, int pIdx, const double &stiffness, bool applyElement = false);
	double computeEnergy(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX);

	void computeGradient(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, Eigen::VectorXd& grad);
	void addGradient(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, Eigen::VectorXd& vf);
	void computeHessian(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, Eigen::MatrixXd& hes);
	void addHessian(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, SpMat & A);
	void addTriplets(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, TripVec& triplets);
	void addProductJacobianTriplets(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, const Eigen::VectorXd& prodVector, TripVec& triplets) override;
	virtual void addParameterJacTriplets(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, TripVec& triplets) override;
	virtual void addParameterThirdOrderTriplets(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, std::vector<TripVec>& vec_triplets) override;
	void addRestJacTriplets(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, TripVec& triplets);

protected:
	double m_stiffness;
};

#endif