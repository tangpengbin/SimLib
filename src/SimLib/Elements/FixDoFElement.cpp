#include "FixDoFElement.h"

#include "../Core/SOUtils.h"

#include "../Codegen/computeFixDoFEnergy.h"

using namespace soutil;

//this is 2 dimensional fix
FixDoFElement::FixDoFElement()
	:DofElementBase()
{
	m_stiffness = std::numeric_limits<double>::quiet_NaN();
}


FixDoFElement::~FixDoFElement()
{
}

void FixDoFElement::init(int idx, int pIdx, const double &stiffness, bool applyElement)
{
	setDofIndices({ idx });
	setParameterIndices({ pIdx });
	m_stiffness = stiffness;
	m_applyElement = applyElement;
}
double FixDoFElement::computeEnergy(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX)
{
	if (!m_applyElement)
		return 0.0;
	Eigen::VectorXd x = gatherVector(vx);
	Eigen::VectorXd X = gatherVector(vX);

	double energy;
	Codegen::computeFixDoFEnergy(m_stiffness, x, X, energy);
	
	return energy;
}

void FixDoFElement::computeGradient(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, Eigen::VectorXd& grad)
{
	if (!m_applyElement)
	{
		grad = Eigen::VectorXd(1);
		grad.setZero();
		return;
	}
	Eigen::VectorXd x = gatherVector(vx);
	Eigen::VectorXd X = gatherVector(vX);

	Eigen::Matrix<double, 1, 1> f;
	Codegen::computeFixDoFEnergyGradient(m_stiffness, x, X, f);

	grad = f;
}

void FixDoFElement::addGradient(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, Eigen::VectorXd& vf)
{
	if (!m_applyElement)
	{
		return;
	}
	Eigen::VectorXd x = gatherVector(vx);
	Eigen::VectorXd X = gatherVector(vX);

	Eigen::Matrix<double, 1, 1> f;
	Codegen::computeFixDoFEnergyGradient(m_stiffness, x, X, f);
	f *= -1.0;//force

	scatherAdd(f, vf);
}
void FixDoFElement::computeHessian(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, Eigen::MatrixXd& hes)
{
	if (!m_applyElement)
	{
		hes = Eigen::MatrixXd(1,1);
		hes.setZero();
		return;
	}
	Eigen::VectorXd x = gatherVector(vx);
	Eigen::VectorXd X = gatherVector(vX);

	Eigen::Matrix<double, 1, 1> J;
	Codegen::computeFixDoFEnergyHessian(m_stiffness, x, X, J);
	
	hes = J;
}
void FixDoFElement::addHessian(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, SpMat & A)
{
	throw std::logic_error("not implemented");
}
void FixDoFElement::addTriplets(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, TripVec& triplets)
{
	if (!m_applyElement)
	{
		return;
	}
	Eigen::VectorXd x = gatherVector(vx);
	Eigen::VectorXd X = gatherVector(vX);

	Eigen::Matrix<double, 1, 1> hessian;
	Codegen::computeFixDoFEnergyHessian(m_stiffness, x, X, hessian);

	//Eigen::MatrixXd h = hessian;
	//IPC::IglUtils::makePD<double, Eigen::Dynamic>(h);

	hessian *= -1.0;//dforce / dx

	scatherAdd(hessian, triplets);
}
void FixDoFElement::addProductJacobianTriplets(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, const Eigen::VectorXd& prodVector, TripVec& triplets)
{
	if (!m_applyElement)
	{
		return;
	}
	Eigen::VectorXd x = gatherVector(vx);
	Eigen::VectorXd X = gatherVector(vX);

	Eigen::VectorXd v = gatherVector(prodVector);

	Eigen::Matrix<double, 1, 1> J;
	Codegen::computeFixDoFEnergyHessianProductJacobian(m_stiffness, x, X, v, J);
	//J *= -1.0;//dforce / dx

	scatherAdd(J, triplets);
}
void FixDoFElement::addParameterJacTriplets(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, TripVec& triplets)
{
	if (m_pInd[0] != -1) throw std::logic_error("not implemented");
}
void FixDoFElement::addParameterThirdOrderTriplets(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, std::vector<TripVec>& vec_triplets)
{
	if (m_pInd[0] != -1) throw std::logic_error("not implemented");
}
void FixDoFElement::addRestJacTriplets(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, TripVec& triplets)
{
	if (!m_applyElement)
	{
		return;
	}
	Eigen::VectorXd x = gatherVector(vx);
	Eigen::VectorXd X = gatherVector(vX);

	Eigen::Matrix<double, 1, 1> Jac;
	Codegen::computeFixDoFEnergyRestConfJac(m_stiffness, x, X, Jac);
	//-dF / dX

	scatherAdd(Jac, triplets);
}