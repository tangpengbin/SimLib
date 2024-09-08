/*=====================================================================================*/
/*! \file		SODeformableObject.cpp
\author		bthomasz
\brief		Implementation of class SODeformableObject
*/
/*=====================================================================================*/

#include "SODeformableObject.h"
#include "./Core/SOUtils.h"
#include "./Elements/DofElementBase.h"
#include <iostream>
#include <cmath>
#include <fstream>

#define USE_TBB
#include "tbb/tbb.h"
#include <spdlog/spdlog.h>

const int m_maxCorrSteps = 20;
const double m_wc0 = 1e-4;
using namespace soutil;

SODeformableObject::SODeformableObject() 
{
	m_newtonSolveResidual = 1e-5;
	USE_GRAVITY = true;
	//disable log
	simulationLog = false;
	m_newtonMaxLinesearchSteps = 10;

	m_dt = 0.01;
	m_g = 9.8;

	m_meshDrawer = nullptr;

	visualize_isDirty = true;

}

SODeformableObject::~SODeformableObject() 
{

}

void SODeformableObject::addGradient(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, Eigen::VectorXd& vf)
{
	for (int i = 0; i < (int)m_elements.size(); i++)
	{
		m_elements[i]->addGradient(vx, vX, vf);
	}
	if (USE_GRAVITY)
	{
		vf += computeGravityForce();
	}
	vf += m_appliedForces;
}

void SODeformableObject::computeHessian(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, SpMat & A)
{
	A = SpMat(m_x.size(), m_x.size());
	A.setZero();
	//Column Major Matrix
	TripVec triplets;
	triplets.reserve(6 * m_x.size());

	//add triplets for all edges
	for (int i = 0; i < m_elements.size(); i++)
	{
		m_elements[i]->addTriplets(vx, vX, triplets);
	}
	A.setFromTriplets(triplets.begin(), triplets.end());
	//Convert to CCS
	A.makeCompressed();
}

void SODeformableObject::addGradient(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, const Eigen::VectorXd& vxo, Eigen::VectorXd& vf)
{
	addGradient(vx, vX, vf);
}
void SODeformableObject::computeHessian(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, const Eigen::VectorXd& vxo, SpMat& A)
{
	computeHessian(vx, vX, A);
}

void SODeformableObject::computeExternalForce(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, Eigen::VectorXd& ext_f)
{
	int N = this->getDoFCount();
	ext_f = Eigen::VectorXd(N); ext_f.setZero();

	for (int i = 0; i < m_elements.size(); i++)
	{
		if (m_elements[i]->getElementProperty() == ExternalElement)
			m_elements[i]->addGradient(vx, vX, ext_f);
	}

	if (USE_GRAVITY)
	{
		ext_f += computeGravityForce();
	}
}

void SODeformableObject::computeInternalForce(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, Eigen::VectorXd& int_f)
{
	int N = getDoFCount();
	int_f = Eigen::VectorXd(N); int_f.setZero();
	for (int i = 0; i < m_elements.size(); i++)
	{
		if (m_elements[i]->getElementProperty() == InternalElement) //internal 
			m_elements[i]->addGradient(vx, vX, int_f);    //-dE/dx
	}
}

void SODeformableObject::compute_dInternaldx(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, SpMat& dInternaldx)
{
	int N = getDoFCount();
	dInternaldx = SpMat(N, N);

	TripVec trips;
	for (int i = 0; i < m_elements.size(); i++)
	{
		if (m_elements[i]->getElementProperty() == InternalElement) //internal 
			m_elements[i]->addTriplets(vx, vX, trips);
	}

	dInternaldx.setFromTriplets(trips.begin(), trips.end());
	dInternaldx.makeCompressed();
}

void SODeformableObject::compute_dInternaldX(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, SpMat& dInternaldX)
{
	int N = getDoFCount();
	dInternaldX = SpMat(N, N);

	TripVec trips;
	for (int i = 0; i < m_elements.size(); i++)
	{
		if (m_elements[i]->getElementProperty() == InternalElement) //internal 
			m_elements[i]->addRestJacTriplets(vx, vX, trips);
	}
	
	dInternaldX.setFromTriplets(trips.begin(), trips.end());
	dInternaldX.makeCompressed();
	dInternaldX *= -1.0;
}

void SODeformableObject::init(SOMaterial* mat)
{
	m_material = mat;
	initElements(m_X);
	computeMasses(m_X);
}

void SODeformableObject::step()
{
}

bool SODeformableObject::staticSolve_newton_deprecated(Eigen::VectorXd& vx, const Eigen::VectorXd& vX)
{
	simulationLog = true;
	std::cout << " simulationLog " << simulationLog << std::endl;

	int N = getDoFCount();
	Eigen::VectorXd xi(N); xi.setZero();
	Eigen::VectorXd xe(N); xe.setZero();
	Eigen::VectorXd rhs(N); rhs.setZero();

	copyConstrainedValues(vX, vx);

	xe = vx;
	xi = vx;

	//updateConstraintTargets(vX);

	double norm_0;
	double nrm_res = 1.0;
	int it = 0;
	int max_static_iteration = 300;// m_newtonMaxIterations;
	bool failed = false;
	for (it; it < max_static_iteration && !failed; it++)
	{
		if (it == 0)
		{
			staticSolve_evalRes(xe, vX, rhs);
			filter(rhs);
			nrm_res = rhs.norm();
			
			norm_0 = nrm_res;
		}

		if (nrm_res < m_newtonSolveResidual)
			break;

		//*** compute matrix
		//soutil::logPrint("Rebuilding Matrix..\n");
		SpMat A;
		computeHessian(xe, vX, A);
		A *= -1.0;
		filterMat(A);
		//cout << "Norm of matrix A " << A.norm() << endl;

		Eigen::VectorXd dx(getDoFCount()); dx.setZero();
		int it_lin = solveLinearSystem(A, rhs, dx);
		//soutil::logPrint("Needed %d regularization steps. Residual norm is %f, dv|grad is %f\n", it_lin, nrm_res_lin, dot_lin);
		
		filter(dx);

		double nrm_rel_step = 1;
		int itrel = 0;
		double beta = 0.5;
		Eigen::VectorXd xl(xe);
		double E0 = 0.0;
		E0 = staticSolve_evalObj(xe, vX);
		double E1 = E0;
		for (itrel = 0; ; itrel++)
		{
			xe += dx;
			rhs.setZero();// dcutil::set(rhs, 0.0);
			staticSolve_evalRes(xe, vX, rhs);
			filter(rhs);
			nrm_rel_step = rhs.norm();
			E1 = staticSolve_evalObj(xe, vX);
			//if((nrm_rel_step<nrm_res)||(itrel==m_newtonMaxLinesearchSteps))
			if ((E1 < E0))
				break;
			if (itrel >= m_newtonMaxLinesearchSteps)
			{
				//if (E1 < E0 * 10)//we can go out if it not that large
				{//failed = true;
					break;
				}
			}

			dx *= beta;
			//revert to previous positions
			xe = xl;
		}
		nrm_res = nrm_rel_step;
	}
	vx = xe;

	return(nrm_res < m_newtonSolveResidual);
}

double SODeformableObject::staticSolve_evalObj(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX)
{
	double Eint = 0.0;
	for (int i = 0; i < m_elements.size(); i++)
	{
		Eint += m_elements[i]->computeEnergy(vx, vX);
	}
	double Egrav = 0.0;
	if (USE_GRAVITY)
	{
		Egrav += -(computeGravityForce().dot(vx));
	}


	//Eigen::VectorXd vX(3 * m_nv);

	double EappliedForces = 0.0;
	EappliedForces -= m_appliedForces.dot(vx - vX);

	double E = Eint + Egrav + EappliedForces;

	return E;
}

void SODeformableObject::staticSolve_evalRes(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, Eigen::VectorXd& res)
{
	//rhs is -dWdx
	res.fill(0.0);
	addGradient(vx, vX, res);
}

int SODeformableObject::solveLinearSystem(SpMat& A, const Eigen::VectorXd& rhs, Eigen::VectorXd& x)
{
	int it = 0;
	bool solved = false;
	int it_lin = 0;
	double nrm_res_lin = 0.0;
	double dot_lin = 0.0;
	const int IT_LIN_MAX = 10;
	while (!solved)
	{
		Eigen::SimplicialCholesky<SpMat, Eigen::Upper> chol(A);
		Eigen::VectorXd dmydv = chol.solve(rhs);
		x = dmydv;
		solved = true;
		//check factorization
		if (chol.info() != Eigen::Success)
			solved = false;
		//check redsiduum
		Eigen::VectorXd r = A * x - rhs;
		nrm_res_lin = r.norm();
		if (nrm_res_lin > 10e-8)
			solved = false;
		//check dot product between gradient and search direction
		dot_lin = x.normalized().dot(rhs.normalized());
		if (dot_lin < 0.0)
			solved = false;

		if (solved || it > IT_LIN_MAX)
			break;

		double reg = 1e-4*pow(2, it);
		for (int i = 0; i < A.rows(); i++)
			//for (int j = 0; j<3; j++)
			A.coeffRef(i, i) += reg;
		it++;
	}
	
	return it;

}


void SODeformableObject::filter(Eigen::VectorXd& v)
{
	for (auto &keyValuePair: m_constraints)
	{
		int constrIdx = keyValuePair.first;
		v[constrIdx] = 0.0;
	}
}

void SODeformableObject::filterMat(SpMat & A, double diagValue)
{
	for (auto &keyValuePair: m_constraints)
	{
		int constrIdx = keyValuePair.first;
		for (SpMat::InnerIterator it(A, constrIdx); it; ++it)
		{
			int ii = it.row();
			int jj = it.col();
			if (ii == jj)
			{
				it.valueRef() = diagValue;
			}
			else
			{
				A.coeffRef(ii, jj) = 0.0;
				A.coeffRef(jj, ii) = 0.0;
			}
		}
	}

}
void SODeformableObject::copyConstrainedValues(const Eigen::VectorXd &from, Eigen::VectorXd &to)
{
	for (auto &keyValuePair : m_constraints)
	{
		int constrIdx = keyValuePair.first;
		to[constrIdx] = from[constrIdx];
	}
}

double SODeformableObject::computeElementEnergy(int index)
{
	assert(index < m_elements.size());

	return m_elements[index]->computeEnergy(m_x, m_para_X);
}

void SODeformableObject::computeElementGradient(int index, Eigen::VectorXd& grad, std::vector<int>& dofIndices)
{
	m_elements[index]->computeGradient(m_x, m_para_X, grad);
	if (dynamic_cast<DofElementBase*>(m_elements[index]) != nullptr)
		dofIndices = dynamic_cast<DofElementBase*>(m_elements[index])->getDofIndices(); //we only have dof element
	else
		throw std::logic_error("elements contain vertex element base");
}

void SODeformableObject::computeElementHessian(int index, Eigen::MatrixXd& Hes, std::vector<int>& dofIndices)
{
	m_elements[index]->computeHessian(m_x, m_para_X, Hes);
	if (dynamic_cast<DofElementBase*>(m_elements[index]) != nullptr)
		dofIndices = dynamic_cast<DofElementBase*>(m_elements[index])->getDofIndices(); //we only have dof element
	else
		throw std::logic_error("elements contain vertex element base");
}



void SODeformableObject::setConstraintDoFIndices(vector<int> &cIndices)
{
	clearConstraints();
	for (int cIdx : cIndices)
	{
		setConstrainedDimension(cIdx, std::numeric_limits<double>::quiet_NaN());
	}
}
std::vector<int> SODeformableObject::getConstraintDofIdcsAsVector() const
{
	return m_constraintIdxToDofIdx;
}
void SODeformableObject::setConstrainedDimension(int dofIdx, double val)
{
	if (m_constraints.find(dofIdx) != m_constraints.end())
	{
		m_constraints[dofIdx] = val;
	}
	else
	{
		m_constraints[dofIdx] = val;
		m_constraintIdxToDofIdx.push_back(dofIdx);
	}
}
void SODeformableObject::setConstrainedPosition(int iVtx, const V3D& pos)
{ 
	for (int i = 0; i < 3; i++)
	{
		setConstrainedDimension(3 * iVtx + i, pos[i]);
	}
}

void SODeformableObject::clearConstraints()
{
	m_constraints.clear();
	m_constraintIdxToDofIdx.clear();
}


double SODeformableObject::getElasticEnergy(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX)
{
	double enrg = 0;
	for(int i =0 ; i< (int)m_elements.size(); i++)
		enrg += m_elements[i]->computeEnergy(vx,vX);

	return enrg;
}


void SODeformableObject::applyConstraints(Eigen::VectorXd& pos)
{
	for (auto &keyValuePair : m_constraints)
	{
		int idx = keyValuePair.first;
		pos[idx] = keyValuePair.second;
	}
}

void SODeformableObject::reset()
{
	m_x = m_X;
	m_v.setZero();
}
void SODeformableObject::setVisualizationConfigurationInitial()
{
	setVisualizationConfiguration(m_X);
}
void SODeformableObject::setVisualizationConfigurationUndeformed()
{
	setVisualizationConfiguration(m_para_X);
}

void SODeformableObject::solveStaticState()
{
	if (!staticSolve_newton(getCurrentPositions(), getParamPosition()))
		cout << "The static solve doesn't reach equilibrim state, the clamped modes will have some problems" << endl;
	else
		cout << "The static solve has been correctly done!" << endl;

	
	setVisualizationConfiguration(getCurrentPositions());
}

void SODeformableObject::updateConstraintTargets(const Eigen::VectorXd& vx)
{
	for (auto &keyValuePair : m_constraints)
	{
		int idx = keyValuePair.first;
		keyValuePair.second = vx[idx];
	}

}


