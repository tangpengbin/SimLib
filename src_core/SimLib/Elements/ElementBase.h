/*=====================================================================================*/
/*! \file		ElementBase.h
\author		bthomasz
\brief		Declaration of ElementBase class
*/
/*=====================================================================================*/

#ifndef ELEMENT_BASE_H
#define ELEMENT_BASE_H

#include "../Core/SOTypes.h"
#include "../Core/SOMaterial.h"
#include <exception>

class SOMaterial;

enum elementProperty { InternalElement, ExternalElement }; //this shows the property of the elemetn: internal force contribution or external force contribution

class ElementBase
{
public:
	

	ElementBase(void) { m_elemProperty = InternalElement; };//default property
	~ElementBase(void) {};
	//virtual void init(const std::vector<int>& inds, const Eigen::VectorXd& x, const SOMaterial* pMat) = 0;
	void setElementProperty(elementProperty elem_property) { m_elemProperty = elem_property; }
	elementProperty getElementProperty() { return m_elemProperty; }

	virtual const std::vector<int>& getDofIndices() const = 0;
	
	//constant rest state (use precomputed variables)
	virtual void addGradient(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, Eigen::VectorXd& vf) = 0;
	virtual void addHessian(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, SpMat & A) = 0;
	virtual void addTriplets(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, TripVec& triplets) = 0;
	virtual double computeEnergy(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX) = 0;
	

	virtual void computeGradient(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, Eigen::VectorXd& grad) {
		throw std::logic_error("not implemented");
	}
	virtual void computeHessian(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, Eigen::MatrixXd& hes) {
		throw std::logic_error("not implemented");
	}

	virtual void addProductJacobianTriplets(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, const Eigen::VectorXd& prodVector, TripVec& triplets)
	{
		throw std::logic_error("not implemented");
	}

	//dE^2/dxdX
	virtual void addRestJacTriplets(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, TripVec& triplets)
	{
		throw std::logic_error("not implemented");
	}
	//dE^2/dxdX
	virtual void addParameterJacTriplets(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, TripVec& triplets)
	{
		throw std::logic_error("not implemented");
	}
	//dE^3/dx^3
	virtual void addThirdOrderTriplets(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, std::vector<TripVec>& vec_triplets) {
		throw std::logic_error("not implemented");
	}
	//dE^3/(dx^2dX)
	virtual void addRestConfThirdOrderTriplets(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, std::vector<TripVec>& vec_triplets) {
		throw std::logic_error("not implemented");
	}
	
	virtual void addParameterThirdOrderTriplets(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, std::vector<TripVec>& vec_triplets)
	{
		throw std::logic_error("not implemented");
	}

	//v1 * dE^4/(dx^4) * v2
	virtual void addTransformedFourthOrderTriplets(const Eigen::VectorXd&vx, const Eigen::VectorXd& vX, const Eigen::VectorXd& v1, const Eigen::VectorXd &v2, TripVec& triplets) {
		throw std::logic_error("not implemented");
	}
	//v1 * dE^4/(dx^3dX) * v2
	virtual void addTransformedRestConfFourthOrderTriplets(const Eigen::VectorXd&vx, const Eigen::VectorXd& vX, const Eigen::VectorXd& v1, const Eigen::VectorXd &v2, TripVec& triplets) {
		throw std::logic_error("not implemented");
	}

	void setMaterial(const SOMaterial* mat) { m_mat = mat; }
	//SOMaterial* getMaterial() { return m_mat; }
	const SOMaterial* getMaterial() const { return m_mat; }

	virtual Eigen::VectorXd gatherVector(const Eigen::VectorXd &v) const = 0;

	void updateApplyElement(bool applyElement) { m_applyElement = applyElement; }
	bool getApplyElement() { return m_applyElement; }
protected:
	const SOMaterial* m_mat;
	elementProperty m_elemProperty;
	bool m_applyElement;
};

#endif
