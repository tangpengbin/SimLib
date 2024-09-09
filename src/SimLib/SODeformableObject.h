/*=====================================================================================*/
/*! \file		DeformableObject.h
\author		bthomasz
\brief		Declaration of DeformableObject class
*/
/*=====================================================================================*/

#ifndef SO_DEFORMABLE_OBJECT_H
#define SO_DEFORMABLE_OBJECT_H

#include "./Core/SOTypes.h"
#include "./Elements/ElementBase.h"
#include "./Core/SOMaterial.h"
#include "./Geom/TriMesh.h"
#include "./Viewer/PrimitiveDrawer.h"
#include <set>
#include <map>
using namespace std;

class SODeformableObjectBase
{
public:
	SODeformableObjectBase() {}
	virtual ~SODeformableObjectBase() {}

	virtual Eigen::VectorXd& getCurrentPositions() = 0;
	virtual Eigen::VectorXd& getCurrentVelocities() = 0;
	virtual const Eigen::VectorXd& getRestPositions() const = 0;
	virtual void setPositions(const Eigen::VectorXd& pos) = 0;
	virtual void setVelocities(const Eigen::VectorXd& vel) = 0;
	//virtual int getNumVtx() const = 0;
	virtual TriMesh* outputMesh() = 0;

	//virtual double getGravitationalEnergy(const Eigen::VectorXd& x, const std::set<int>& unfiltered) = 0;
	//virtual void addGravityForce(Eigen::VectorXd& f) = 0;
	//virtual void addGravityForceFiltered(Eigen::VectorXd& f, const std::set<int>& unfiltered) = 0;
		
	//create matrix A, where columns are concatenated columns of the transformed internal energy Hessians J^T H_xx J
	virtual double getElasticEnergy(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX) = 0;
	virtual void addGradient(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, Eigen::VectorXd& vf) = 0;
	virtual void addHessian(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, SpMat & A) {};

	virtual void setStepSize(double dt) = 0;
	virtual double& getStepSize( ) = 0;
	virtual void setGravity(double g) = 0;
	virtual double getGravity() = 0;

	//virtual void clearPointConstraints() = 0;
};

class SODeformableObject : public SODeformableObjectBase
{
public:
	SODeformableObject();
	virtual ~SODeformableObject();

	//virtual methods
	virtual void init(SOMaterial* mat);
	virtual void step();
	virtual void reset();
	virtual void setVisualizationConfigurationInitial();
	virtual void setVisualizationConfigurationUndeformed();
	virtual void solveStaticState();
	//constant rest state (use precomputed variables)
	virtual void addGradient(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, Eigen::VectorXd& vf);
	virtual void computeHessian(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, SpMat & A);
	virtual void addGradient(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, const Eigen::VectorXd& vxo, Eigen::VectorXd& vf);
	virtual void computeHessian(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, const Eigen::VectorXd& vxo, SpMat& A);

	//compute internal and external derivatives
	virtual void computeExternalForce(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, Eigen::VectorXd& ext_f);
	virtual void computeInternalForce(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, Eigen::VectorXd& int_f);
	virtual void compute_dInternaldx(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, SpMat& dInternaldx);
	virtual void compute_dInternaldX(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, SpMat& dInternaldX);
	
	virtual Eigen::VectorXd applyMassMatrix(const Eigen::VectorXd& v) = 0;
	virtual Eigen::VectorXd applyInvMassMatrix(const Eigen::VectorXd& v) = 0;
	virtual void addMassMatrix(Eigen::SparseMatrix<double> &mat) = 0;
	virtual SpMat computeMassMatrix() = 0;
	virtual Eigen::VectorXd computeGravityForce() = 0;

	virtual TriMesh* outputMesh() { return m_outputMesh; }
	virtual TriMesh* getTriMesh() { return m_outputMesh; }

	//virtual void staticSolver_evalRes(const Eigen::VectorXd& vx, Eigen::VectorXd& vres, double loadFactor);
	virtual void computeMasses(const Eigen::VectorXd& x)=0;

	//common methods, only implemented in base class
	void setElementMaterial(int iElem, const SOMaterial* mat)
	{
		assert(iElem<m_elements.size());
		m_elements[iElem]->setMaterial(mat);
	}
	const SOMaterial* getElementMaterial(int iElem) 
	{ 
		assert(iElem<m_elements.size()); 
		return m_elements[iElem]->getMaterial(); 
	}
	const SOMaterial* getElementMaterial(int iElem) const
	{
		assert(iElem<m_elements.size()); 
		return m_elements[iElem]->getMaterial(); 
	}
	void setMaterial(SOMaterial* mat) 
	{ 
		m_material = mat; 
		for(int i=0; i<(int)m_elements.size(); i++)
			m_elements[i]->setMaterial(m_material);
	}
	virtual void setStepSize(double dt) { m_dt = dt; }
	virtual double& getStepSize(){return m_dt;}
	virtual void setGravity(double g) { m_g = g; }
	virtual double getGravity() {return m_g;}
	//void setAppliedForces(const Eigen::VectorXd& vForces) { m_appliedForces = vForces;}
	//virtual int getNumVtx() const { return (int)m_x.size()/dimension; }
	virtual int getDoFCount() const { return (int)m_x.size(); }
	virtual int getNumPoints() const { return (int)m_x.size() / 3;}
	void setConstraintDoFIndices(vector<int> &cIndices);
	std::vector<int> getConstraintDofIdcsAsVector() const;
	//void setConstraintPositions(vector<double> &cPositions) { m_constraintPositions = cPositions; }
	//void setConstraints(const QVector<int>& vInds);
	bool isDoFConstrained(int dofIdx) const { return (m_constraints.find(dofIdx) != m_constraints.end()); }
	//bool isConstrained(int iVtx) const { return (std::find(m_constraintIndices.begin(), m_constraintIndices.end(), iVtx) != m_constraintIndices.end()); }
	void setConstrainedDimension(int dofIdx, double val);
	void setConstrainedPosition(int iVtx, const V3D& pos) ;
	void applyConstraints(Eigen::VectorXd& pos);
	void updateConstraintTargets(const Eigen::VectorXd& vx);
	void clearConstraints();
	void setSolverResidual(double r) { m_solverResidual = r; }
	double getSolverResidual() { return m_solverResidual; }
	void setSolverMaxIterations(int m) { m_solverMaxIterations = m; }
	int getSolverMaxIterations() { return m_solverMaxIterations; }
	void setNewtonSolveResidual(double r) { m_newtonSolveResidual = r; }
	double getNewtonSolveResidual() { return m_newtonSolveResidual; }
	
	void setLoadingSteps(int n) { m_loadingSteps = n; }
	void setMaxNewtonIterations(int n) {m_newtonMaxIterations = n; }
	void setMaxLinesearchSteps(int n) {m_newtonMaxLinesearchSteps = n; }
	virtual double getElasticEnergy(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX);

	Eigen::VectorXd& getCurrentPositions(){	return m_x; }
	Eigen::VectorXd& getCurrentVelocities(){ return m_v; }
	void setRestPosition(Eigen::VectorXd &restPos) { m_X = restPos; }
	Eigen::VectorXd& getRestPositions(){ return m_X; }
	const Eigen::VectorXd& getRestPositions() const { return m_X; }
	Eigen::VectorXd& getReferenceParamPosition() { return m_para_X; }
	const Eigen::VectorXd& getParamPosition() { return m_para_X; }
	void setPositions(const Eigen::VectorXd& pos){ m_x = pos; }
	void setVelocities(const Eigen::VectorXd& vel){	m_v = vel; }

	std::vector<ElementBase*>& getElements() { return m_elements; }
	virtual void setVisualizationConfiguration(const Eigen::VectorXd& vx) {  }
	virtual Eigen::VectorXd getVisualizationConfiguration() { return m_outputMesh->vertices();	}
	meshDrawer* getMeshDrawer() { return m_meshDrawer; }
	PrimitiveDrawer *getPrimitiveDrawer() { return m_primitiveDrawer; }
	void setMeshDrawer(meshDrawer* meshDrawerInstance) { m_meshDrawer = meshDrawerInstance; }
	void setPrimitiveDrawer(PrimitiveDrawer *primitiveDrawerInstance) { m_primitiveDrawer = primitiveDrawerInstance; }
	virtual void render() = 0;
	virtual void renderImGUIViewerWindow() = 0;


	void filter(Eigen::VectorXd& v);
	//void filter(Eigen::VectorXd& v, const vector<int> & constraintIndices);
	void filterMat(SpMat & A, double diagValue=1.0);
	//void filterMatVec(SpMat & A, Eigen::VectorXd & r, const vector<int> & constraintIndices, double diagValue=1.0);
	void copyConstrainedValues(const Eigen::VectorXd &from, Eigen::VectorXd &to);

	virtual void updateOutputMesh() {}

	//elementwise computation
	virtual double computeElementEnergy(int index);
	virtual void computeElementGradient(int index, Eigen::VectorXd& grad, std::vector<int>& dofIndices);
	virtual void computeElementHessian(int index, Eigen::MatrixXd& Hes, std::vector<int>& dofIndices);
	bool getApplyElement(int index) { return m_elements[index]->getApplyElement(); }
protected:
	virtual void initElements(const Eigen::VectorXd& vX) = 0;
	virtual void reconstructElementVector() = 0;

	//static solver
	//static solver
	virtual bool staticSolve_newton_deprecated(Eigen::VectorXd& vx, const Eigen::VectorXd& vX);
	virtual bool staticSolve_newton(Eigen::VectorXd& vx, const Eigen::VectorXd& vX) = 0;
	virtual double staticSolve_evalObj(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX);
	virtual void staticSolve_evalRes(const Eigen::VectorXd& vx, const Eigen::VectorXd& vX, Eigen::VectorXd& res);
	virtual int solveLinearSystem(SpMat& A, const Eigen::VectorXd& rhs, Eigen::VectorXd& x);

protected:
	//mesh for rendering
	TriMesh* m_outputMesh;
	meshDrawer* m_meshDrawer;
	PrimitiveDrawer* m_primitiveDrawer;

	Eigen::VectorXd visualize_m_x;
	bool visualize_isDirty;
	
	SOMaterial*				m_material;
	std::vector<ElementBase*>		m_elements;

	//Linear solver parameters
	//SparseLinSolver*				m_pSolver;
	double							m_solverResidual;
	int								m_solverMaxIterations;
	double							m_newtonSolveResidual;

	//Nonlinear solver parameters
	int m_loadingSteps;
	int m_newtonMaxIterations;
	int m_newtonMaxLinesearchSteps;

	map<int, double>						m_constraints;
	std::vector<int>						m_constraintIdxToDofIdx; // this gives order to m_constraints

	Eigen::VectorXd							m_appliedForces;
	double							m_loadFactor;

	SpMat m_A;
	Eigen::VectorXd m_x;
	Eigen::VectorXd m_X;
	Eigen::VectorXd m_v;
	Eigen::VectorXd m_V;
	
	//this x can be edited by parameter of the shape
	Eigen::VectorXd m_para_X;
	
	double m_dt;
	double m_g;
	bool USE_GRAVITY;

	bool simulationLog;

};


//#include "SODeformableObject.hpp"

#endif
