#ifndef DOF_ELEMENT_BASE_H
#define DOF_ELEMENT_BASE_H

#include "ElementBase.h"

class DofElementBase : public ElementBase
{
public:

	DofElementBase(){}

	void setDofIndices(const std::vector<int> &dofInd) { m_dofIdcs = dofInd; }
	const std::vector<int>& getDofIndices() const { return m_dofIdcs; }
	void setParameterIndices(const std::vector<int> &pInd) { m_pInd = pInd; }
	virtual Eigen::VectorXd gatherVector(const Eigen::VectorXd &v) const override
	{//this function will gather all dofs related to this element together to a vector to compute energy etc.
		Eigen::VectorXd result(m_dofIdcs.size());
		for (int i = 0; i < m_dofIdcs.size(); i++)
		{
			result(i) = v[m_dofIdcs[i]];
		}
		return result;
	}
	static Eigen::VectorXd gatherVector(const Eigen::VectorXd& v, const std::vector<int>& dofs)
	{//this function will gather all dofs related to this element together to a vector to compute energy etc.
		Eigen::VectorXd result(dofs.size());
		for (int i = 0; i < dofs.size(); i++)
		{
			result(i) = v[dofs[i]];
		}
		return result;
	}

	template<typename Derived>
	void scatherAdd(const Eigen::MatrixBase< Derived> &vlocal, Eigen::VectorXd &vGlobal) const
	{//add local sub vector to a global vector according to indices
		assert(vlocal.size() == m_dofIdcs.size());
		for (int i = 0; i < m_dofIdcs.size(); i++)
		{
			vGlobal[m_dofIdcs[i]] += vlocal[i];
		}
	}
	template<typename Derived>
	static void scatherAdd(const Eigen::MatrixBase< Derived>& vlocal, Eigen::VectorXd& vGlobal, const std::vector<int>& dofs)
	{//add local sub vector to a global vector according to indices
		assert(vlocal.size() == dofs.size());
		for (int i = 0; i < dofs.size(); i++)
		{
			vGlobal[dofs[i]] += vlocal[i];
		}
	}

	template<typename Derived>
	void scatherAdd(const Eigen::MatrixBase< Derived> &mLocal, TripVec& triplets) const
	{//add local sub matrix to a global sparse matrix according to indices
		assert(mLocal.rows() == m_dofIdcs.size());
		assert(mLocal.cols() == m_dofIdcs.size());
		for (int i = 0; i < m_dofIdcs.size(); i++)
		{
			for (int j = 0; j < m_dofIdcs.size(); j++)
			{
				triplets.emplace_back(m_dofIdcs[i], m_dofIdcs[j], mLocal(i, j));
			}
		}
	}
	template<typename Derived>
	static void scatherAdd(const Eigen::MatrixBase< Derived>& mLocal, TripVec& triplets, const std::vector<int>& dofs)
	{//add local sub matrix to a global sparse matrix according to indices
		assert(mLocal.rows() == dofs.size());
		assert(mLocal.cols() == dofs.size());
		for (int i = 0; i < dofs.size(); i++)
		{
			for (int j = 0; j < dofs.size(); j++)
			{
				triplets.emplace_back(dofs[i], dofs[j], mLocal(i, j));
			}
		}
	}

	template<typename Derived>
	void scatherParameterJacAdd(const Eigen::MatrixBase< Derived> &mLocal, TripVec& triplets) const
	{//add local sub matrix (parameter) to a global sparse matrix according to indices
		assert(mLocal.rows() == m_dofIdcs.size());
		assert(mLocal.cols() == m_pInd.size());
		for (int j = 0; j < m_pInd.size(); j++)
		{
			if (m_pInd[j] == -1) continue;
			for (int i = 0; i < m_dofIdcs.size(); i++)
			{
				triplets.emplace_back(m_dofIdcs[i], m_pInd[j], mLocal(i, j));
			}
		}
	}
	
	bool hasParameters() const
	{
		for (int idx : m_pInd) if (idx != -1) return true;
		return false;
	}

protected:
	std::vector<int> m_dofIdcs;
	std::vector<int> m_pInd;
};

#endif