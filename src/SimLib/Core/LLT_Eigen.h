#pragma once

#ifndef SIM_OPT_WITH_INTEL_MKL

#include <Eigen/Sparse>

namespace SimOpt
{

template<typename _MatrixType, int _Options = Eigen::Upper>
class LLT_Eigen
{
public:
	LLT_Eigen()
	{

	}
	void setShift(typename _MatrixType::Scalar offset)
	{
		m_decomposition.setShift(offset);
		m_shiftOffset = offset;
	}

	typename _MatrixType::Scalar getShift()
	{
		return m_shiftOffset;
		//return m_decomposition.m_shiftOffset;
	}
	template<typename MatType>
	void compute(const MatType& mat)
	{
		m_decomposition.compute(mat);
	}

	void analyzePattern(const _MatrixType &mat)
	{
		m_decomposition.analyzePattern(mat);
	}

	void factorize(const _MatrixType &mat)
	{
		m_decomposition.factorize(mat);
	}

	template<typename Rhs>
	const Eigen::Solve<Eigen::SimplicialLLT<_MatrixType, _Options>, Rhs> solve(const Eigen::MatrixBase<Rhs>& vec) const
	{
		return m_decomposition.solve(vec);
	}

	Eigen::ComputationInfo info() const
	{
		return m_decomposition.info();
	}
private:
	Eigen::SimplicialLLT<_MatrixType, _Options> m_decomposition;
	typename _MatrixType::Scalar m_shiftOffset;
};

}

#endif
