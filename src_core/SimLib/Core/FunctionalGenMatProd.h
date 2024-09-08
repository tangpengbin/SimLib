#pragma once

// Copyright (C) 2016-2018 Yixuan Qiu <yixuan.qiu@cos.name>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <functional>

namespace Spectra {


///
/// \ingroup MatOp
///
/// This class defines the matrix-vector multiplication operation on a
/// sparse real matrix \f$A\f$, i.e., calculating \f$y=Ax\f$ for any vector
/// \f$x\f$. It is mainly used in the GenEigsSolver and SymEigsSolver
/// eigen solvers.
///
template <typename Scalar_>
class FunctionalGenMatProd
{
public:
    ///
    /// Element type of the matrix.
    ///
    using Scalar = Scalar_;
private:
    typedef Eigen::Matrix<Scalar_, Eigen::Dynamic, 1> Vector;
    typedef Eigen::Map<const Vector> MapConstVec;
    typedef Eigen::Map<Vector> MapVec;

	std::function<Vector(const Vector &x)> m_applyMat;
	int m_rows;
	int m_cols;
public:
    ///
    /// Constructor to create the matrix operation object.
    ///
    /// \param mat_ An **Eigen** sparse matrix object, whose type is
    /// `Eigen::SparseMatrix<Scalar_, ...>`.
    ///
	FunctionalGenMatProd(std::function<Vector(const Vector &x)> applyMat, int rows, int cols) :
        m_applyMat(applyMat),
		m_rows(rows),
		m_cols(cols)
    {}

    ///
    /// Return the number of rows of the underlying matrix.
    ///
    int rows() const { return m_rows; }
    ///
    /// Return the number of columns of the underlying matrix.
    ///
    int cols() const { return m_cols; }

    ///
    /// Perform the matrix-vector multiplication operation \f$y=Ax\f$.
    ///
    /// \param x_in  Pointer to the \f$x\f$ vector.
    /// \param y_out Pointer to the \f$y\f$ vector.
    ///
    // y_out = A * x_in
    void perform_op(const Scalar_* x_in, Scalar_* y_out) const
    {
        MapConstVec x(x_in,  m_cols);
        MapVec      y(y_out, m_rows);
        y.noalias() = m_applyMat(x);
    }
};


} // namespace Spectra
