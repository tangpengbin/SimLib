#ifndef SO_UTILS_H
#define SO_UTILS_H
#pragma once

#include "SOTypes.h"
//#include <glm/glm.hpp>
#define USE_TBB
#include "tbb/tbb.h"

class TriMesh;

namespace soutil
{
	V2D get2D(int i, const Eigen::VectorXd& vx);
	void add2D(int i, const V2D& v, Eigen::VectorXd& vx);
	void set2D(int i, const V2D& v, Eigen::VectorXd& vx);

	V3D get3D(int i, const Eigen::VectorXd& vx);
	void add3D(int i, const V3D& v, Eigen::VectorXd& vx);
	void set3D(int i, const V3D& v, Eigen::VectorXd& vx);
	void scale(Eigen::VectorXd& vx, double s);
	V3D crossProd3D(const V3D& u, const V3D& v);
	void stdToEigenVec(const std::vector<double>& vin, Eigen::VectorXd& vout);


	template <typename T>
	std::string to_string_with_precision(const T a_value, const int n = 6)
	{
		std::ostringstream out;
		out.precision(n);
		out << std::fixed << a_value;
		return out.str();
	}
	/// Determine rotation matrix from coordinate system 1 (vectors x1, y1, z1) to coordinate system 2 (vectors x2, y2, z2)
	Eigen::Matrix3d computeRotationFromTwoCoordinateSystems(Eigen::Vector3d x1, Eigen::Vector3d y1, Eigen::Vector3d z1, Eigen::Vector3d x2, Eigen::Vector3d y2, Eigen::Vector3d z2);

	void copyTriMesh(const TriMesh* in, TriMesh* out);

	// project a symmetric real matrix to the nearest SPD matrix
	template <typename Scalar, int size>
	static void makePD(Eigen::Matrix<Scalar, size, size>& symMtr, bool usePD = false)
	{
#ifndef DISABLE_MAKE_POSITIVE_DEFINITE
		if (usePD)
		{
			Eigen::SelfAdjointEigenSolver<Eigen::Matrix<Scalar, size, size>> eigenSolver(symMtr);
			if (eigenSolver.eigenvalues()[0] >= 0.0) {
				return;
			}
			Eigen::DiagonalMatrix<Scalar, size> D(eigenSolver.eigenvalues());
			int rows = ((size == Eigen::Dynamic) ? symMtr.rows() : size);
			for (int i = 0; i < rows; i++) {
				if (D.diagonal()[i] < 0.0) {
					D.diagonal()[i] = 0.0;
				}
				else {
					break;
				}
			}
			symMtr = eigenSolver.eigenvectors() * D * eigenSolver.eigenvectors().transpose();
		}
#endif // !DISABLE_MAKE_POSITIVE_DEFINITE
	}

	template<int vn, int tp, int tr, int tc>
	void tensorMulVector1(const std::vector<Eigen::Matrix<double, tr,tc>>& in_tensor, const Eigen::Vector<double, vn>& in_vec, Eigen::Matrix<double, vn, tp>& out_matrix)
	{
		out_matrix = Eigen::MatrixXd(in_vec.size(), in_tensor.size());
		out_matrix.setZero();

		//#if defined(_OPENMP)
		//#pragma omp parallel for
		//#endif
		for (int i = 0; i < in_tensor.size(); i++)
		{
			//out_matrix.col(i) = (in_tensor[i] * in_vec).sparseView();
			Eigen::VectorXd col_i = in_tensor[i] * in_vec;//becomes a col
			for (int j = 0; j < col_i.size(); j++)
			{
				if (col_i[j] != 0.0)
					out_matrix(j, i) = col_i[j];
			}
		}

	}
	
	template<int vn, int tp, int tr,int tc>
	void vectorTransposeMulTensor(const Eigen::Vector<double, vn>& in_vec, const std::vector<Eigen::Matrix<double, tr, tc>>& in_tensor, Eigen::Matrix<double, tp, tc>& out_matrix)
	{
		//assert(tp == in_tensor.size());
		//for symmetric tensor, this will give the same result for vectorTransposeMulTensor2
		//out_matrix = Eigen::MatrixXd(tp, tc);//page * cols
		out_matrix.setZero();

		Eigen::RowVectorXd in_vec_transpose = in_vec.transpose();

		for (int i = 0; i < in_tensor.size(); i++)
		{
			out_matrix.row(i) = in_vec_transpose * in_tensor[i]; //each row
		}

	}

	template<int vn, int tp, int tr, int tc>
	void tensorMulVector2(const std::vector<Eigen::Matrix<double, tr, tc>>& in_tensor, const Eigen::Vector<double, vn>& in_vec, Eigen::Matrix<double, tr, tc>& out_matrix)
	{
		out_matrix = Eigen::MatrixXd(in_tensor[0].rows(), in_tensor[0].cols());
		out_matrix.setZero();

		//#if defined(_OPENMP)
		//#pragma omp parallel for
		//#endif
		for (int i = 0; i < in_tensor.size(); i++)
		{
			out_matrix += in_tensor[i] * in_vec[i];
		}
	}
	template<int ti_rows, int ti_cols, int to_rows, int to_cols>
	void setTensorBlock(const std::vector<Eigen::Matrix<double, ti_rows, ti_cols>>& inputTensor, std::vector<Eigen::Matrix<double, to_rows, to_cols>>& outputTensor, int output_startPage, int output_startRow, int output_startCol)
	{//this function will set all of inputTensor to the outputTensor
		int numPages = inputTensor.size();
		int numRows = inputTensor[0].rows();
		int numCols = inputTensor[0].cols();
		//assert(outputTensor.size() >= output_startPage + numPages);
		//assert(to_rows >= output_startRow + numRows);
		//assert(to_cols >= output_startCol + numCols);

#if defined(_OPENMP)
#pragma omp parallel for
#endif
		for (int i = 0; i < numPages; i++)
		{
			outputTensor[output_startPage + i].block(output_startRow, output_startCol, numRows, numCols) = inputTensor[i];
		}
	}

	template<int ti_rows, int ti_cols, int to_rows, int to_cols>
	void setTensorBlock(const std::vector<Eigen::Matrix<double, ti_rows, ti_cols>>& inputTensor, int input_startPage, int input_startRow, int input_startCol, int input_numPages, int input_numRows, int input_numCols,
		std::vector<Eigen::Matrix<double, to_rows, to_cols>>& outputTensor, int output_startPage, int output_startRow, int output_startCol)
	{//this function will select part of the InputTensor to the outputTensor

		//assert(outputTensor.size() >= output_startPage + input_numPages);
		//assert(to_rows >= output_startRow + input_numRows);
		//assert(to_cols >= output_startCol + input_numCols);

#if defined(_OPENMP)
#pragma omp parallel for
#endif
		for (int i = 0; i < input_numPages; i++)
		{
			outputTensor[output_startPage + i].block(output_startRow, output_startCol, input_numRows, input_numCols) =
				inputTensor[input_startPage + i].block(input_startRow, input_startCol, input_numRows, input_numCols);
		}
	}




	//sinc stuff
	double sinc(const double& x);
	/// Compute the L2 norm with abs
	double absL2norm(const Eigen::Vector3d& x);
	/// Compute sinc(||x||)
	double sinc_normx(const Eigen::Vector3d& x);
	// Compute gradient of sinc(||x||)
	Eigen::Vector3d sinc_normx_grad(const Eigen::Vector3d& x);
	// Compute hessian sinc(||x||)
	Eigen::Matrix3d sinc_normx_hess(const Eigen::Vector3d& x);


	//hat operation, construction of the skew-symmetric (cross-product) matrix
	Eigen::Matrix3d Hat(const Eigen::Vector3d& x);

	//construct rotation stuff, identity is (0,0,0)
	Eigen::Matrix3d construct_rotation_matrix(const Eigen::Vector3d& r);
	void construct_rotation_matrix_jacobian(const Eigen::Vector3d& r, std::vector<Eigen::Matrix3d>& drotation_dr);
	void construct_rotation_matrix_hessians(const Eigen::Vector3d& r, std::vector<std::vector<Eigen::Matrix3d>> &d2rotation_dr2);
	template <typename Derived, typename T>
	Eigen::Quaternion<T> construct_quaternion(const Eigen::MatrixBase<Derived>& r);
}
#endif