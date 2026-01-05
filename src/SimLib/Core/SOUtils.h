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
	V2D get2D(int i, const Eigen::VectorX<ScalarType> &vx);
	void add2D(int i, const V2D& v, Eigen::VectorX<ScalarType>& vx);
	void set2D(int i, const V2D& v, Eigen::VectorX<ScalarType>& vx);

	V3D get3D(int i, const Eigen::VectorX<ScalarType>& vx);
	void add3D(int i, const V3D& v, Eigen::VectorX<ScalarType>& vx);
	void set3D(int i, const V3D& v, Eigen::VectorX<ScalarType>& vx);
	void scale(Eigen::VectorX<ScalarType>& vx, ScalarType s);
	V3D crossProd3D(const V3D& u, const V3D& v);
	void stdToEigenVec(const std::vector<ScalarType>& vin, Eigen::VectorX<ScalarType>& vout);

	Eigen::Vector4<ScalarType> get4D(int i, const Eigen::VectorX<ScalarType>& vx);

	template <typename T>
	std::string to_string_with_precision(const T a_value, const int n = 6)
	{
		std::ostringstream out;
		out.precision(n);
		out << std::fixed << a_value;
		return out.str();
	}
	Eigen::Matrix3<ScalarType> crossProductMatrix(const Eigen::Vector3<ScalarType> &v);

	ScalarType computeTetVolume(Eigen::Vector3<ScalarType> v0, Eigen::Vector3<ScalarType> v1, Eigen::Vector3<ScalarType> v2, Eigen::Vector3<ScalarType> v3);

	//center
	void computeCenterPosition(const Eigen::VectorX<ScalarType> element_x, Eigen::Vector3<ScalarType>& centerPosition);
	void computeCenterPositionJacobian(const Eigen::VectorX<ScalarType> element_x, SpMat& jacobian);
	//hessian is zero
	//void computeCenterPositionHessian(const Eigen::VectorX<ScalarType> element_x, std::vector<SpMat>& hessians);

	//Eigen::Matrix4f toEigen(const glm::mat4 &mat);
	Eigen::Vector3<ScalarType> projectPointOntoLine(const Eigen::Vector3<ScalarType>& point, const Eigen::Vector3<ScalarType>& linePassthoughPoint, const Eigen::Vector3<ScalarType>& lineVector);
	Eigen::Vector3<ScalarType> computePositionAroundAxis(const Eigen::Vector3<ScalarType>& rotate_point, const Eigen::Vector3<ScalarType>& through_p, const Eigen::Vector3<ScalarType>& dir, ScalarType sinTheta, ScalarType cosTheta);

	void soDebug(const char* szDebug);
	void copyTriMesh(const TriMesh* in, TriMesh* out);

	/// Determine rotation matrix from coordinate system 1 (vectors x1, y1, z1) to coordinate system 2 (vectors x2, y2, z2)
	Eigen::Matrix3<ScalarType> computeRotationFromTwoCoordinateSystems(Eigen::Vector3<ScalarType> x1, Eigen::Vector3<ScalarType> y1, Eigen::Vector3<ScalarType> z1, Eigen::Vector3<ScalarType> x2, Eigen::Vector3<ScalarType> y2, Eigen::Vector3<ScalarType> z2);

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


	class TriTriplet {
	public:
		int key[3];

		TriTriplet(const int* p_key)
		{
			key[0] = p_key[0];
			key[1] = p_key[1];
			key[2] = p_key[2];
		}
		TriTriplet(int key0, int key1, int key2)
		{
			key[0] = key0;
			key[1] = key1;
			key[2] = key2;
		}

		bool operator<(const TriTriplet& right) const
		{
			if (key[0] < right.key[0]) {
				return true;
			}
			else if (key[0] == right.key[0]) {
				if (key[1] < right.key[1]) {
					return true;
				}
				else if (key[1] == right.key[1]) {
					if (key[2] < right.key[2]) {
						return true;
					}
				}
			}
			return false;
		}
	};
	void findSurfaceTris(const Eigen::MatrixXi& TT, Eigen::MatrixXi& F);
	void findSurfaceTris_parallel(const Eigen::MatrixXi& TT, Eigen::MatrixXi& F);
	void getAllTriangleMeshFromTet(const Eigen::MatrixXi& T, Eigen::MatrixXi& F);
	bool readTetMesh(const std::string& filePath,
		Eigen::MatrixX<ScalarType>& TV, Eigen::MatrixXi& TT,
		Eigen::MatrixXi& SF, bool findSurface = true);
	
	void logPrint(const char *format, ...);
	void harwell_boeing(const SpMat& A, int & num_rows, std::vector<ScalarType> & V, std::vector<int> & R, std::vector<int> & C);
	ScalarType computeL1Norm(const Eigen::SparseMatrix<ScalarType> &mat);

	//matrix regularization solver
	bool sparseMatrixRegularizationSolver();

	//helper functions, tensor for sparse matrix
	void tensorAddTensor(const std::vector<SpMat> &in_tensor1, const std::vector<SpMat> &in_tensor2, std::vector<SpMat>& out_tensor);
	void tensorMulMatrix(const std::vector<SpMat> &in_tensor, const SpMat &in_matrix, std::vector<SpMat>& out_tensor);
	void tensorTransposeMulVector(const std::vector<SpMat>& in_tensor, const Eigen::VectorX<ScalarType>& in_vec, SpMat& out_matrix);
	void tensorMulVector1(const std::vector<SpMat> &in_tensor, const Eigen::VectorX<ScalarType> &in_vec, SpMat& out_matrix);
	void tensorMulVector2(const std::vector<SpMat>& in_tensor, const Eigen::VectorX<ScalarType>& in_vec, SpMat& out_matrix);
	void vectorMulTensor(const Eigen::VectorX<ScalarType> & in_vec, const std::vector<SpMat>& in_tensor, SpMat& out_matrix);
	void vectorTransposeMulTensor(const Eigen::VectorX<ScalarType>& in_vec, const std::vector<SpMat>& in_tensor, SpMat& out_matrix);
	void vectorTransposeMulTensor2(const Eigen::VectorX<ScalarType>& in_vec, const std::vector<SpMat>& in_tensor, SpMat& out_matrix);
	void matrixMulTensor(const SpMat& in_matrix, const std::vector<SpMat>& in_tensor, std::vector<SpMat>& out_tensor);
	void tensorTranspose_inplace(std::vector<SpMat>& transpose_tensor);
	std::vector<SpMat> tensorTranspose(const std::vector<SpMat>& transpose_tensor);
	std::vector<SpMat> getTensorBlock(const std::vector<SpMat> &inputTensor, int startPage, int startRow, int startCol, int numPages, int numRows, int numCols);

	ScalarType doubleContraction(const SpMat A, const SpMat &B);

	//helper functions, tensor for dense matrix
	template<int n, int m>
	void tensorAddTensor(const std::vector<Eigen::Matrix<ScalarType,n,m>>& in_tensor1, const std::vector<Eigen::Matrix<ScalarType, n, m>>& in_tensor2, std::vector<Eigen::Matrix<ScalarType, n, m>>& out_tensor)
	{
		//assert(in_tensor1.size() == in_tensor2.size() && in_tensor1[0].rows() == in_tensor2[0].rows() && in_tensor1[0].cols() == in_tensor2[0].cols());

		out_tensor.resize(in_tensor1.size());

//#if defined(_OPENMP)
//#pragma omp parallel for
//#endif
		for (int i = 0; i < in_tensor1.size(); i++)
		{
			out_tensor[i] = in_tensor1[i] + in_tensor2[i];
		}
	}
	void tensorMulMatrix(const std::vector<Eigen::MatrixX<ScalarType>>& in_tensor, const Eigen::MatrixX<ScalarType>& in_matrix, std::vector<Eigen::MatrixX<ScalarType>>& out_tensor);

	/*
	!!!!!important note!!!!!!!!!!!
	It worth to know that if dv/dx * tensor, the dx will be col in the compute tensor (in this case, we have to transpose col and page)
	if A * dB/dx, the dx will be the page (this case is right)
	*/
	template<int vn, int tp, int tr, int tc>
	void tensorTransposeMulVector(const std::vector<Eigen::Matrix<ScalarType, tr, tc>>& in_tensor, const Eigen::Vector<ScalarType, vn>& in_vec, Eigen::Matrix<ScalarType, tc, tp>& out_matrix)
	{
		out_matrix = SpMat(in_tensor[0].cols(), in_tensor.size());
		out_matrix.setZero();
		for (int page_k = 0; page_k < tp; page_k++)
		{
			Eigen::Vector<ScalarType, tc> col_k = in_tensor[page_k].transpose() * in_vec;
			for (int row_i = 0; row_i < col_k.size(); row_i++)
				out_matrix(row_i, page_k) = col_k[row_i];
		}
	}
	template<int vn, int tp, int tr, int tc>
	void tensorMulVector1(const std::vector<Eigen::Matrix<ScalarType, tr,tc>>& in_tensor, const Eigen::Vector<ScalarType, vn>& in_vec, Eigen::Matrix<ScalarType, vn, tp>& out_matrix)
	{
		out_matrix = Eigen::MatrixX<ScalarType>(in_vec.size(), in_tensor.size());
		out_matrix.setZero();

		//#if defined(_OPENMP)
		//#pragma omp parallel for
		//#endif
		for (int i = 0; i < in_tensor.size(); i++)
		{
			//out_matrix.col(i) = (in_tensor[i] * in_vec).sparseView();
			Eigen::VectorX<ScalarType> col_i = in_tensor[i] * in_vec;//becomes a col
			for (int j = 0; j < col_i.size(); j++)
			{
				if (col_i[j] != 0.0)
					out_matrix(j, i) = col_i[j];
			}
		}

	}
	template<int vn, int tp, int tr, int tc>
	void tensorMulVector2(const std::vector<Eigen::Matrix<ScalarType, tr, tc>>& in_tensor, const Eigen::Vector<ScalarType, vn>& in_vec, Eigen::Matrix<ScalarType, tr, tc>& out_matrix)
	{
		out_matrix = Eigen::MatrixX<ScalarType>(in_tensor[0].rows(), in_tensor[0].cols());
		out_matrix.setZero();

		//#if defined(_OPENMP)
		//#pragma omp parallel for
		//#endif
		for (int i = 0; i < in_tensor.size(); i++)
		{
			out_matrix += in_tensor[i] * in_vec[i];
		}
	}
	void vectorMulTensor(const Eigen::VectorX<ScalarType>& in_vec, const std::vector<Eigen::MatrixX<ScalarType>>& in_tensor, Eigen::MatrixX<ScalarType>& out_matrix);
	template<int vn, int tp, int tr,int tc>
	void vectorTransposeMulTensor(const Eigen::Vector<ScalarType, vn>& in_vec, const std::vector<Eigen::Matrix<ScalarType, tr, tc>>& in_tensor, Eigen::Matrix<ScalarType, tp, tc>& out_matrix)
	{
		//assert(tp == in_tensor.size());
		//for symmetric tensor, this will give the same result for vectorTransposeMulTensor2
		//out_matrix = Eigen::MatrixX<ScalarType>(tp, tc);//page * cols
		out_matrix.setZero();

		Eigen::RowVectorX<ScalarType> in_vec_transpose = in_vec.transpose();

		for (int i = 0; i < in_tensor.size(); i++)
		{
			out_matrix.row(i) = in_vec_transpose * in_tensor[i]; //each row
		}

	}
	template<int vn, int tp, int tr, int tc>
	void vectorTransposeMulTensor2(const Eigen::Vector<ScalarType, vn>& in_vec, const std::vector<Eigen::Matrix<ScalarType, tr,tc>>& in_tensor, Eigen::Matrix<ScalarType, tc, tp>& out_matrix)
	{
		//this can apply to something like (dE/dx)^T * d^2x/dq^2
		//out_matrix = Eigen::MatrixX<ScalarType>(in_tensor[0].cols(), in_tensor.size());//cols * page
		assert(tp == in_tensor.size());
		out_matrix.setZero();

		for (int i = 0; i < in_vec.size(); i++)
		{
			Eigen::MatrixX<ScalarType> page_slice(in_tensor[0].cols(), in_tensor.size());
			for (int j = 0; j < in_tensor.size(); j++)
			{
				page_slice.row(j) = in_tensor[j].row(i);
			}
			out_matrix += in_vec[i] * page_slice;
		}
	}
	template <int mr, int mc, int tp, int tr, int tc>
	void matrixSplitTransposeMulTensor(const Eigen::Matrix<ScalarType, mr, mc> &in_matrix, const std::vector<Eigen::Matrix<ScalarType, tr, tc>> in_tensor, std::vector<Eigen::Matrix<ScalarType, tp, tc>> &out_tensor)
	{//this will split matrix to vector and use vectorTransposeMulTensor
		out_tensor.resize(mc);
		for (int i = 0; i < mc; i++)
			vectorTransposeMulTensor<mr, tp, tr, tc>(in_matrix.col(i), in_tensor, out_tensor[i]);
	}
	template <int mr, int mc, int tp, int tr, int tc>
	void matrixTransposeMulTensor(const Eigen::Matrix<ScalarType, mr, mc> &in_matrix, const std::vector<Eigen::Matrix<ScalarType, tr, tc>> in_tensor, std::vector<Eigen::Matrix<ScalarType, mc, tc>> &out_tensor)
	{//this will directly use matrix transpose to mutiply with each page of the tensor
		out_tensor.resize(tp);
		for (int i = 0; i < tp; i++)
			out_tensor[i] = in_matrix.transpose() * in_tensor[i];
	}
	template <int mr, int mc, int tp, int tr, int tc>
	void matrixTransposeMulTensorTranspose(const Eigen::Matrix<ScalarType, mr, mc> &in_matrix, const std::vector<Eigen::Matrix<ScalarType, tr, tc>> in_tensor, std::vector<Eigen::Matrix<ScalarType, mc, tr>> &out_tensor)
	{//this will give a tensor with each page = matrix.transpose * tensor[i].transpose()
		out_tensor.resize(tp);
		for (int i = 0; i < tp; i++)
			out_tensor[i] = in_matrix.transpose() * in_tensor[i].transpose();
	}

	template <int mr, int mc, int tp, int tr, int tc>
	void matrixMulTensor(const Eigen::Matrix<ScalarType, mr, mc> &in_matrix, const std::vector<Eigen::Matrix<ScalarType, tr, tc>> in_tensor, std::vector<Eigen::Matrix<ScalarType, mr, tc>> &out_tensor)
	{
		out_tensor.resize(tp);
		for (int i = 0; i < tp; i++)
			out_tensor[i] = in_matrix * in_tensor[i];
	}
	template <int tp, int tr, int tc, int mr, int mc>
	void tensorRowColTransposeMulMatrix(const std::vector<Eigen::Matrix<ScalarType, tr, tc>> in_tensor, const Eigen::Matrix<ScalarType, mr, mc> &in_matrix, std::vector<Eigen::Matrix<ScalarType, tc, mc>>& out_tensor)
	{
		out_tensor.resize(tp);
		for (int i = 0; i < tp; i++)
			out_tensor[i] = in_tensor[i].transpose() * in_matrix;
	}
	template <int tp, int tr, int tc, int mr, int mc>
	void tensorColPageTransposeMulMatrix(const std::vector<Eigen::Matrix<ScalarType, tr, tc>> in_tensor, const Eigen::Matrix<ScalarType, mr, mc> &in_matrix, std::vector<Eigen::Matrix<ScalarType, tr, mc>>& out_tensor)
	{
		out_tensor.resize(tc);
		for (int i = 0; i < tc; i++)
		{
			Eigen::Matrix<ScalarType, tr, tp> in_tensor_colPage_i;
			for (int pi = 0; pi < tp; pi++)
				in_tensor_colPage_i.col(pi) = in_tensor[pi].col(i);

			out_tensor[i] = in_tensor_colPage_i * in_matrix;
		}
	}
	template <int tp, int tr, int tc, int mr, int mc>
	void tensorRowPageTransposeMulMatrix(const std::vector<Eigen::Matrix<ScalarType, tr, tc>> in_tensor, const Eigen::Matrix<ScalarType, mr, mc> &in_matrix, std::vector<Eigen::Matrix<ScalarType, tp, mc>>& out_tensor)
	{
		out_tensor.resize(tr);
		for (int i = 0; i < tr; i++)
			for (int pi = 0; pi < tp; pi++)
				out_tensor[i].row(pi) = in_tensor[pi].row(i) * in_matrix;
	}
	template <int tp, int tr, int tc>
	std::vector<Eigen::Matrix<ScalarType, tr, tp>> tensorColPageTranspose(const std::vector<Eigen::Matrix<ScalarType, tr, tc>> &in_tensor)
	{//this function will transpose colume to page
		std::vector<Eigen::Matrix<ScalarType, tr, tp>> outTensor(tc);
		for (int i = 0; i < tc; i++)
		{
			for (int pi = 0; pi < tp; pi++)
				outTensor[i].col(pi) = in_tensor[pi].col(i);
		}
		return outTensor;
	}
	template <int tp, int tr, int tc>
	std::vector<Eigen::Matrix<ScalarType, tp, tc>> tensorRowPageTranspose(const std::vector<Eigen::Matrix<ScalarType, tr, tc>> &in_tensor)
	{//this function will transpose colume to page
		std::vector<Eigen::Matrix<ScalarType, tp, tc>> outTensor(tr);
		for (int i = 0; i < tr; i++)
		{
			for (int pi = 0; pi < tp; pi++)
				outTensor[i].row(pi) = in_tensor[pi].row(i);
		}
		return outTensor;
	}
	template <int tp, int tr, int tc>
	std::vector<Eigen::Matrix<ScalarType, tc, tr>> tensorRowColTranspose(const std::vector<Eigen::Matrix<ScalarType, tr, tc>> &in_tensor)
	{//this function will transpose colume to page
		std::vector<Eigen::Matrix<ScalarType, tc, tr>> outTensor(tp);
		for (int i = 0; i < tp; i++)
		{
			outTensor[i] = in_tensor[i].transpose();
		}
		return outTensor;
	}
	template<int vn, int tg, int tp, int tr, int tc>
	void vectorTransposeMulFourthOrderTensor(const Eigen::Vector<ScalarType, vn>& in_vector, const std::vector<std::vector<Eigen::Matrix<ScalarType, tr, tc>>>& in_fourthOrderTensor, std::vector<Eigen::Matrix<ScalarType, tp, tc>>& out_tensor)
	{
		out_tensor.resize(tg);
		for (int gi = 0; gi < tg; gi++)
			vectorTransposeMulTensor<vn, tp, tr, tc>(in_vector, in_fourthOrderTensor[gi], out_tensor[gi]);
	}
	template<int mr, int mc, int tg, int tp, int tr, int tc>
	void matrixTransposeMulFourthOrderTensor(const Eigen::Matrix<ScalarType, mr, mc>& in_matrix, const std::vector<std::vector<Eigen::Matrix<ScalarType, tr, tc>>>& in_fourthOrderTensor, std::vector<std::vector<Eigen::Matrix<ScalarType, tp, tc>>>& out_fourthOrderTensor)
	{
		out_fourthOrderTensor.resize(tg);
		for (int gi = 0; gi < tg; gi++)
			//vectorTransposeMulTensor<vn, tp, tr, tc>(in_vector, in_fourthOrderTensor[gi], out_tensor[gi]);
			matrixSplitTransposeMulTensor<mr, mc, tp, tr, tc>(in_matrix, in_fourthOrderTensor[gi], out_fourthOrderTensor[gi]);
	}
	template<int tg, int tp, int tr, int tc, int vn>
	void fourthOrderTensorMulVector(const std::vector<std::vector<Eigen::Matrix<ScalarType, tr, tc>>>& in_fourthOrderTensor, const Eigen::Vector<ScalarType, vn>& in_vector, std::vector<Eigen::Matrix<ScalarType, vn, tp>>& out_tensor)
	{
		out_tensor.resize(tg);
		for (int gi = 0; gi < tg; gi++)
			tensorMulVector1<tp, tr, tc, vn>(in_fourthOrderTensor[gi], in_vector, out_tensor[gi]);
	}

	template<int tp, int tr, int tc>
	void tensorMulConstant(const std::vector<Eigen::Matrix<ScalarType, tr, tc>>& in_tensor, ScalarType c, std::vector<Eigen::Matrix<ScalarType, tr, tc>>& out_tensor)
	{
		out_tensor.resize(tp);
		for (int i = 0; i < tp; i++)
		{
			out_tensor[i] = c * in_tensor[i];
		}
	}

	template<int tp, int tr, int tc, int mr, int mc>
	void tensorMulMatrix(const std::vector<Eigen::Matrix<ScalarType, tr, tc>>& in_tensor,const Eigen::Matrix<ScalarType, mr, mc>& in_matrix, std::vector<Eigen::Matrix<ScalarType, tr, mc>> &out_tensor)
	{
		out_tensor.resize(tp);

		for (int i = 0; i < tp; i++)
		{
			out_tensor[i] = in_tensor[i] * in_matrix;
		}
	}
	template<int mr, int mc, int tp, int tr, int tc>
	void matrixTransposeMulTensorMulMatrix(const Eigen::Matrix<ScalarType, mr, mc>& in_matrix, const std::vector<Eigen::Matrix<ScalarType, tr, tc>>& in_tensor, std::vector<Eigen::Matrix<ScalarType, tr, mc>>& out_tensor)
	{
		std::vector<Eigen::Matrix<ScalarType, tr, mc>> temp(tp);
		for (int i = 0; i < in_tensor.size(); i++)
		{
			temp[i] = in_tensor[i] * in_matrix;
		}


		out_tensor.resize(mc);
		for (int col_i = 0; col_i < mc; col_i++)
		{
			Eigen::Matrix<ScalarType, tr, tp> colMatrix;
			for (int i = 0; i < tp; i++)
			{
				colMatrix.col(i) = temp[i].col(col_i);
			}
			out_tensor[col_i] = colMatrix * in_matrix;
		}
	}

	void vectorTransposeMulTensorxd(const Eigen::VectorX<ScalarType>& in_vec, const std::vector<Eigen::MatrixX<ScalarType>>& in_tensor, Eigen::MatrixX<ScalarType>& out_matrix);
	void matrixMulTensor(const Eigen::MatrixX<ScalarType>& in_matrix, const std::vector<Eigen::MatrixX<ScalarType>>& in_tensor, std::vector<Eigen::MatrixX<ScalarType>>& out_tensor);
	void matrixMulTensor(const Eigen::MatrixX<ScalarType>& in_matrix, const std::vector<Eigen::MatrixX<ScalarType>>& in_tensor, Eigen::VectorX<ScalarType>& out_vector);
	void matrixTransposeMulTensorMulMatrix(const Eigen::MatrixX<ScalarType>& in_matrix, const std::vector<Eigen::MatrixX<ScalarType>>& in_tensor, std::vector<Eigen::MatrixX<ScalarType>>& out_tensor);
	void tensorTranspose_inplace(std::vector<Eigen::MatrixX<ScalarType>>& transpose_tensor);
	std::vector<Eigen::MatrixX<ScalarType>> tensorTranspose(const std::vector<Eigen::MatrixX<ScalarType>>& transpose_tensor);

	std::vector<Eigen::MatrixX<ScalarType>> getTensorBlock(const std::vector<Eigen::MatrixX<ScalarType>>& inputTensor, int startPage, int startRow, int startCol, int numPages, int numRows, int numCols);

	template<int ti_rows, int ti_cols, int to_rows, int to_cols>
	void setTensorBlock(const std::vector<Eigen::Matrix<ScalarType, ti_rows, ti_cols>>& inputTensor, std::vector<Eigen::Matrix<ScalarType, to_rows, to_cols>>& outputTensor, int output_startPage, int output_startRow, int output_startCol)
	{//this function will set all of inputTensor to the outputTensor
		int numPages = inputTensor.size();
		int numRows = inputTensor[0].rows();
		int numCols = inputTensor[0].cols();
		//assert(outputTensor.size() >= output_startPage + numPages);
		//assert(to_rows >= output_startRow + numRows);
		//assert(to_cols >= output_startCol + numCols);

//#if defined(_OPENMP)
//#pragma omp parallel for
//#endif
		for (int i = 0; i < numPages; i++)
		{
			outputTensor[output_startPage + i].block(output_startRow, output_startCol, numRows, numCols) = inputTensor[i];
		}
	}

	template<int ti_rows, int ti_cols, int to_rows, int to_cols>
	void setTensorBlock(const std::vector<Eigen::Matrix<ScalarType, ti_rows, ti_cols>>& inputTensor, int input_startPage, int input_startRow, int input_startCol, int input_numPages, int input_numRows, int input_numCols,
		std::vector<Eigen::Matrix<ScalarType, to_rows, to_cols>>& outputTensor, int output_startPage, int output_startRow, int output_startCol)
	{//this function will select part of the InputTensor to the outputTensor

		//assert(outputTensor.size() >= output_startPage + input_numPages);
		//assert(to_rows >= output_startRow + input_numRows);
		//assert(to_cols >= output_startCol + input_numCols);

//#if defined(_OPENMP)
//#pragma omp parallel for
//#endif
		for (int i = 0; i < input_numPages; i++)
		{
			outputTensor[output_startPage + i].block(output_startRow, output_startCol, input_numRows, input_numCols) =
				inputTensor[input_startPage + i].block(input_startRow, input_startCol, input_numRows, input_numCols);
		}
	}

	ScalarType doubleContraction(const Eigen::MatrixX<ScalarType> A, const Eigen::MatrixX<ScalarType> &B);

	template <const int n, const int m>
	Eigen::Vector<ScalarType, n * m> matrixVectorization(const Eigen::Matrix<ScalarType, n, m>& A)
	{
		return Eigen::Map<const Eigen::Vector<ScalarType, n * m>>(A.data(), n * m);
	}
	template <const int p, const int n, const int m>
	Eigen::Matrix<ScalarType, n * m, p> tensorVectorization(const std::vector<Eigen::Matrix<ScalarType, n, m>>& A)
	{
		assert(p == A.size());
		Eigen::Matrix<ScalarType, n * m, p> out;
		for (int i = 0; i < p; i++)
		{
			int A_i = 0;
			for (int col_i = 0; col_i < A[i].cols(); col_i++)
				for (int row_i = 0; row_i < A[i].rows(); row_i++)
				{
					out.col(i)[A_i] = A[i](row_i, col_i);
					A_i++;
				}
		}

		return out;
	}
	Eigen::MatrixX<ScalarType> tensorVectorization(const std::vector<Eigen::MatrixX<ScalarType>> &A);

	//mkl pinv
	//void mkl_pinv(Eigen::MatrixX<ScalarType> &A, Eigen::MatrixX<ScalarType> &inva);

	template<typename Mat, int size>
	void polar_eigen(const Mat& A, Mat& R)
	{//https://matthias-research.github.io/pages/publications/MeshlessDeformations_SIG05.pdf
		typedef Eigen::Matrix<typename Mat::Scalar, size, 1> Vec;
		Eigen::SelfAdjointEigenSolver<Mat> eig;
		eig.computeDirect(A.transpose() * A);
		Vec S = eig.eigenvalues().cwiseSqrt();

		R = A * eig.eigenvectors() * S.asDiagonal().inverse()
			* eig.eigenvectors().transpose();
	}

	template<class type>
	void concatenateVectors(std::vector<type>& addToVector, const std::vector<type>& addFromVector)
	{
		addToVector.insert(addToVector.end(), addFromVector.begin(), addFromVector.end());
	}

	template <class ScalarType, class MatrixType>
	MatrixType computeRotationMatrixFromEulerAngles(const ScalarType& x_angle, const ScalarType& y_angle, const ScalarType& z_angle)
	{
		MatrixType yaw; yaw.setZero();
		MatrixType pitch; pitch.setZero();
		MatrixType roll; roll.setZero();

		//z rotation
		yaw(0, 0) = cos(z_angle);	yaw(0, 1) = -sin(z_angle);
		yaw(1, 0) = sin(z_angle);	yaw(1, 1) = cos(z_angle);
		yaw(2, 2) = 1.0;
		//y rotation
		pitch(0, 0) = cos(y_angle); pitch(0, 2) = sin(y_angle);
		pitch(1, 1) = 1.0;
		pitch(2, 0) = -sin(y_angle); pitch(2, 2) = cos(y_angle);
		//x rotation
		roll(0, 0) = 1.0;
		roll(1, 1) = cos(x_angle); roll(1, 2) = -sin(x_angle);
		roll(2, 1) = sin(x_angle); roll(2, 2) = cos(x_angle);

		MatrixType rotationMatrix = yaw * pitch * roll;
		return rotationMatrix;
	}

	template <class ScalarType, class VectorType, class MatrixType>
	VectorType computeEulerAnglesFromRotationMatrix(const MatrixType& rotationMatrix)
	{
		//https://stackoverflow.com/questions/15022630/how-to-calculate-the-angle-from-rotation-matrix
		
		ScalarType x_angle = atan2(rotationMatrix(2, 1), rotationMatrix(2, 2));
		ScalarType y_angle = atan2(-rotationMatrix(2, 0), sqrt(rotationMatrix(2, 1) * rotationMatrix(2, 1) + rotationMatrix(2, 2) * rotationMatrix(2, 2)));
		ScalarType z_angle = atan2(rotationMatrix(1, 0), rotationMatrix(0, 0));
		
		return VectorType(x_angle, y_angle, z_angle);
	}


	//parallel transport
	template<class ScalarType, class VectorType>
	class RotationAngleAxis
	{
	public:

		RotationAngleAxis(const ScalarType& angle, const VectorType& axis)
			:m_angle(angle), m_axis(axis)
		{

		}

		VectorType operator*(const VectorType& x) const
		{
			using std::cos;
			using std::sin;
			ScalarType cosa = cos(m_angle);
			ScalarType sina = sin(m_angle);
			return cosa * x + sina * (m_axis.cross(x)) + (m_axis.dot(x) * (ScalarType(1.0) - cosa)) * m_axis;
		}

		VectorType applyToOrthogonal(const VectorType& x) const
		{
			using std::cos;
			using std::sin;
			ScalarType cosa = cos(m_angle);
			ScalarType sina = sin(m_angle);
			return cosa * x + sina * (m_axis.cross(x));
		}

	private:
		ScalarType m_angle;
		VectorType m_axis;
	};

	template<class ScalarType, class VectorType>
	class ParallelTransport
	{
	public:

		ParallelTransport(const VectorType& vFrom, const VectorType& vTo)
			:m_vFrom(vFrom), m_vTo(vTo)
		{

		}

		VectorType operator*(const VectorType& x) const
		{
			return x * (m_vFrom.dot(m_vTo)) + (m_vFrom.cross(m_vTo)).cross(x) + (m_vFrom.cross(m_vTo)) * ((m_vFrom.cross(m_vTo)).dot(x) / (ScalarType(1.0) + m_vFrom.dot(m_vTo)));
		}

	private:
		VectorType m_vFrom;
		VectorType m_vTo;
	};


	//sinc stuff
	ScalarType sinc(const ScalarType& x);
	/// Compute the L2 norm with abs
	ScalarType absL2norm(const Eigen::Vector3<ScalarType>& x);
	/// Compute sinc(||x||)
	ScalarType sinc_normx(const Eigen::Vector3<ScalarType>& x);
	// Compute gradient of sinc(||x||)
	Eigen::Vector3<ScalarType> sinc_normx_grad(const Eigen::Vector3<ScalarType>& x);
	// Compute hessian sinc(||x||)
	Eigen::Matrix3d sinc_normx_hess(const Eigen::Vector3<ScalarType>& x);


	//hat operation, construction of the skew-symmetric (cross-product) matrix
	Eigen::Matrix3d Hat(const Eigen::Vector3<ScalarType>& x);

	//construct rotation stuff, identity is (0,0,0)
	Eigen::Matrix3d construct_rotation_matrix(const Eigen::Vector3<ScalarType>& r);
	void construct_rotation_matrix_jacobian(const Eigen::Vector3<ScalarType>& r, std::vector<Eigen::Matrix3d>& drotation_dr);
	void construct_rotation_matrix_hessians(const Eigen::Vector3<ScalarType>& r, std::vector<std::vector<Eigen::Matrix3d>> &d2rotation_dr2);
	template <typename Derived, typename T>
	Eigen::Quaternion<T> construct_quaternion(const Eigen::MatrixBase<Derived>& r);


	template<int nv>
	Eigen::Vector<ScalarType, 3 * nv> rigidTransformVertex(const Eigen::Vector3<ScalarType>& T, const Eigen::Vector3<ScalarType>& r, const Eigen::Vector<ScalarType, 3 * nv>& v)
	{
		Eigen::Matrix3d rotation = construct_rotation_matrix(r);
		Eigen::MatrixX<ScalarType> v_cols = Eigen::Map<const Eigen::MatrixX<ScalarType>>(v.data(), 3, nv);
		v_cols = (rotation * v_cols).eval();
		v_cols.colwise() += T;
		Eigen::Vector<ScalarType, 3 * nv> output_v = Eigen::Map<Eigen::Vector<ScalarType, 3 * nv>>(v_cols.data(), 3 * nv);
		return output_v;
	}

	template<int nv>
	Eigen::Matrix<ScalarType, 3 * nv, 6>  rigidTransformVertexJacobian(const Eigen::Vector3<ScalarType>& T, const Eigen::Vector3<ScalarType>& r, const Eigen::Vector<ScalarType, 3 * nv>& v)
	{
		Eigen::Matrix<ScalarType, 3 * nv, 6> dtransformedVertexRB_dx;
		dtransformedVertexRB_dx.setZero();

		std::vector<Eigen::Matrix3d> drotationdr;
		soutil::construct_rotation_matrix_jacobian(r, drotationdr);

		tbb::parallel_for(
			tbb::blocked_range<size_t>(size_t(0), nv),
			[&](const tbb::blocked_range<size_t>& r) {
				for (size_t v_i = r.begin(); v_i < r.end(); v_i++)
					//for (int v_i = 0; v_i < v.size() / 3; v_i++)
				{
					//jacobian of translation part
					for (int t_i = 0; t_i < 3; t_i++)
					{
						dtransformedVertexRB_dx(v_i * 3 + t_i, t_i) = 1.0;
					}
					//dtransformedVertexRB_dx.block(v_i * 3, 0, 3, 3).setIdentity();

					//jacobian of rotation part
					for (int r_col = 0; r_col < 3; r_col++)
					{
						Eigen::Vector3<ScalarType> dcol = drotationdr[r_col] * v.segment(v_i * 3, 3);
						for (int r_row = 0; r_row < 3; r_row++)
						{
							dtransformedVertexRB_dx(3 * v_i + r_row, 3 + r_col) = dcol[r_row];
						}
					}
				}
			});

		return dtransformedVertexRB_dx;
	}

	template<int nv>
	std::vector<Eigen::Matrix<ScalarType, 3 * nv, 6>> rigidTransformVertexHessians(const Eigen::Vector3<ScalarType>& T, const Eigen::Vector3<ScalarType>& r, const Eigen::Vector<ScalarType, 3 * nv>& v)
	{
		std::vector<std::vector<Eigen::Matrix3d>> d2rotationdr2;
		soutil::construct_rotation_matrix_hessians(r, d2rotationdr2);

		std::vector<Eigen::Matrix<ScalarType, 3 * nv, 6>> d2transformedVertexRB_dx2;
		d2transformedVertexRB_dx2.resize(6);
		for (int theta_i = 0; theta_i < 6; theta_i++)
		{
			d2transformedVertexRB_dx2[theta_i].setZero();
		}

		for (int v_i = 0; v_i < nv; v_i++)
		{
			//hessian of translation is zero
			//hessian of rotation d2x / dtheta2
			for (int theta_l = 0; theta_l < 3; theta_l++)
			{
				for (int theta_k = 0; theta_k < 3; theta_k++)
				{
					//d theta_k d theta_l
					Eigen::Vector3<ScalarType> d2x_dthetak_dthetal = d2rotationdr2[theta_l][theta_k] * v.segment(v_i * 3, 3);
					for (int row_i = 0; row_i < 3; row_i++)
						d2transformedVertexRB_dx2[3 + theta_l]( //theta l page
							3 * v_i + row_i, //vertex row
							3 + theta_k)  //theta k col)
						= d2x_dthetak_dthetal[row_i];
				}
			}
		}

		return d2transformedVertexRB_dx2;
	}

	Eigen::MatrixX<ScalarType> rigidTransformVertices(const Eigen::Vector3<ScalarType> &T, const Eigen::Vector3<ScalarType> &r, const Eigen::MatrixX<ScalarType> &v);
	Eigen::MatrixX<ScalarType> rigidTransformVerticesJacobian(const Eigen::Vector3<ScalarType> &T, const Eigen::Vector3<ScalarType> &r, const Eigen::MatrixX<ScalarType> &v);
	std::vector<Eigen::MatrixX<ScalarType>> rigidTransformVerticesHessians(const Eigen::Vector3<ScalarType> &T, const Eigen::Vector3<ScalarType> &r, const Eigen::MatrixX<ScalarType> &v);
}
#endif