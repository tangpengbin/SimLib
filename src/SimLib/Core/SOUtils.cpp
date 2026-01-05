#include "SOUtils.h"
#include "SOTypes.h"
#include <stdarg.h>

#include "../Geom/TriMesh.h"
#include <iostream>
//#include <Eigen/PardisoSupport>
//#include "mkl.h"

namespace soutil
{
	V2D get2D(int i, const Eigen::VectorX<ScalarType>& vx)
	{
		assert(vx.size() >= 2 * (i + 1));
		V2D v;
		v(0) = vx(2 * i);
		v(1) = vx(2 * i + 1);
		return v;
	}
	void add2D(int i, const V2D& v, Eigen::VectorX<ScalarType>& vx)
	{
		assert(vx.size() >= 2 * (i + 1));
		vx(2 * i) += v(0);
		vx(2 * i + 1) += v(1);
	}
	void set2D(int i, const V2D& v, Eigen::VectorX<ScalarType>& vx)
	{
		assert(vx.size() >= 2 * (i + 1));
		vx(2 * i) = v(0);
		vx(2 * i + 1) = v(1);
	}
	V3D get3D(int i, const Eigen::VectorX<ScalarType>& vx)
	{
		assert(vx.size() >= 3 * (i + 1));
		V3D v;
		v(0) = vx(3 * i);
		v(1) = vx(3 * i + 1);
		v(2) = vx(3 * i + 2);
		return v;
	}

	void add3D(int i, const V3D& v, Eigen::VectorX<ScalarType>& vx)
	{
		assert(vx.size() >= 3 * (i + 1));
		vx(3 * i) += v(0);
		vx(3 * i + 1) += v(1);
		vx(3 * i + 2) += v(2);
	}

	void set3D(int i, const V3D& v, Eigen::VectorX<ScalarType>& vx)
	{
		assert(vx.size() >= 3 * (i + 1));
		vx(3 * i    ) = v(0);
		vx(3 * i + 1) = v(1);
		vx(3 * i + 2) = v(2);
	}

	void scale(Eigen::VectorX<ScalarType>& vx, double s)
	{
		for (int i = 0; i < vx.size(); i++)
			vx(i) *= s;
	}

	void stdToEigenVec(const std::vector<double>& vin, Eigen::VectorX<ScalarType>& vout)
	{
		vout = Eigen::VectorX<ScalarType>(vin.size());
		for (int i = 0; i < vin.size(); i++)
			vout(i) = vin[i];
	}

	Eigen::Vector4<ScalarType> get4D(int i, const Eigen::VectorX<ScalarType>& vx)
	{
		assert(vx.size() >= 4 * (i + 1));
		Eigen::Vector4<ScalarType> v;
		v(0) = vx(4 * i);
		v(1) = vx(4 * i + 1);
		v(2) = vx(4 * i + 2);
		v(3) = vx(4 * i + 3);
		return v;
	}

	Eigen::Matrix3<ScalarType> crossProductMatrix(const Eigen::Vector3<ScalarType> &v)
	{
		Eigen::Matrix3<ScalarType> crossProductMatrix;
		crossProductMatrix.setZero();

		crossProductMatrix(0, 1) = -v[2];
		crossProductMatrix(1, 0) = v[2];

		crossProductMatrix(0, 2) = v[1];
		crossProductMatrix(2, 0) = -v[1];

		crossProductMatrix(1, 2) = -v[0];
		crossProductMatrix(2, 1) = v[0];
		return crossProductMatrix;
	}

	double computeTetVolume(Eigen::Vector3<ScalarType> v0, Eigen::Vector3<ScalarType> v1, Eigen::Vector3<ScalarType> v2, Eigen::Vector3<ScalarType> v3)
	{
		V3D x12 = v1 - v0;
		V3D x13 = v2 - v0;
		V3D x14 = v3 - v0;

		double Volume = -1.0 / 6.0 * x14.dot(x13.cross(x12));
		return Volume;
	}


	void computeCenterPosition(const Eigen::VectorX<ScalarType> element_x, Eigen::Vector3<ScalarType>& centerPosition)
	{
		int numVertices = element_x.size() / 3;

		centerPosition.setZero();
		for (int i = 0; i < numVertices; i++)
		{
			centerPosition += element_x.segment<3>(i * 3);
		}
		centerPosition /= double(numVertices);
	}
	void computeCenterPositionJacobian(const Eigen::VectorX<ScalarType> element_x, SpMat& jacobian)
	{
		int numVertices = element_x.size() / 3;

		TripVec jacobian_triplets;

		double averagedGrad = 1.0 / double(numVertices);
		for (int i = 0; i < numVertices; i++)
		{
			for (int v_i = 0; v_i < 3; v_i++)
			{
				jacobian_triplets.emplace_back(v_i, i * 3 + v_i, averagedGrad);
			}
		}

		jacobian = SpMat(3, 3 * numVertices);
		jacobian.setFromTriplets(jacobian_triplets.begin(), jacobian_triplets.end());
		jacobian.makeCompressed();
	}

	/*Eigen::Matrix4f toEigen(const glm::mat4 &mat)
	{
		Eigen::Matrix4f res;
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				res(i, j) = mat[j][i];
			}
		}
		return res;
	}*/

	Eigen::Vector3<ScalarType> projectPointOntoLine(const Eigen::Vector3<ScalarType>& point, const Eigen::Vector3<ScalarType>& linePassthoughPoint, const Eigen::Vector3<ScalarType>& lineVector)
	{
		//https://math.stackexchange.com/a/62718
		Eigen::Vector3<ScalarType> projected_point_onto_line = lineVector * lineVector.transpose() / lineVector.dot(lineVector) * (point - linePassthoughPoint) + linePassthoughPoint;
		return projected_point_onto_line;
	}

	Eigen::Vector3<ScalarType> computePositionAroundAxis(const Eigen::Vector3<ScalarType>& rotate_point, const Eigen::Vector3<ScalarType>& through_p, const Eigen::Vector3<ScalarType>& dir, double sinTheta, double cosTheta)
	{
		/*
https://sites.google.com/site/glennmurray/Home/rotation-matrices-and-formulas/rotation-about-an-arbitrary-axis-in-3-dimensions
6.1 The matrix for rotation about an arbitrary line
*/

		double x = rotate_point[0];
		double y = rotate_point[1];
		double z = rotate_point[2];

		double a = through_p[0];
		double b = through_p[1];
		double c = through_p[2];

		double u = dir[0];
		double v = dir[1];
		double w = dir[2];

		double L = u * u + v * v + w * w;

		double new_x = ((a * (v * v + w * w) - u * (b * v + c * w - u * x - v * y - w * z)) * (1.0 - cosTheta) + L * x * cosTheta + sqrt(L) * (-c * v + b * w - w * y + v * z) * sinTheta) / L;
		double new_y = ((b * (u * u + w * w) - v * (a * u + c * w - u * x - v * y - w * z)) * (1.0 - cosTheta) + L * y * cosTheta + sqrt(L) * (c * u - a * w + w * x - u * z) * sinTheta) / L;
		double new_z = ((c * (u * u + v * v) - w * (a * u + b * v - u * x - v * y - w * z)) * (1.0 - cosTheta) + L * z * cosTheta + sqrt(L) * (-b * u + a * v - v * x + u * y) * sinTheta) / L;

		return Eigen::Vector3<ScalarType>(new_x, new_y, new_z);
	}

	void soDebug(const char* szDebug)
	{
		printf(szDebug);
	}

	void logPrint(const char *format, ...) {
			return;
		static char message[1024];
		va_list vl;

		va_start(vl, format);
		vsprintf(message, format, vl);
		va_end(vl);

		static FILE *fp = fopen("log.txt", "w");
		fprintf(fp, "%s", message);
		fflush(fp);
		soDebug(message);
	}

	void copyTriMesh(const TriMesh* in, TriMesh* out)
	{
		//assert(false);
		out->vertices() = in->vertices();
		out->faces() = in->faces();
		out->faceColors() = in->faceColors();


		//use tiny obj for loading and writing
		out->m_tinyMaterials = in->m_tinyMaterials;
		out->m_attrib = in->m_attrib;
		out->m_shapes = in->m_shapes;

		out->m_meshFileName = in->m_meshFileName;
	}

	Eigen::Matrix3<ScalarType> computeRotationFromTwoCoordinateSystems(Eigen::Vector3<ScalarType> x1, Eigen::Vector3<ScalarType> y1, Eigen::Vector3<ScalarType> z1,
		Eigen::Vector3<ScalarType> x2, Eigen::Vector3<ScalarType> y2, Eigen::Vector3<ScalarType> z2) {

		Eigen::Matrix3<ScalarType> coordinate1;
		coordinate1 << x1, y1, z1;

		Eigen::Matrix3<ScalarType> coordinate2;
		coordinate2 << x2, y2, z2;

		Eigen::Matrix3<ScalarType> R = coordinate2 * coordinate1.inverse();
		return R;
	}

	void findSurfaceTris(const Eigen::MatrixXi& TT, Eigen::MatrixXi& F)
	{
		//TODO: merge with below
		std::map<TriTriplet, int> tri2Tet;
		for (int elemI = 0; elemI < TT.rows(); elemI++) {
			const Eigen::RowVector4i& elemVInd = TT.row(elemI);
			tri2Tet[TriTriplet(elemVInd[0], elemVInd[2], elemVInd[1])] = elemI;
			tri2Tet[TriTriplet(elemVInd[0], elemVInd[3], elemVInd[2])] = elemI;
			tri2Tet[TriTriplet(elemVInd[0], elemVInd[1], elemVInd[3])] = elemI;
			tri2Tet[TriTriplet(elemVInd[1], elemVInd[2], elemVInd[3])] = elemI;
		}

		//TODO: parallelize
		F.conservativeResize(0, 3);
		for (const auto& triI : tri2Tet) {
			const int* triVInd = triI.first.key;
			// find dual triangle with reversed indices:
			auto finder = tri2Tet.find(TriTriplet(triVInd[2], triVInd[1], triVInd[0]));
			if (finder == tri2Tet.end()) {
				finder = tri2Tet.find(TriTriplet(triVInd[1], triVInd[0], triVInd[2]));
				if (finder == tri2Tet.end()) {
					finder = tri2Tet.find(TriTriplet(triVInd[0], triVInd[2], triVInd[1]));
					if (finder == tri2Tet.end()) {
						int oldSize = F.rows();
						F.conservativeResize(oldSize + 1, 3);
						F(oldSize, 0) = triVInd[0];
						F(oldSize, 1) = triVInd[1];
						F(oldSize, 2) = triVInd[2];
					}
				}
			}
		}
	}

	class Key {
	public:
		int a, b, c;
		Key(int a, int b, int c) {
			this->a = a;
			this->b = b;
			this->c = c;
		}

		Key(const Key& a)
		{
			this->a = a.a;
			this->b = a.b;
			this->c = a.c;
		}
		Key& operator=(const Key& a)
		{
			this->a = a.a;
			this->b = a.b;
			this->c = a.c;
		}
		~Key()
		{

		}
	};

	class tbb_compare
	{
	public:
		static int hash(const Key& k) {
			return (k.a << 10 ^ k.b << 5 ^ k.c);
		}

		static bool equal(const Key& k1, const Key& k2) {
			return (k1.a == k2.a && k1.b == k2.b && k1.c == k2.c);
		}
	};

	void findSurfaceTris_parallel(const Eigen::MatrixXi& TT, Eigen::MatrixXi& F)
	{
		//TODO: merge with below
		
		typedef tbb::concurrent_hash_map<Key, int, tbb_compare> triElemMap;
		triElemMap tri2Tet;
		//std::map<TriTriplet, int> tri2Tet;
		for (int elemI = 0; elemI < TT.rows(); elemI++) {
			const Eigen::RowVector4i& elemVInd = TT.row(elemI);

			tri2Tet.insert(std::pair(Key(elemVInd[0], elemVInd[2], elemVInd[1]), elemI));
			tri2Tet.insert(std::pair(Key(elemVInd[0], elemVInd[3], elemVInd[2]), elemI));
			tri2Tet.insert(std::pair(Key(elemVInd[0], elemVInd[1], elemVInd[3]), elemI));
			tri2Tet.insert(std::pair(Key(elemVInd[1], elemVInd[2], elemVInd[3]), elemI));
		}

		//TODO: parallelize
		tbb::enumerable_thread_specific<std::vector<int>> storage(std::vector<int>({}));

		tbb::parallel_for(
			tri2Tet.range(),
			[&](triElemMap::range_type& r) {
				auto& local_face = storage.local();
				for (auto triElemMap_i = r.begin(); triElemMap_i != r.end(); triElemMap_i++)
					//for (size_t i = 0; i < m_elements.size(); i++)
				{
					const auto& triVInd = triElemMap_i->first;

					triElemMap::accessor accessor;
					if (!tri2Tet.find(accessor, Key(triVInd.c, triVInd.b, triVInd.a))) 
						if (!tri2Tet.find(accessor, Key(triVInd.a, triVInd.c, triVInd.b)))
							if (!tri2Tet.find(accessor, Key(triVInd.b, triVInd.a, triVInd.c)))
							{
								local_face.push_back(triVInd.a);
								local_face.push_back(triVInd.b);
								local_face.push_back(triVInd.c);
							}
					
				}

			});

		F.conservativeResize(0, 3);
		for (const auto& local_face : storage) {
			int addNumFaces = local_face.size() / 3;

			int oldSize = F.rows();
			F.conservativeResize(oldSize + addNumFaces, 3);

			const auto local_face_xi = Eigen::Map<const Eigen::MatrixXi>(local_face.data(), 3, addNumFaces);
			F.bottomRows(addNumFaces) = local_face_xi.transpose();
		}
	}


	void getAllTriangleMeshFromTet(const Eigen::MatrixXi& T, Eigen::MatrixXi& F)
	{
		auto isFaceTheSame = [](const Eigen::RowVector3i& a, const Eigen::RowVector3i& b)
		{
			bool situation1 = (a[0] == b[0] && a[1] == b[1] && a[2] == b[2]);
			bool situation2 = (a[0] == b[0] && a[1] == b[2] && a[2] == b[1]);

			bool situation3 = (a[0] == b[1] && a[1] == b[0] && a[2] == b[2]);
			bool situation4 = (a[0] == b[1] && a[1] == b[2] && a[2] == b[0]);

			bool situation5 = (a[0] == b[2] && a[1] == b[0] && a[2] == b[1]);
			bool situation6 = (a[0] == b[2] && a[1] == b[1] && a[2] == b[0]);

			if (situation1 || situation2 || situation3 || situation5 || situation6)
				return true;
			else
				return false;

		};
		for (int tet_i = 0; tet_i < T.rows(); tet_i++)
		{
			Eigen::RowVector4i tetVIndices = T.row(tet_i);
			std::vector<Eigen::RowVector3i> faceIndices = { Eigen::RowVector3i(tetVIndices[0],tetVIndices[1],tetVIndices[2]),
															Eigen::RowVector3i(tetVIndices[0],tetVIndices[1],tetVIndices[3]),
															Eigen::RowVector3i(tetVIndices[0],tetVIndices[2],tetVIndices[3]),
															Eigen::RowVector3i(tetVIndices[1],tetVIndices[2],tetVIndices[3]) };
			bool findFace[4] = { false,false,false,false };
			tbb::parallel_for(
				tbb::blocked_range2d<size_t, size_t>(0, 4, 0, F.rows()),
				[&](const tbb::blocked_range2d<size_t>& r) {

					for (int tet_fi = r.rows().begin(); tet_fi < r.rows().end(); tet_fi++)
					{
						for (int find_fi = r.cols().begin(); find_fi < r.cols().end(); find_fi++)
						{//search for all triangles
							if (isFaceTheSame(faceIndices[tet_fi], F.row(find_fi)))
							{//find, since only one will exist in Face, so we don't need to add lock
								findFace[tet_fi] = true;
							}
						}

					}
				});
			for (int tet_fi = 0; tet_fi < 4; tet_fi++)
			{
				if (!findFace[tet_fi])
				{ //add
					int oldSize = F.rows();
					F.conservativeResize(oldSize + 1, 3);
					F(oldSize, 0) = faceIndices[tet_fi][0];
					F(oldSize, 1) = faceIndices[tet_fi][1];
					F(oldSize, 2) = faceIndices[tet_fi][2];
				}
			}

		}

	}

	bool readTetMesh(const std::string& filePath,
		Eigen::MatrixX<ScalarType>& TV, Eigen::MatrixXi& TT,
		Eigen::MatrixXi& SF, bool findSurface)
	{
		FILE* in = fopen(filePath.c_str(), "r");
		if (!in) {
			return false;
		}

		TV.resize(0, 3);
		TT.resize(0, 4);
		SF.resize(0, 3);

		char buf[BUFSIZ];
		while ((!feof(in)) && fgets(buf, BUFSIZ, in)) {
			if (strncmp("$Nodes", buf, 6) == 0) {
				fgets(buf, BUFSIZ, in);
				int vAmt;
				sscanf(buf, "1 %d", &vAmt);
				TV.resize(vAmt, 3);
				fgets(buf, BUFSIZ, in);
				break;
			}
		}
		assert(TV.rows() > 0);
		int bypass;
		for (int vI = 0; vI < TV.rows(); vI++) {
			fscanf(in, "%d %le %le %le\n", &bypass, &TV(vI, 0), &TV(vI, 1), &TV(vI, 2));
		}

		while ((!feof(in)) && fgets(buf, BUFSIZ, in)) {
			if (strncmp("$Elements", buf, 9) == 0) {
				fgets(buf, BUFSIZ, in);
				int elemAmt;
				sscanf(buf, "1 %d", &elemAmt);
				TT.resize(elemAmt, 4);
				fgets(buf, BUFSIZ, in);
				break;
			}
		}
		assert(TT.rows() > 0);
		for (int elemI = 0; elemI < TT.rows(); elemI++) {
			fscanf(in, "%d %d %d %d %d\n", &bypass,
				&TT(elemI, 0), &TT(elemI, 1), &TT(elemI, 2), &TT(elemI, 3));
		}
		TT.array() -= 1;

		while ((!feof(in)) && fgets(buf, BUFSIZ, in)) {
			if (strncmp("$Surface", buf, 7) == 0) {
				fgets(buf, BUFSIZ, in);
				int elemAmt;
				sscanf(buf, "%d", &elemAmt);
				SF.resize(elemAmt, 3);
				break;
			}
		}
		for (int triI = 0; triI < SF.rows(); triI++) {
			fscanf(in, "%d %d %d\n", &SF(triI, 0), &SF(triI, 1), &SF(triI, 2));
		}
		if (SF.rows() > 0) {
			SF.array() -= 1;
		}
		else if (findSurface) {
			// if no surface triangles information provided, then find
			findSurfaceTris(TT, SF);
		}

		std::cout << "tet mesh loaded with " << TV.rows() << " nodes, " << TT.rows() << " tets, and " << SF.rows() << " surface triangles.\n";

		fclose(in);

		return true;
	}

	V3D crossProd3D(const V3D& u, const V3D& v)
	{
		V3D uxv;
		uxv(0) = u(1)*v(2) - u(2)*v(1);
		uxv(1) = u(2)*v(0) - u(0)*v(2);
		uxv(2) = u(0)*v(1) - u(1)*v(0);
		return uxv;
	}

	void harwell_boeing(
		const SpMat& A,
		int & num_rows,
		std::vector<double> & V,
		std::vector<int> & R,
		std::vector<int> & C)
	{
		num_rows = A.rows();
		int num_cols = A.cols();
		int nnz = A.nonZeros();
		V.resize(nnz);
		R.resize(nnz);
		C.resize(num_cols + 1);

		// Assumes outersize is columns
		assert(A.cols() == A.outerSize());
		int column_pointer = 0;
		int i = 0;
		int k = 0;
		// Iterate over outside
		for (; k<A.outerSize(); ++k)
		{
			C[k] = column_pointer;
			// Iterate over inside
			for (typename Eigen::SparseMatrix<double>::InnerIterator it(A, k); it; ++it)
			{
				V[i] = it.value();
				R[i] = it.row();
				i++;
				// Also increment column pointer
				column_pointer++;
			}
		}
		// by convention C[num_cols] = nnz
		C[k] = column_pointer;
	}


	double computeL1Norm(const Eigen::SparseMatrix<double> &mat)
	{
		Eigen::VectorX<ScalarType> l1s(mat.cols());

//#pragma omp parallel for
		for (int k = 0; k < mat.cols(); ++k)
		{
			double mi = 0.0;
			for (Eigen::SparseMatrix<double>::InnerIterator it(mat, k); it; ++it)
			{
				mi += std::abs(it.value());
			}
			l1s[k] = mi;
		}

		return l1s.maxCoeff();
	}

	bool sparseMatrixRegularizationSolver(const SpMat& A, const SpMat& b, SpMat& x)
	{
		bool solved = false;
		int it_lin = 0;
		const int IT_LIN_MAX = 10;
		SpMat local_A = A;
		while (!solved)
		{
			Eigen::SparseLU<SpMat> solver(local_A);
			//Eigen::PardisoLU<SpMat> solver(local_A);

			solved = true;
			//test if this is a sime-positive definite matrix
			if (solver.info() != Eigen::Success) {
				/// decomposition failed
				printf("A solve failed\n");
				solved = false;
				return false;
			}
			x = solver.solve(-b);

			if (solver.info() != Eigen::Success)
			{
				printf("A solve failed\n");
				solved = false;
				return false;
			}


			//check redsiduum
			SpMat r = local_A * x + b;
			if (r.norm() > 10e-7) {
				solved = false;
				std::cout << " residual too large when computing x " << r.norm() << it_lin << std::endl;
			}
			if (solved || it_lin > IT_LIN_MAX)
				break;

			solved = false;

			double reg = 1e-4*pow(2.0, (double)it_lin);
			//cout << "reg value " << reg << endl;
			SpMat regu_I(local_A.rows(), local_A.cols()); regu_I.setIdentity();
			regu_I *= reg;
			local_A += regu_I;

			it_lin++;
		}
		soutil::logPrint("	Needed %d regularization steps.\n", it_lin);
		return solved;
	}

	void tensorAddTensor(const std::vector<SpMat> &in_tensor1, const std::vector<SpMat> &in_tensor2, std::vector<SpMat>& out_tensor)
	{
		assert(in_tensor1.size() == in_tensor2.size() && in_tensor1[0].rows() == in_tensor2[0].rows() && in_tensor1[0].cols() == in_tensor2[0].cols());

		out_tensor.resize(in_tensor1.size());

//#if defined(_OPENMP)
//#pragma omp parallel for
//#endif
		for (int i = 0; i < in_tensor1.size(); i++)
		{
			out_tensor[i] = in_tensor1[i] + in_tensor2[i];
		}
	}


	void tensorMulMatrix(const std::vector<SpMat> &in_tensor, const SpMat &in_matrix, std::vector<SpMat>& out_tensor)
	{
	
		/*#if defined(_OPENMP)
		#pragma omp parallel for
		#endif
			for (int i = 0; i < in_tensor.size(); i++)
			{
				//assert(in_tensor[i].cols() == in_matrix.cols() && in_tensor[i].rows() == in_matrix.rows());

				out_tensor[i] = in_tensor[i] * in_matrix;
			}*/

		int out_pages = in_matrix.cols();
		out_tensor.resize(out_pages);
		//pre-define the storage
		for (int i = 0; i < out_pages; i++)
		{
			out_tensor[i] = SpMat(in_tensor[0].rows(), in_tensor[0].cols());
		}
		std::vector<TripVec> out_tensor_triplets(out_pages);

		//first we multiply i-th page with i-th row to form a new tensor
		for (int i = 0; i < in_tensor.size(); i++)
		{
			for (int k = 0; k < in_tensor[i].outerSize(); ++k)
				for (SpMat::InnerIterator it(in_tensor[i], k); it; ++it)
				{
					//we multiply this value with the row of in_matrix to form a span value cross pages
					Eigen::VectorX<ScalarType> page_v = it.value() * in_matrix.row(i);
					for (int page = 0; page < out_pages; page++)
						out_tensor_triplets[page].push_back(Tripletd(it.row(), it.col(), page_v[page])); //this equals to +=

				}
		}

		for (int i = 0; i < out_pages; i++)
		{
			out_tensor[i].setFromTriplets(out_tensor_triplets[i].begin(), out_tensor_triplets[i].end());
			out_tensor[i].makeCompressed();
		}
	}


	void tensorTransposeMulVector(const std::vector<SpMat>& in_tensor, const Eigen::VectorX<ScalarType>& in_vec, SpMat& out_matrix)
	{
		out_matrix = SpMat(in_tensor[0].cols(), in_tensor.size());

		TripVec triplets;
		for (int page_k = 0; page_k < in_tensor.size(); page_k++)
		{
			Eigen::VectorX<ScalarType> col_k = in_tensor[page_k].transpose() * in_vec;
			for (int row_i = 0; row_i < col_k.size(); row_i++)
				if (col_k[row_i] != 0.0)
					triplets.emplace_back(row_i, page_k, col_k[row_i]);
		}

		out_matrix.setFromTriplets(triplets.begin(), triplets.end());
		out_matrix.makeCompressed();
	}

	void tensorMulVector1(const std::vector<SpMat> &in_tensor, const Eigen::VectorX<ScalarType> &in_vec, SpMat& out_matrix)
	{
		//Since the tensor is: outputs n - times derivative of hessian(energy) wrt x[i]
		//we multiply first colume of every page with that vector
		/*out_matrix = SpMat(in_tensor[0].rows(), in_tensor[0].cols());

		for (int i = 0; i < in_tensor[0].cols(); i++)
		{
			//remake the matrix to multiply with the vector
			SpMat xi(in_tensor[0].rows(), in_tensor.size()); //make the size
			for (int j = 0; j < in_tensor.size(); j++)
			{
				xi.col(j) = in_tensor[j].col(i);
			}

			out_matrix.col(i) = (xi * in_vec).sparseView();
			//out_matrix.col(i) = (in_tensor[i] * in_vec).sparseView();
		}
		out_matrix.makeCompressed();*/

		out_matrix = SpMat(in_vec.size(), in_tensor.size());
		TripVec triples;
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
					triples.push_back(Tripletd(j, i, col_i[j]));
			}
		}
		out_matrix.setFromTriplets(triples.begin(), triples.end());
		out_matrix.makeCompressed();

		//printf("diff %f\n", (out_matrix - out_matrix1).norm());
	}

	void tensorMulVector2(const std::vector<SpMat>& in_tensor, const Eigen::VectorX<ScalarType>& in_vec, SpMat& out_matrix)
	{
		//Since the tensor is: outputs n - times derivative of hessian(energy) wrt x[i]
		//we multiply first colume of every page with that vector
		out_matrix = SpMat(in_tensor[0].rows(), in_tensor[0].cols());
		out_matrix.setZero();

		std::vector<SpMat> out_matrix_i(in_tensor.size());
#ifdef USE_TBB
		tbb::parallel_for(0, (int)in_tensor.size(), 1, [&](int i)
#else
		for (int i = 0; i < in_tensor.size(); i++)
#endif
		{
			out_matrix_i[i] = in_tensor[i] * in_vec[i];
			//out_matrix += in_tensor[i] * in_vec[i];
		}
#ifdef USE_TBB
		);
#endif
		for (int i = 0; i < in_tensor.size(); i++)
		{
			out_matrix += out_matrix_i[i];
		}

		//out_matrix.makeCompressed();

		//printf("diff %f\n", (out_matrix - out_matrix1).norm());
	}

	void vectorMulTensor(const Eigen::VectorX<ScalarType>& in_vec, const std::vector<SpMat>& in_tensor, SpMat& out_matrix)
	{
		/*out_matrix = SpMat(in_tensor.size(), in_tensor[0].cols());
		TripVec triples;

		for (int i = 0; i < in_tensor.size(); i++)
		{
			Eigen::VectorX<ScalarType> row_i = in_vec.transpose()*in_tensor[i];
			for (int j = 0; j < row_i.size(); j++)
			{
				if (row_i[j] != 0.0)
					triples.push_back(Tripletd(i, j, row_i[j]));
			}
		}
		out_matrix.setFromTriplets(triples.begin(), triples.end());
		out_matrix.makeCompressed();*/


		out_matrix = SpMat(in_tensor[0].rows(), in_tensor[0].cols()); out_matrix.setZero();
		for (int i = 0; i < in_tensor.size(); i++)
		{
			out_matrix += in_vec[i] * in_tensor[i];
		}

	}

	void vectorTransposeMulTensor(const Eigen::VectorX<ScalarType>& in_vec, const std::vector<SpMat>& in_tensor, SpMat& out_matrix)
	{//this can apply to something like (dE/dx)^T * d^2x/dq^2
		out_matrix = SpMat(in_tensor.size(), in_tensor[0].cols());//page * cols
		out_matrix.setZero();

		Eigen::RowVectorX<ScalarType> in_vec_transpose = in_vec.transpose();

		TripVec triplets;
		for (int k = 0; k < in_tensor.size(); k++)
		{
			//out_matrix.row(i) = in_vec_transpose * in_tensor[i]; //each row
			Eigen::RowVectorX<ScalarType> row_k = in_vec_transpose * in_tensor[k];
			for (int j = 0; j < row_k.size(); j++)
			{
				if (row_k[j] != 0.0)
					triplets.emplace_back(k, j, row_k[j]);
			}
		}
		out_matrix.setFromTriplets(triplets.begin(), triplets.end());
		out_matrix.makeCompressed();


		//tbb::enumerable_thread_specific<TripVec> storage;

		//tbb::parallel_for(
		//	tbb::blocked_range<size_t>(size_t(0), in_tensor.size()),
		//	[&](const tbb::blocked_range<size_t>& r)
		//	{
		//		auto& local_hess_triplets = storage.local();

		//		for (size_t k = r.begin(); k < r.end(); k++)
		//		{
		//			//out_matrix.row(i) = in_vec_transpose * in_tensor[i]; //each row
		//			Eigen::RowVectorX<ScalarType> row_k = in_vec_transpose * in_tensor[k];
		//			for (int j = 0; j < row_k.size(); j++)
		//			{
		//				if (row_k[j] != 0.0)
		//					local_hess_triplets.emplace_back(k, j, row_k[j]);
		//			}
		//		}
		//	}
		//);

		//for (const auto& local_hess_triplets : storage) {
		//	Eigen::SparseMatrix<double> local_hess(in_tensor.size(), in_tensor[0].cols());
		//	local_hess.setFromTriplets(
		//		local_hess_triplets.begin(), local_hess_triplets.end());
		//	out_matrix += local_hess;
		//}
	}

	void vectorTransposeMulTensor2(const Eigen::VectorX<ScalarType>& in_vec, const std::vector<SpMat>& in_tensor, SpMat& out_matrix)
	{
		//this can apply to something like (dE/dx)^T * d^2x/dq^2
		out_matrix = SpMat(in_tensor[0].cols(), in_tensor.size());//cols * page
		out_matrix.setZero();

		Eigen::RowVectorX<ScalarType> in_vec_transpose = in_vec.transpose();

		TripVec triplets;
		for (int k = 0; k < in_tensor.size(); k++)
		{
			Eigen::RowVectorX<ScalarType> col_k = in_vec_transpose * in_tensor[k];//dk is a col
			for (int j = 0; j < col_k.size(); j++)
			{
				if (col_k[j] != 0.0)
					triplets.emplace_back(j, k, col_k[j]);
			}
		}
		out_matrix.setFromTriplets(triplets.begin(), triplets.end());
		out_matrix.makeCompressed();



		/*for (int i = 0; i < in_vec.size(); i++)
		{
			Eigen::MatrixX<ScalarType> page_slice(in_tensor[0].cols(), in_tensor.size());
			for (int j = 0; j < in_tensor.size(); j++)
			{
				page_slice.row(j) = in_tensor[j].row(i);
			}
			out_matrix += in_vec[i] * page_slice;
		}*/

		/*for (int page_i = 0; page_i < in_tensor.size(); page_i++)
		{
			Eigen::VectorX<ScalarType> col_i = (in_vec.transpose() * in_tensor[page_i]).transpose();

			out_matrix.col(page_i) = col_i;
		}*/
	}

	void matrixMulTensor(const SpMat& in_matrix, const std::vector<SpMat>& in_tensor, std::vector<SpMat>& out_tensor)
	{
		//the in_matrix has transpose outside of this function is assumed
		//this function has some problems, need to check it again 
		int out_pages = in_matrix.rows();
		out_tensor.resize(out_pages);

		for (int i = 0; i < out_pages; i++)
		{
			out_tensor[i] = SpMat(in_tensor[0].rows(), in_tensor[0].cols());
			out_tensor[i].setZero();
		}
		std::vector<TripVec> out_tensor_triplets(out_pages);

		//first we multiply i-th page with i-th row to form a new tensor
		for (int i = 0; i < out_pages; i++)
		{
			Eigen::VectorX<ScalarType> row_i = in_matrix.row(i);
			for (int l = 0; l < row_i.size(); l++)
			{
				out_tensor[i] += row_i[l] * in_tensor[l];
			}
		}

	}

	void tensorTranspose_inplace(std::vector<SpMat>& transpose_tensor)
	{
		//this is a tensor transpose which is used when (A^t*T*B)^t = B^t * T^t * A, where A and B are vectors
		//The size of 3 dimension should be the same
		assert(transpose_tensor.size() == transpose_tensor[0].rows() && transpose_tensor[0].rows() == transpose_tensor[0].cols());

		std::vector<SpMat> temp_tensor(transpose_tensor.size());
		std::vector<TripVec> temp_triplets(transpose_tensor.size());
		//firstly, transpose in each page
//#if defined(_OPENMP)
//#pragma omp parallel for
//#endif
		for (int i = 0; i < transpose_tensor.size(); i++)
		{
			temp_tensor[i] = SpMat(transpose_tensor[0].rows(), transpose_tensor[0].cols());
			transpose_tensor[i] = transpose_tensor[i].transpose();
		}
		//then, move each k-th row to k-page,i-th row
		for (int i = 0; i < transpose_tensor.size(); i++)
		{
			//for i-th page, we move every row to other pages
			for (int k = 0; k < transpose_tensor[i].outerSize(); ++k)
			{
				for (SpMat::InnerIterator it(transpose_tensor[i], k); it; ++it)
				{
					//dGdp_triplets.push_back(Tripletd(it.row(), it.col(), it.value()));
					temp_triplets[it.row()].push_back(Tripletd(i, it.col(), it.value()));
				}
			}
		}
		//finally, transpose in each page
//#if defined(_OPENMP)
//#pragma omp parallel for
//#endif
		for (int i = 0; i < temp_tensor.size(); i++)
		{
			temp_tensor[i].setFromTriplets(temp_triplets[i].begin(), temp_triplets[i].end());
			temp_tensor[i].makeCompressed();

			transpose_tensor[i] = temp_tensor[i].transpose();
		}
	}

	std::vector<SpMat> tensorTranspose(const std::vector<SpMat>& transpose_tensor)
	{
		//this is a tensor transpose which is used when (A^t*T*B)^t = B^t * T^t * A, where A and B are vectors
		//The size of 3 dimension should be the same
		assert(transpose_tensor.size() == transpose_tensor[0].rows() && transpose_tensor[0].rows() == transpose_tensor[0].cols());

		std::vector<SpMat> transposed_tensor(transpose_tensor.size());
		std::vector<SpMat> temp_tensor(transpose_tensor.size()); std::vector<TripVec> temp_triplets(transpose_tensor.size());
		//firstly, transpose in each page
//#if defined(_OPENMP)
//#pragma omp parallel for
//#endif
		for (int i = 0; i < transpose_tensor.size(); i++)
		{
			temp_tensor[i] = SpMat(transpose_tensor[0].rows(), transpose_tensor[0].cols());
			transposed_tensor[i] = transpose_tensor[i].transpose();
		}
		//then, move each k-th row to k-page,i-th row
		for (int i = 0; i < transposed_tensor.size(); i++)
		{
			//for i-th page, we move every row to other pages
			for (int k = 0; k < transposed_tensor[i].outerSize(); ++k)
			{
				for (SpMat::InnerIterator it(transposed_tensor[i], k); it; ++it)
				{
					//dGdp_triplets.push_back(Tripletd(it.row(), it.col(), it.value()));
					temp_triplets[it.row()].push_back(Tripletd(i, it.col(), it.value()));
				}
			}
		}
		//finally, transpose in each page
//#if defined(_OPENMP)
//#pragma omp parallel for
//#endif
		for (int i = 0; i < temp_tensor.size(); i++)
		{
			temp_tensor[i].setFromTriplets(temp_triplets[i].begin(), temp_triplets[i].end());
			temp_tensor[i].makeCompressed();

			transposed_tensor[i] = temp_tensor[i].transpose();
		}

		return transposed_tensor;
	}
	
	std::vector<SpMat> getTensorBlock(const std::vector<SpMat> &inputTensor, int startPage, int startRow, int startCol, int numPages, int numRows, int numCols)
	{
		assert(inputTensor.size() >= startPage + numPages);
		assert(inputTensor[0].rows() >= startRow + numRows);
		assert(inputTensor[0].cols() >= startCol + numCols);
		std::vector<SpMat> outputTensor(numPages);

//#if defined(_OPENMP)
//#pragma omp parallel for
//#endif
		for (int i = 0; i < numPages; i++)
		{
			outputTensor[i] = inputTensor[startPage + i].block(startRow, startCol, numRows, numCols);
		}
		return outputTensor;
	}


	double doubleContraction(const SpMat A, const SpMat &B)
	{
		assert(A.rows() == B.rows());
		assert(A.cols() == B.cols());

		double sum = 0.0;
		for (int k = 0; k < A.outerSize(); ++k)
			for (SpMat::InnerIterator it(A, k); it; ++it)
			{
				double B_value = B.coeff(it.row(), it.col());
				if (B_value != 0.0)
				{
					sum += B_value * it.value();
				}
			}
		return sum;
	}

	void tensorMulMatrix(const std::vector<Eigen::MatrixX<ScalarType>>& in_tensor, const Eigen::MatrixX<ScalarType>& in_matrix, std::vector<Eigen::MatrixX<ScalarType>>& out_tensor)
	{

		int out_pages = in_matrix.cols();
		out_tensor.resize(out_pages);
		//pre-define the storage
		for (int i = 0; i < out_pages; i++)
		{
			out_tensor[i] = Eigen::MatrixX<ScalarType>(in_tensor[0].rows(), in_tensor[0].cols());
			out_tensor[i].setZero();
		}
		std::vector<TripVec> out_tensor_triplets(out_pages);

		//first we multiply i-th page with i-th row to form a new tensor
		for (int i = 0; i < in_tensor.size(); i++)
		{
			//for (int k = 0; k < in_tensor[i].outerSize(); ++k)
			//	for (SpMat::InnerIterator it(in_tensor[i], k); it; ++it)
			for (int rowi = 0; rowi < in_tensor[i].rows(); rowi++)
				for (int coli = 0; coli < in_tensor[i].cols(); coli++)
				{
					//we multiply this value with the row of in_matrix to form a span value cross pages
					Eigen::VectorX<ScalarType> page_v = in_tensor[i](rowi,coli) * in_matrix.row(i);
					for (int page = 0; page < out_pages; page++)
						out_tensor[page](rowi, coli) += page_v[page];

				}
		}
	}


	void vectorMulTensor(const Eigen::VectorX<ScalarType>& in_vec, const std::vector<Eigen::MatrixX<ScalarType>>& in_tensor, Eigen::MatrixX<ScalarType>& out_matrix)
	{
		out_matrix = Eigen::MatrixX<ScalarType>(in_tensor[0].rows(), in_tensor[0].cols());
		out_matrix.setZero();
		for (int i = 0; i < in_tensor.size(); i++)
		{
			out_matrix += in_vec[i] * in_tensor[i];
		}

	}

	//template<int vn, int tp, int tr, int tc>
	//void vectorTransposeMulTensor(const Eigen::Vector<double, vn>& in_vec, const std::vector<Eigen::Matrix<double, tr, tc>>& in_tensor, Eigen::Matrix<double, tp, tc>& out_matrix)
	//{
	//	assert(tp == in_tensor.size());
	//	//for symmetric tensor, this will give the same result for vectorTransposeMulTensor2
	//	//out_matrix = Eigen::MatrixX<ScalarType>(tp, tc);//page * cols
	//	out_matrix.setZero();

	//	Eigen::RowVectorX<ScalarType> in_vec_transpose = in_vec.transpose();

	//	for (int i = 0; i < in_tensor.size(); i++)
	//	{
	//		out_matrix.row(i) = in_vec_transpose * in_tensor[i]; //each row
	//	}

	//}

	//void vectorTransposeMulTensor2(const Eigen::VectorX<ScalarType>& in_vec, const std::vector<Eigen::MatrixX<ScalarType>>& in_tensor, Eigen::MatrixX<ScalarType>& out_matrix)
	//{
	//	//this can apply to something like (dE/dx)^T * d^2x/dq^2
	//	out_matrix = Eigen::MatrixX<ScalarType>(in_tensor[0].cols(), in_tensor.size());//cols * page
	//	out_matrix.setZero();

	//	for (int i = 0; i < in_vec.size(); i++)
	//	{
	//		Eigen::MatrixX<ScalarType> page_slice(in_tensor[0].cols(), in_tensor.size());
	//		for (int j = 0; j < in_tensor.size(); j++)
	//		{
	//			page_slice.row(j) = in_tensor[j].row(i);
	//		}
	//		out_matrix += in_vec[i] * page_slice;
	//	}
	//}

	void vectorTransposeMulTensorxd(const Eigen::VectorX<ScalarType>& in_vec, const std::vector<Eigen::MatrixX<ScalarType>>& in_tensor, Eigen::MatrixX<ScalarType>& out_matrix)
	{
		out_matrix.setZero(in_tensor.size(), in_tensor[0].cols());

		Eigen::RowVectorX<ScalarType> in_vec_transpose = in_vec.transpose();

		for (int i = 0; i < in_tensor.size(); i++)
		{
			out_matrix.row(i) = in_vec_transpose * in_tensor[i]; //each row
		}
	}

	void matrixMulTensor(const Eigen::MatrixX<ScalarType>& in_matrix, const std::vector<Eigen::MatrixX<ScalarType>>& in_tensor, std::vector<Eigen::MatrixX<ScalarType>>& out_tensor)
	{
		//the in_matrix has transpose outside of this function is assumed
		//this function has some problems, need to check it again 
		int out_pages = in_matrix.rows();
		out_tensor.resize(out_pages);

		for (int i = 0; i < out_pages; i++)
		{
			out_tensor[i] = Eigen::MatrixX<ScalarType>(in_tensor[0].rows(), in_tensor[0].cols());
			out_tensor[i].setZero();
		}

		//first we multiply i-th page with i-th row to form a new tensor
		for (int i = 0; i < out_pages; i++)
		{
			Eigen::VectorX<ScalarType> row_i = in_matrix.row(i);
			for (int l = 0; l < row_i.size(); l++)
			{
				out_tensor[i] += row_i[l] * in_tensor[l];
			}
		}

	}

	void matrixMulTensor(const Eigen::MatrixX<ScalarType>& in_matrix, const std::vector<Eigen::MatrixX<ScalarType>>& in_tensor, Eigen::VectorX<ScalarType>& out_vector)
	{
		//compute da/dB * dB/dvc -> da/vc is a vector
		//sum over B
		out_vector = Eigen::VectorX<ScalarType>(in_tensor.size());
		out_vector.setZero();
		for (int page_i = 0; page_i < 3; page_i++)
		{
			for (int Qi = 0; Qi < in_matrix.rows(); Qi++)
			{
				for (int Qj = 0; Qj < in_matrix.cols(); Qj++)
				{
					out_vector[page_i] += in_matrix(Qi, Qj) * in_tensor[page_i](Qi, Qj);
				}
			}
		}
	}

	void matrixTransposeMulTensorMulMatrix(const Eigen::MatrixX<ScalarType>& in_matrix, const std::vector<Eigen::MatrixX<ScalarType>>& in_tensor, std::vector<Eigen::MatrixX<ScalarType>>& out_tensor)
	{
		//A^T * B * A

		std::vector<Eigen::MatrixX<ScalarType>> temp(in_tensor.size());
		for (int i = 0; i < in_tensor.size(); i++)
		{
			temp[i] = in_tensor[i] * in_matrix;
		}


		out_tensor.resize(temp[0].cols());
		for (int col_i = 0; col_i < temp[0].cols(); col_i++)
		{
			Eigen::MatrixX<ScalarType> colMatrix(temp[0].rows(), temp.size());
			for (int i = 0; i < temp.size(); i++)
			{
				colMatrix.col(i) = temp[i].col(col_i);
			}
			out_tensor[col_i] = colMatrix * in_matrix;
		}
	}

	void tensorTranspose_inplace(std::vector<Eigen::MatrixX<ScalarType>>& transpose_tensor)
	{
		//this is a tensor transpose which is used when (A^t*T*B)^t = B^t * T^t * A, where A and B are vectors
		//The size of 3 dimension should be the same
		assert(transpose_tensor.size() == transpose_tensor[0].rows() && transpose_tensor[0].rows() == transpose_tensor[0].cols());

		std::vector<Eigen::MatrixX<ScalarType>> temp_tensor(transpose_tensor.size());
		//firstly, transpose in each page
//#if defined(_OPENMP)
//#pragma omp parallel for
//#endif
		for (int i = 0; i < transpose_tensor.size(); i++)
		{
			temp_tensor[i] = Eigen::MatrixX<ScalarType>(transpose_tensor[0].rows(), transpose_tensor[0].cols());
			temp_tensor[i].setZero();
			transpose_tensor[i] = transpose_tensor[i].transpose();
		}
		//then, move each k-th row to k-page,i-th row
		for (int i = 0; i < transpose_tensor.size(); i++)
		{
			//for i-th page, we move every row to other pages
			for (int rowi = 0; rowi < transpose_tensor[i].rows(); rowi++)
			{
				for (int coli = 0; coli < transpose_tensor[i].cols(); coli++)
				{
					temp_tensor[rowi](i, coli) += transpose_tensor[i](rowi, coli);
				}
			}
		}
		//finally, transpose in each page
//#if defined(_OPENMP)
//#pragma omp parallel for
//#endif
		for (int i = 0; i < temp_tensor.size(); i++)
		{
			transpose_tensor[i] = temp_tensor[i].transpose();
		}
	}

	std::vector<Eigen::MatrixX<ScalarType>> tensorTranspose(const std::vector<Eigen::MatrixX<ScalarType>>& transpose_tensor)
	{
		//this is a tensor transpose which is used when (A^t*T*B)^t = B^t * T^t * A, where A and B are vectors
		//The size of 3 dimension should be the same
		assert(transpose_tensor.size() == transpose_tensor[0].rows() && transpose_tensor[0].rows() == transpose_tensor[0].cols());

		std::vector<Eigen::MatrixX<ScalarType>> transposed_tensor(transpose_tensor.size());
		std::vector<Eigen::MatrixX<ScalarType>> temp_tensor(transpose_tensor.size());
		//firstly, transpose in each page
//#if defined(_OPENMP)
//#pragma omp parallel for
//#endif
		for (int i = 0; i < transpose_tensor.size(); i++)
		{
			temp_tensor[i] = Eigen::MatrixX<ScalarType>(transpose_tensor[0].rows(), transpose_tensor[0].cols());
			temp_tensor[i].setZero();
			transposed_tensor[i] = transpose_tensor[i].transpose();
		}
		//then, move each k-th row to k-page,i-th row
		for (int i = 0; i < transposed_tensor.size(); i++)
		{
			//for i-th page, we move every row to other pages
			for (int rowi = 0; rowi < transpose_tensor[i].rows(); rowi++)
			{
				for (int coli = 0; coli < transpose_tensor[i].cols(); coli++)
				{
					temp_tensor[rowi](i, coli) += transpose_tensor[i](rowi, coli);
				}
			}
		}
		//finally, transpose in each page
//#if defined(_OPENMP)
//#pragma omp parallel for
//#endif
		for (int i = 0; i < temp_tensor.size(); i++)
		{
			transposed_tensor[i] = temp_tensor[i].transpose();
		}

		return transposed_tensor;
	}

	std::vector<Eigen::MatrixX<ScalarType>> getTensorBlock(const std::vector<Eigen::MatrixX<ScalarType>>& inputTensor, int startPage, int startRow, int startCol, int numPages, int numRows, int numCols)
	{
		assert(inputTensor.size() >= startPage + numPages);
		assert(inputTensor[0].rows() >= startRow + numRows);
		assert(inputTensor[0].cols() >= startCol + numCols);
		std::vector<Eigen::MatrixX<ScalarType>> outputTensor(numPages);

//#if defined(_OPENMP)
//#pragma omp parallel for
//#endif
		for (int i = 0; i < numPages; i++)
		{
			outputTensor[i] = inputTensor[startPage + i].block(startRow, startCol, numRows, numCols);
		}
		return outputTensor;
	}

	/*void setTensorBlock(const std::vector<Eigen::MatrixX<ScalarType>>& inputTensor, std::vector<Eigen::MatrixX<ScalarType>>& outputTensor, int output_startPage, int output_startRow, int output_startCol)
	{//this function will set all of inputTensor to the outputTensor
		int numPages = inputTensor.size();
		int numRows = inputTensor[0].rows();
		int numCols = inputTensor[0].cols();

		assert(outputTensor.size() >= output_startPage + numPages);
		assert(outputTensor[0].rows() >= output_startRow + numRows);
		assert(outputTensor[0].cols() >= output_startCol + numCols);

#if defined(_OPENMP)
#pragma omp parallel for
#endif
		for (int i = 0; i < numPages; i++)
		{
			outputTensor[output_startPage + i].block(output_startRow, output_startCol, numRows, numCols) = inputTensor[i];
		}
	}*/

	/*void setTensorBlock(const std::vector<Eigen::MatrixX<ScalarType>>& inputTensor, int input_startPage, int input_startRow, int input_startCol, int input_numPages, int input_numRows, int input_numCols,
		std::vector<Eigen::MatrixX<ScalarType>>& outputTensor, int output_startPage, int output_startRow, int output_startCol)
	{//this function will select part of the InputTensor to the outputTensor

		assert(outputTensor.size() >= output_startPage + input_numPages);
		assert(outputTensor[0].rows() >= output_startRow + input_numRows);
		assert(outputTensor[0].cols() >= output_startCol + input_numCols);

#if defined(_OPENMP)
#pragma omp parallel for
#endif
		for (int i = 0; i < input_numPages; i++)
		{
			outputTensor[output_startPage + i].block(output_startRow, output_startCol, input_numRows, input_numCols) =
				inputTensor[input_startPage + i].block(input_startRow, input_startCol, input_numRows, input_numCols);
		}
	}*/


	void test_tensor_operation()
	{
		//////////vec tensor transpose///////////////////////
		{
			std::vector<SpMat> test_tensor(10);
			for (int i = 0; i < 10; i++)
			{
				test_tensor[i] = SpMat(10, 10);
				Eigen::MatrixX<ScalarType> random(10, 10); random.setRandom();
				test_tensor[i] = random.sparseView();
			}
			Eigen::VectorX<ScalarType> A(10); A.setRandom();
			Eigen::VectorX<ScalarType> B(10); B.setRandom();
			SpMat inter_matrix;
			soutil::vectorMulTensor(A, test_tensor, inter_matrix);
			Eigen::VectorX<ScalarType>  final1 = inter_matrix * B;
			/////////////////////////////////////////////////
			soutil::tensorTranspose_inplace(test_tensor);
			soutil::vectorMulTensor(B, test_tensor, inter_matrix);
			Eigen::VectorX<ScalarType>  final2 = inter_matrix * A;
			printf("final1 - final2 = %f\n",(final1 - final2).norm());
		}

		//////////matrix mul tensor///////////////////////
		{
			std::vector<SpMat> test_tensor(10);
			for (int i = 0; i < 10; i++)
			{
				test_tensor[i] = SpMat(10, 10);
				Eigen::MatrixX<ScalarType> random; random.setRandom();
				test_tensor[i] = random.sparseView();
			}

			SpMat A(10, 10);
			{
				Eigen::MatrixX<ScalarType> random(10, 10); random.setRandom();
				A = random.sparseView();
			}
			Eigen::VectorX<ScalarType> b(10); b.setRandom();


			std::vector<SpMat> final_tensor1;
			soutil::matrixMulTensor(A.transpose(), test_tensor, final_tensor1);
			SpMat final1;
			soutil::tensorMulVector1(final_tensor1, b, final1);


			SpMat final_matrix2;
			soutil::tensorTranspose_inplace(test_tensor);
			soutil::vectorMulTensor(b, test_tensor, final_matrix2);
			SpMat final2 = final_matrix2 * A.transpose();

			printf("final1 - final2 = %f\n", (final1 - final2).norm());
		}
	}

	double doubleContraction(const Eigen::MatrixX<ScalarType> A, const Eigen::MatrixX<ScalarType> &B)
	{
		assert(A.rows() == B.rows());
		assert(A.cols() == B.cols());

		tbb::enumerable_thread_specific<double> storage(0);

		tbb::parallel_for(
				tbb::blocked_range2d<size_t, size_t>(0, A.rows(), 0, A.cols()),
				[&](const tbb::blocked_range2d<size_t>& r) {
					auto& local_sum = storage.local();
					for (int row_i = r.rows().begin(); row_i < r.rows().end(); row_i++)
					{
						for (int col_i = r.cols().begin(); col_i < r.cols().end(); col_i++)
						{
							local_sum += A(row_i, col_i) * B(row_i, col_i);
						}
					}
		});
		double sum = 0.0;
		for (const auto &local_sum : storage)
		{
			sum += local_sum;
		}
		return sum;
	}


	Eigen::MatrixX<ScalarType> tensorVectorization(const std::vector<Eigen::MatrixX<ScalarType>> &A)
	{
		Eigen::MatrixX<ScalarType> out(A[0].size(), A.size());
		tbb::parallel_for(
			tbb::blocked_range<size_t>(0, A.size()),
			[&](const tbb::blocked_range<size_t> &r) {
				for (int page_i = r.begin(); page_i < r.end(); page_i++)
				{
					out.col(page_i) = Eigen::Map<const Eigen::VectorX<ScalarType>>(A[page_i].data(), A[page_i].size());
				}
			});

		return out;
	}

	// We use these bounds because for example 1 + x^2 = 1 for x < sqrt(epsilon).
	static const double taylor_0_bound = std::numeric_limits<double>::epsilon();
	static const double taylor_2_bound = sqrt(taylor_0_bound);
	static const double taylor_n_bound = sqrt(taylor_2_bound);

	double sinc(const double& x)
	{
		if (abs(x) >= taylor_n_bound) {
			return sin(x) / x;
		}

		// approximation by taylor series in x at 0 up to order 1
		double result = 1;

		if (abs(x) >= taylor_0_bound) {
			double x2 = x * x;

			// approximation by taylor series in x at 0 up to order 3
			result -= x2 / 6.0;

			if (abs(x) >= taylor_2_bound) {
				// approximation by taylor series in x at 0 up to order 5
				result += (x2 * x2) / 120.0;
			}
		}

		return result;
	}
	/// Compute the L2 norm with abs
	double absL2norm(const Eigen::Vector3<ScalarType>& x)
	{
		// Do an explicit abs to avoid possible problems with intervals
		Eigen::Vector3<ScalarType> absx(x.size());
		for (int i = 0; i < x.size(); i++) {
			absx(i) = abs(x(i));
		}
		return sqrt(absx.dot(absx));
	}

	/// Compute sinc(||x||)
	double sinc_normx(const Eigen::Vector3<ScalarType>& x)
	{
		return sinc(absL2norm(x));
	}

	inline double dsinc_over_x(double x)
	{
		static const double eps = 1e-4;

		double x2 = x * x;
		if (abs(x) > eps) {
			return (x * cos(x) - sin(x)) / (x2 * x);
		}

		// approximation by taylor series in x at 0 up to order 5
		return x2 * (-x2 / 840.0 + 1.0 / 30.0) - 1.0 / 3.0;
	}


	// Compute gradient of sinc(||x||)
	Eigen::Vector3<ScalarType> sinc_normx_grad(const Eigen::Vector3<ScalarType>& x)
	{
		return dsinc_over_x(x.norm()) * x;
	}

	inline double ddsinc_over_x2_minus_dsinc_over_x3(double x)
	{
		static const double eps = 0.1;

		double x2 = x * x;
		double x4 = x2 * x2;
		if (abs(x) > eps) {
			return ((3 - x2) * sin(x) - 3 * x * cos(x)) / (x4 * x);
		}

		// approximation by taylor series in x at 0 up to order 5
		return x4 / 7560.0 - x2 / 210.0 + 1.0 / 15.0;
	}

	// Compute hessian of sinc(||x||)
	Eigen::Matrix3<ScalarType> sinc_normx_hess(const Eigen::Vector3<ScalarType>& x)
	{
		double normx = x.norm();
		return ddsinc_over_x2_minus_dsinc_over_x3(normx) * x * x.transpose()
			+ dsinc_over_x(normx) * Eigen::Matrix3<ScalarType>::Identity(x.size(), x.size());
	}

	Eigen::Matrix3<ScalarType> Hat(const Eigen::Vector3<ScalarType>& x)
	{
		Eigen::Matrix3<ScalarType> M;
		double zero(0);
		M.row(0) << zero, -x.z(), x.y();
		M.row(1) << x.z(), zero, -x.x();
		M.row(2) << -x.y(), x.x(), zero;
		return M;
	}

	void Hat_jacobian(const Eigen::Vector3<ScalarType>& x, std::vector<Eigen::Matrix3<ScalarType>>& dhat_dx)
	{
		dhat_dx.resize(3);
		double zero(0);
		double one(1);
		dhat_dx[0].row(0) << zero, zero, zero;
		dhat_dx[0].row(1) << zero, zero, -one;
		dhat_dx[0].row(2) << zero, one, zero;

		dhat_dx[1].row(0) << zero, zero, one;
		dhat_dx[1].row(1) << zero, zero, zero;
		dhat_dx[1].row(2) << -one, zero, zero;

		dhat_dx[2].row(0) << zero, -one, zero;
		dhat_dx[2].row(1) << one, zero, zero;
		dhat_dx[2].row(2) << zero, zero, zero;

		//since this jacobian is a constant matrix so we don't need to build it hessian
	}

	Eigen::Matrix3<ScalarType> construct_rotation_matrix(const Eigen::Vector3<ScalarType>& r)
	{
		double sinc_angle = sinc_normx(r);
		double sinc_half_angle = sinc_normx((r / 2.0).eval());
		Eigen::Matrix3<ScalarType> K = Hat(r);
		Eigen::Matrix3<ScalarType> K2 = K * K;
		Eigen::Matrix3<ScalarType> R = sinc_angle * K + 0.5 * sinc_half_angle * sinc_half_angle * K2;
		R.diagonal().array() += 1.0;
		
		return R;
	}

	void construct_rotation_matrix_jacobian(const Eigen::Vector3<ScalarType>& r, std::vector<Eigen::Matrix3<ScalarType>>& drotation_dr)
	{
		double sinc_angle = sinc_normx(r);
		double sinc_half_angle = sinc_normx((r / 2.0).eval());
		Eigen::Matrix3<ScalarType> K = Hat(r);
		Eigen::Matrix3<ScalarType> K2 = K * K;

		Eigen::Vector3<ScalarType> sinc_angle_grad = sinc_normx_grad(r);

		Eigen::Vector3<ScalarType> sinc_half_angle_grad = sinc_normx_grad((r/2.0).eval());

		std::vector<Eigen::Matrix3<ScalarType>> dK_dx;
		Hat_jacobian(r, dK_dx);

		drotation_dr.resize(3);
		for (int i = 0; i < 3; i++)
		{
			drotation_dr[i] = sinc_angle_grad[i] * K + sinc_angle * dK_dx[i]
				+ 0.5 * sinc_half_angle * sinc_half_angle_grad[i] * K2 + 0.5 * sinc_half_angle * sinc_half_angle * (dK_dx[i] * K + K * dK_dx[i]);
		}

	}

	void construct_rotation_matrix_hessians(const Eigen::Vector3<ScalarType>& r, std::vector<std::vector<Eigen::Matrix3<ScalarType>>>& d2rotation_dr2)
	{
		double sinc_angle = sinc_normx(r);
		double sinc_half_angle = sinc_normx((r / 2.0).eval());
		Eigen::Matrix3<ScalarType> K = Hat(r);
		Eigen::Matrix3<ScalarType> K2 = K * K;

		Eigen::Vector3<ScalarType> sinc_angle_grad = sinc_normx_grad(r);
		Eigen::Matrix3<ScalarType> sinc_angle_hess = sinc_normx_hess(r);

		Eigen::Vector3<ScalarType> sinc_half_angle_grad = sinc_normx_grad((r / 2.0).eval());
		Eigen::Matrix3<ScalarType> sinc_half_angle_hess = sinc_normx_hess((r / 2.0).eval());

		std::vector<Eigen::Matrix3<ScalarType>> dK_dx;
		Hat_jacobian(r, dK_dx);
		//the hessian of the hat matrix is zero

		d2rotation_dr2.resize(3);
		for (int group_i = 0; group_i < 3; group_i++)
		{
			d2rotation_dr2[group_i].resize(3);
			for (int page_j = 0; page_j < 3; page_j++)
			{
				// dQ / dtheta_j dtheta_i
				//which means we first take j derivative then i derivative
				d2rotation_dr2[group_i][page_j] =
					sinc_angle_hess(page_j, group_i) * K + sinc_angle_grad[page_j] * dK_dx[group_i] +  //first term derivative in the gradient
					sinc_angle_grad[group_i] * dK_dx[page_j] + //0, second term derivative in the gradient
					0.25 * sinc_half_angle_grad[group_i] * sinc_half_angle_grad[page_j] * K2 + 0.5 * sinc_half_angle * (0.5 * sinc_half_angle_hess(page_j, group_i) * K2 + sinc_half_angle_grad[page_j] * (dK_dx[group_i] * K + K * dK_dx[group_i])) +   //third term derivative in the gradient
					0.5 * sinc_half_angle * sinc_half_angle_grad[group_i] * (dK_dx[page_j] * K + K * dK_dx[page_j]) + 0.5 * sinc_half_angle * sinc_half_angle * (dK_dx[page_j] * dK_dx[group_i] + dK_dx[group_i] * dK_dx[page_j]);	//0, fourth term derivative in the gradient
			}
		}
	}

	template <typename Derived, typename T>
	Eigen::Quaternion<T> construct_quaternion(const Eigen::MatrixBase<Derived>& r)
	{
		assert(r.size() == 3 && (r.rows() == 3 || r.cols() == 3));
		T angle = r.norm();
		if (angle == 0) {
			return Eigen::Quaternion<T>::Identity(); //w=1, xyz = 0
		}
		return Eigen::Quaternion<T>(Eigen::AngleAxis<T>(angle, r / angle));
	}


	Eigen::MatrixX<ScalarType> rigidTransformVertices(const Eigen::Vector3<ScalarType>& T, const Eigen::Vector3<ScalarType>& r, const Eigen::MatrixX<ScalarType>& v)
	{
		Eigen::Matrix3<ScalarType> rotation = construct_rotation_matrix(r);
		//3 by nv
		Eigen::MatrixX<ScalarType> output_v = (rotation * v).eval();
		output_v.colwise() += T;
		return output_v;
	}

	Eigen::MatrixX<ScalarType> rigidTransformVerticesJacobian(const Eigen::Vector3<ScalarType>& T, const Eigen::Vector3<ScalarType>& r, const Eigen::MatrixX<ScalarType>& v)
	{
		int nv = v.cols();
		Eigen::MatrixX<ScalarType> dtransformedVertexRB_dx(3 * nv, 6);
		dtransformedVertexRB_dx.setZero();

		std::vector<Eigen::Matrix3<ScalarType>> drotationdr;
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
						Eigen::Vector3<ScalarType> dcol = drotationdr[r_col] * v.col(v_i);
						for (int r_row = 0; r_row < 3; r_row++)
						{
							dtransformedVertexRB_dx(3 * v_i + r_row, 3 + r_col) = dcol[r_row];
						}
					}
				}
			});

		return dtransformedVertexRB_dx;
	}

	std::vector<Eigen::MatrixX<ScalarType>> rigidTransformVerticesHessians(const Eigen::Vector3<ScalarType>& T, const Eigen::Vector3<ScalarType>& r, const Eigen::MatrixX<ScalarType>& v)
	{
		int nv = v.cols();
		std::vector<std::vector<Eigen::Matrix3<ScalarType>>> d2rotationdr2;
		soutil::construct_rotation_matrix_hessians(r, d2rotationdr2);

		std::vector<Eigen::MatrixX<ScalarType>> d2transformedVertexRB_dx2;
		d2transformedVertexRB_dx2.resize(6);
		for (int theta_i = 0; theta_i < 6; theta_i++)
		{
			d2transformedVertexRB_dx2[theta_i].resize(3 * nv, 6);
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
					Eigen::Vector3<ScalarType> d2x_dthetak_dthetal = d2rotationdr2[theta_l][theta_k] * v.col(v_i);
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

}
