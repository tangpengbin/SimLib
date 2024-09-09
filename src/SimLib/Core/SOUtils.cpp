#include "SOUtils.h"
#include "SOTypes.h"
#include <stdarg.h>

#include "../Geom/TriMesh.h"
#include <iostream>
//#include <Eigen/PardisoSupport>
//#include "mkl.h"

namespace soutil
{
	V2D get2D(int i, const Eigen::VectorXd& vx)
	{
		assert(vx.size() >= 2 * (i + 1));
		V2D v;
		v(0) = vx(2 * i);
		v(1) = vx(2 * i + 1);
		return v;
	}
	void add2D(int i, const V2D& v, Eigen::VectorXd& vx)
	{
		assert(vx.size() >= 2 * (i + 1));
		vx(2 * i) += v(0);
		vx(2 * i + 1) += v(1);
	}
	void set2D(int i, const V2D& v, Eigen::VectorXd& vx)
	{
		assert(vx.size() >= 2 * (i + 1));
		vx(2 * i) = v(0);
		vx(2 * i + 1) = v(1);
	}
	V3D get3D(int i, const Eigen::VectorXd& vx)
	{
		assert(vx.size() >= 3 * (i + 1));
		V3D v;
		v(0) = vx(3 * i);
		v(1) = vx(3 * i + 1);
		v(2) = vx(3 * i + 2);
		return v;
	}

	void add3D(int i, const V3D& v, Eigen::VectorXd& vx)
	{
		assert(vx.size() >= 3 * (i + 1));
		vx(3 * i) += v(0);
		vx(3 * i + 1) += v(1);
		vx(3 * i + 2) += v(2);
	}

	void set3D(int i, const V3D& v, Eigen::VectorXd& vx)
	{
		assert(vx.size() >= 3 * (i + 1));
		vx(3 * i    ) = v(0);
		vx(3 * i + 1) = v(1);
		vx(3 * i + 2) = v(2);
	}

	void scale(Eigen::VectorXd& vx, double s)
	{
		for (int i = 0; i < vx.size(); i++)
			vx(i) *= s;
	}

	void stdToEigenVec(const std::vector<double>& vin, Eigen::VectorXd& vout)
	{
		vout = Eigen::VectorXd(vin.size());
		for (int i = 0; i < vin.size(); i++)
			vout(i) = vin[i];
	}

	Eigen::Vector4d get4D(int i, const Eigen::VectorXd& vx)
	{
		assert(vx.size() >= 4 * (i + 1));
		Eigen::Vector4d v;
		v(0) = vx(4 * i);
		v(1) = vx(4 * i + 1);
		v(2) = vx(4 * i + 2);
		v(3) = vx(4 * i + 3);
		return v;
	}

	V3D crossProd3D(const V3D& u, const V3D& v)
	{
		V3D uxv;
		uxv(0) = u(1)*v(2) - u(2)*v(1);
		uxv(1) = u(2)*v(0) - u(0)*v(2);
		uxv(2) = u(0)*v(1) - u(1)*v(0);
		return uxv;
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


	Eigen::Matrix3d computeRotationFromTwoCoordinateSystems(Eigen::Vector3d x1, Eigen::Vector3d y1, Eigen::Vector3d z1,
		Eigen::Vector3d x2, Eigen::Vector3d y2, Eigen::Vector3d z2) {

		Eigen::Matrix3d coordinate1;
		coordinate1 << x1, y1, z1;

		Eigen::Matrix3d coordinate2;
		coordinate2 << x2, y2, z2;

		Eigen::Matrix3d R = coordinate2 * coordinate1.inverse();
		return R;
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
	double absL2norm(const Eigen::Vector3d& x)
	{
		// Do an explicit abs to avoid possible problems with intervals
		Eigen::Vector3d absx(x.size());
		for (int i = 0; i < x.size(); i++) {
			absx(i) = abs(x(i));
		}
		return sqrt(absx.dot(absx));
	}

	/// Compute sinc(||x||)
	double sinc_normx(const Eigen::Vector3d& x)
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
	Eigen::Vector3d sinc_normx_grad(const Eigen::Vector3d& x)
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
	Eigen::Matrix3d sinc_normx_hess(const Eigen::Vector3d& x)
	{
		double normx = x.norm();
		return ddsinc_over_x2_minus_dsinc_over_x3(normx) * x * x.transpose()
			+ dsinc_over_x(normx) * Eigen::Matrix3d::Identity(x.size(), x.size());
	}

	Eigen::Matrix3d Hat(const Eigen::Vector3d& x)
	{
		Eigen::Matrix3d M;
		double zero(0);
		M.row(0) << zero, -x.z(), x.y();
		M.row(1) << x.z(), zero, -x.x();
		M.row(2) << -x.y(), x.x(), zero;
		return M;
	}

	void Hat_jacobian(const Eigen::Vector3d& x, std::vector<Eigen::Matrix3d>& dhat_dx)
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

	Eigen::Matrix3d construct_rotation_matrix(const Eigen::Vector3d& r)
	{
		double sinc_angle = sinc_normx(r);
		double sinc_half_angle = sinc_normx((r / 2.0).eval());
		Eigen::Matrix3d K = Hat(r);
		Eigen::Matrix3d K2 = K * K;
		Eigen::Matrix3d R = sinc_angle * K + 0.5 * sinc_half_angle * sinc_half_angle * K2;
		R.diagonal().array() += 1.0;
		
		return R;
	}

	void construct_rotation_matrix_jacobian(const Eigen::Vector3d& r, std::vector<Eigen::Matrix3d>& drotation_dr)
	{
		double sinc_angle = sinc_normx(r);
		double sinc_half_angle = sinc_normx((r / 2.0).eval());
		Eigen::Matrix3d K = Hat(r);
		Eigen::Matrix3d K2 = K * K;

		Eigen::Vector3d sinc_angle_grad = sinc_normx_grad(r);

		Eigen::Vector3d sinc_half_angle_grad = sinc_normx_grad((r/2.0).eval());

		std::vector<Eigen::Matrix3d> dK_dx;
		Hat_jacobian(r, dK_dx);

		drotation_dr.resize(3);
		for (int i = 0; i < 3; i++)
		{
			drotation_dr[i] = sinc_angle_grad[i] * K + sinc_angle * dK_dx[i]
				+ 0.5 * sinc_half_angle * sinc_half_angle_grad[i] * K2 + 0.5 * sinc_half_angle * sinc_half_angle * (dK_dx[i] * K + K * dK_dx[i]);
		}

	}

	void construct_rotation_matrix_hessians(const Eigen::Vector3d& r, std::vector<std::vector<Eigen::Matrix3d>>& d2rotation_dr2)
	{
		double sinc_angle = sinc_normx(r);
		double sinc_half_angle = sinc_normx((r / 2.0).eval());
		Eigen::Matrix3d K = Hat(r);
		Eigen::Matrix3d K2 = K * K;

		Eigen::Vector3d sinc_angle_grad = sinc_normx_grad(r);
		Eigen::Matrix3d sinc_angle_hess = sinc_normx_hess(r);

		Eigen::Vector3d sinc_half_angle_grad = sinc_normx_grad((r / 2.0).eval());
		Eigen::Matrix3d sinc_half_angle_hess = sinc_normx_hess((r / 2.0).eval());

		std::vector<Eigen::Matrix3d> dK_dx;
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

}
