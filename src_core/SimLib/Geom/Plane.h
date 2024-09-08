#pragma once

#include <Eigen/Core>

/** plane represented through n.dot(x) + d = 0 */
template<typename Scalar, int Dim>
class Plane
{
public:
	typedef Eigen::Matrix<Scalar, Dim, 1> VectorType;

private:
	Plane(const VectorType &n, const Scalar &d)
		:m_n(n),
		m_d(d)
	{

	}

public:

	static Plane<Scalar, Dim> fromNormal(const VectorType &n, Scalar d)
	{
		return Plane(n, d);
	}

	static Plane<Scalar, Dim> fromPointNormal(const VectorType &p, const VectorType &n)
	{
		// eval(p) = 0 => n.dot(p) + d = 0 => d = - n.dot(p)
		Scalar d = -n.dot(p);
		return Plane(n, d);
	}

	Scalar eval(const VectorType &v) const
	{
		return m_n.dot(v) + m_d;
	}

	VectorType& getNormal()
	{
		return m_n;
	}

	const VectorType& getNormal() const
	{
		return m_n;
	}

	Scalar& getD()
	{
		return m_d;
	}

	const Scalar& getD() const
	{
		return m_d;
	}

private:

	VectorType m_n;
	Scalar m_d;
};

typedef Plane<double, 2> Plane2d;
typedef Plane<double, 3> Plane3d;
