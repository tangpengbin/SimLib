#pragma once
#ifndef SO_TYPES
#define SO_TYPES
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif // !_USE_MATH_DEFINES


#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>


//typedef std::vector<double> dVector;
typedef Eigen::Vector3d V3D;
typedef Eigen::Vector2d V2D;
//typedef Eigen::VectorXd dVector;
typedef Eigen::Triplet<double> Tripletd;
typedef std::vector<Eigen::Triplet<double> > TripVec;
typedef Eigen::SparseMatrix<double> SpMat;
typedef Eigen::Array<bool, Eigen::Dynamic, 1> EigenArrayXb;

template<typename Scaler>
struct compareVector2 {
    bool operator()(const Eigen::Matrix<Scaler, 2, 1>& a, const Eigen::Matrix<Scaler, 2, 1>& b) const {
        return a.x() != b.x() ? a.x() < b.x() : a.y() < b.y();
    }
};

template<typename Scaler>
struct compareVector3 {
    bool operator()(const Eigen::Matrix<Scaler, 3, 1>& a, const Eigen::Matrix<Scaler, 3, 1>& b) const {
        if (a.x() != b.x())
            return a.x() < b.x();
        else
        {
            compareVector2<Scaler> comp_2d;
            return comp_2d(a.tail(2), b.tail(2));
            //return a.y() != b.y() ? a.y() < b.y() : a.z() < b.z();
        }
    }
};
template<typename Scaler>
struct compareVector4 {
    bool operator()(const Eigen::Matrix<Scaler, 4, 1>& a, const Eigen::Matrix<Scaler, 4, 1>& b) const {
        if (a.x() != b.x())
            return a.x() < b.x();
        else
        {
            compareVector3<Scaler> comp_3d;
            return comp_3d(a.tail(3), b.tail(3));
            /*if (a.z() != b.z())
                return a.z() < b.z();
            else
            {
                return a.x() != b.x() ? a.x() < b.x() : a.y() < b.y();
            }*/
        }
    }
};

#endif