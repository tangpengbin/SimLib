#ifndef CODEGEN_COMPUTEFIXDOFENERGY_H
#define CODEGEN_COMPUTEFIXDOFENERGY_H

#include <Eigen/Core>

namespace Codegen { 
void computeFixDoFEnergy(double stiffness, const Eigen::Matrix<double,1,1> & x, const Eigen::Matrix<double,1,1> & X, double& energy);
void computeFixDoFEnergyGradient(double stiffness, const Eigen::Matrix<double,1,1> & x, const Eigen::Matrix<double,1,1> & X, Eigen::Matrix<double, 1, 1>& gradient);
void computeFixDoFEnergyHessian(double stiffness, const Eigen::Matrix<double,1,1> & x, const Eigen::Matrix<double,1,1> & X, Eigen::Matrix<double, 1, 1>& hessian);
void computeFixDoFEnergyHessianProductJacobian(double stiffness, const Eigen::Matrix<double,1,1> & x, const Eigen::Matrix<double,1,1> & X, const Eigen::Matrix<double,1,1> & v, Eigen::Matrix<double, 1, 1>& hessianProductJacobian);
void computeFixDoFEnergyRestConfJac(double stiffness, const Eigen::Matrix<double,1,1> & x, const Eigen::Matrix<double,1,1> & X, Eigen::Matrix<double, 1, 1>& rest_conf_jac);
void computeFixDoFEnergyParam_jac(double stiffness, const Eigen::Matrix<double,1,1> & x, const Eigen::Matrix<double,1,1> & X, Eigen::Matrix<double, 1, 1>& param_jac);
void computeFixDoFEnergyParam_thirdOrderDerivatives(double stiffness, const Eigen::Matrix<double,1,1> & x, const Eigen::Matrix<double,1,1> & X, Eigen::Matrix<double, 1, 1>& d3_0);
void computeFixDoFEnergyThirdOrderDerivatives(double stiffness, const Eigen::Matrix<double,1,1> & x, const Eigen::Matrix<double,1,1> & X, Eigen::Matrix<double, 1, 1>& d3_0);
void computeFixDoFEnergyRestConfThirdOrderDerivatives(double stiffness, const Eigen::Matrix<double,1,1> & x, const Eigen::Matrix<double,1,1> & X, Eigen::Matrix<double, 1, 1>& d3_0);
void computeFixDoFEnergyFourthOrderDerivatives(double stiffness, const Eigen::Matrix<double,1,1> & x, const Eigen::Matrix<double,1,1> & X, const Eigen::Matrix<double,1,1> & v1, const Eigen::Matrix<double,1,1> & v2, 
	Eigen::Matrix<double, 1, 1>& fourthOrderDerivative);
void computeFixDoFEnergyThirdOrderToParameterFourthOrderDerivatives(double stiffness, const Eigen::Matrix<double,1,1> & x, const Eigen::Matrix<double,1,1> & X, const Eigen::Matrix<double,1,1> & v1, const Eigen::Matrix<double,1,1> & v2, 
	Eigen::Matrix<double, 1, 1>& thirdOrderToParameterFourthOrderDerivative);

 } 
#endif