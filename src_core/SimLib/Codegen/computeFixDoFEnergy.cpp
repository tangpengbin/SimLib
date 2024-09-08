#include "computeFixDoFEnergy.h"

namespace Codegen { 
void computeFixDoFEnergy(double stiffness, const Eigen::Matrix<double,1,1> & x, const Eigen::Matrix<double,1,1> & X, double& energy){
	double _i_var0, _i_var1, _i_var2, _i_var3, _i_var4;
	_i_var0 = (x(0,0))-(X(0,0));
	_i_var1 = 0.5;
	_i_var2 = (_i_var0)*(_i_var0);
	_i_var3 = (_i_var1)*(stiffness);
	_i_var4 = (_i_var3)*(_i_var2);
	energy = _i_var4;
}
void computeFixDoFEnergyGradient(double stiffness, const Eigen::Matrix<double,1,1> & x, const Eigen::Matrix<double,1,1> & X, Eigen::Matrix<double, 1, 1>& gradient){
	double _i_var0, _i_var1, _i_var2, _i_var3, _i_var4, _i_var5;
	_i_var0 = 0.5;
	_i_var1 = (x(0,0))-(X(0,0));
	_i_var2 = (_i_var0)*(stiffness);
	_i_var3 = (_i_var2)*(_i_var1);
	_i_var4 = 2;
	_i_var5 = (_i_var4)*(_i_var3);
	gradient(0,0) = _i_var5;
}
void computeFixDoFEnergyHessian(double stiffness, const Eigen::Matrix<double,1,1> & x, const Eigen::Matrix<double,1,1> & X, Eigen::Matrix<double, 1, 1>& hessian){
	double _i_var0, _i_var1, _i_var2, _i_var3;
	_i_var0 = 0.5;
	_i_var1 = (_i_var0)*(stiffness);
	_i_var2 = 2;
	_i_var3 = (_i_var2)*(_i_var1);
	hessian(0,0) = _i_var3;
}
void computeFixDoFEnergyHessianProductJacobian(double stiffness, const Eigen::Matrix<double,1,1> & x, const Eigen::Matrix<double,1,1> & X, const Eigen::Matrix<double,1,1> & v, Eigen::Matrix<double, 1, 1>& hessianProductJacobian){
	double _i_var0;
	_i_var0 = 0;
	hessianProductJacobian(0,0) = _i_var0;
}
void computeFixDoFEnergyRestConfJac(double stiffness, const Eigen::Matrix<double,1,1> & x, const Eigen::Matrix<double,1,1> & X, Eigen::Matrix<double, 1, 1>& rest_conf_jac){
	double _i_var0, _i_var1, _i_var2, _i_var3, _i_var4, _i_var5;
	_i_var0 = 0.5;
	_i_var1 = (_i_var0)*(stiffness);
	_i_var2 = 2;
	_i_var3 = -1;
	_i_var4 = (_i_var2)*(_i_var1);
	_i_var5 = (_i_var4)*(_i_var3);
	rest_conf_jac(0,0) = _i_var5;
}
void computeFixDoFEnergyParam_jac(double stiffness, const Eigen::Matrix<double,1,1> & x, const Eigen::Matrix<double,1,1> & X, Eigen::Matrix<double, 1, 1>& param_jac){
	double _i_var0, _i_var1, _i_var2, _i_var3, _i_var4;
	_i_var0 = (x(0,0))-(X(0,0));
	_i_var1 = 2;
	_i_var2 = 0.5;
	_i_var3 = (_i_var1)*(_i_var0);
	_i_var4 = (_i_var3)*(_i_var2);
	param_jac(0,0) = _i_var4;
}
void computeFixDoFEnergyParam_thirdOrderDerivatives(double stiffness, const Eigen::Matrix<double,1,1> & x, const Eigen::Matrix<double,1,1> & X, Eigen::Matrix<double, 1, 1>& d3_0){
	double _i_var0;
	_i_var0 = 1;
	d3_0(0,0) = _i_var0;
}
void computeFixDoFEnergyThirdOrderDerivatives(double stiffness, const Eigen::Matrix<double,1,1> & x, const Eigen::Matrix<double,1,1> & X, Eigen::Matrix<double, 1, 1>& d3_0){
	double _i_var0;
	_i_var0 = 0;
	d3_0(0,0) = _i_var0;
}
void computeFixDoFEnergyRestConfThirdOrderDerivatives(double stiffness, const Eigen::Matrix<double,1,1> & x, const Eigen::Matrix<double,1,1> & X, Eigen::Matrix<double, 1, 1>& d3_0){
	double _i_var0;
	_i_var0 = 0;
	d3_0(0,0) = _i_var0;
}
void computeFixDoFEnergyFourthOrderDerivatives(double stiffness, const Eigen::Matrix<double,1,1> & x, const Eigen::Matrix<double,1,1> & X, const Eigen::Matrix<double,1,1> & v1, const Eigen::Matrix<double,1,1> & v2, 
	Eigen::Matrix<double, 1, 1>& fourthOrderDerivative){
	double _i_var0;
	_i_var0 = 0;
	fourthOrderDerivative(0,0) = _i_var0;
}
void computeFixDoFEnergyThirdOrderToParameterFourthOrderDerivatives(double stiffness, const Eigen::Matrix<double,1,1> & x, const Eigen::Matrix<double,1,1> & X, const Eigen::Matrix<double,1,1> & v1, const Eigen::Matrix<double,1,1> & v2, 
	Eigen::Matrix<double, 1, 1>& thirdOrderToParameterFourthOrderDerivative){
	double _i_var0;
	_i_var0 = 0;
	thirdOrderToParameterFourthOrderDerivative(0,0) = _i_var0;
}

 } 

