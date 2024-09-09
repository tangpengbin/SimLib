/*=====================================================================================*/
/*! \file		Material.h
\author		bthomasz
\brief		Declaration of Material class
*/
/*=====================================================================================*/

#ifndef SO_MATERIAL_H
#define SO_MATERIAL_H

enum EMatModel{MM_StVKMod,MM_NH}; 
//enum EBendModel{BM_ANGLE=0,BM_QUADRATIC, BM_DROD}; 

class SOMaterial
{
public:
	SOMaterial() {kB=0,k1=0,k2=0,k3=0,k4=0,kD=0,rho=0;}
	//Bending coefficient
	double kB;
	//Stretching coefficients
	double k1;
	double k2;
	double k3;
	double k4;
	//Damping coefficients
	double kD;
	//mass density
	double rho;
	EMatModel matModel;
	//EBendModel bendModel;


	SOMaterial* clone() const
	{

		return new SOMaterial(*this);
	}
};


#endif