
//see at the end of http://graphicrants.blogspot.com/2013/08/specular-brdf-reference.html
float computeBlinnPhongGGXSpecular(float a, float NoV, float NoL){
	//compute 1 sample integral =? L_i * f * N.dot(L)
	//  Geometric Shadowing model: Smith: Use two G terms multiplied
	// ignoring fresnel for now
	float a2 = a*a;
	float G_V = NoV + sqrt( (NoV - NoV * a2) * NoV + a2 );
	float G_L = NoL + sqrt( (NoL - NoL * a2) * NoL + a2 );
	float GGGXSmith = 1.0 / ( G_V * G_L ); //computes G_GGX(l) * G_GGX(v) / (4.0 n.dot(l) n.dot(v))
	float d = NoL * NoL * (a2 - 1.0) + 1.0;
	float DGGXV = a2 / (3.142 * d * d);
	return GGGXSmith * DGGXV;
	//return GGXGGX * 1.0 / (3.142 * a2) * pow(NoL, 2.0 / a2 - 2.0);
}
float DBlinnPhong(float a, float NDotM){
	float a2 = a * a;
	return 1.0 / (3.142 * a2) * pow(NDotM, 2.0 / a2 - 2.0);
}
float DGGX(float a, float NoM){
	float a2 = a*a;
	float d = NoM * NoM * (a2 - 1.0) + 1.0;
	return a2 / (3.142 * d * d);
}
float mModel(float a, float NDotV, float NDotHr){
	// I assume here that i = l, o = v
	// we compute G(i, o, h_r) * D(H_r) / (4.0 * i.dot(n) * o.dot(n))    *  i.dot(n) (from rendering integral or sth) https://en.wikipedia.org/wiki/Rendering_equation
	// G(i, o, h_r) * D(H_r) / (4.0 * o.dot(n))
	// G = G_Schlick =  o.dot(n) / ( o.dot(n) (1 - k) + k )
	float k = a * 0.5;
	return DGGX(a, NDotHr) / (4.0 * ( NDotV * (1.0 - k) + k ) );
}
float computeAFromExponent(float exponent)
{
	return sqrt(2.0 / (exponent + 2.0));
}
