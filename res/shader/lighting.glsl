
uniform vec3 cameraPositionWorld;
uniform vec3 ambientLight;
uniform vec3 lightDirection;
uniform vec3 lightColor;

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

vec3 computeLighting(vec3 positionWorld, vec3 normalWorld, vec3 color_ambient, vec3 color_diffuse, vec3 color_specular, float roughness){
	
	vec3 n = normalize(normalWorld);
	vec3 l = -lightDirection; //l points to light, but because it is at infinity it is the negative direction of lightDirection
	vec3 v = normalize(cameraPositionWorld - positionWorld ); // v points from fragment point to viewer
	//vec3 r = reflect( -l, n );
	vec3 hr = normalize(v + l);
	
	float NDotHr = dot(n, hr);
	float NDotV = dot(n, v);
	//float NDotHr = max(0.0, dot(n, hr));
	//float NDotV = max(0.0, dot(n, v));
	float NDotL = dot( l, n );
	vec3 ambient = ambientLight * color_ambient;
	vec3 diffuse = lightColor * color_diffuse * NDotL;
	float alpha = roughness*roughness;
	vec3 spec = lightColor * color_specular * mModel(alpha, NDotV, NDotHr);
	
	vec3 fc;
	fc = ambient + diffuse + spec;
	return fc;
}