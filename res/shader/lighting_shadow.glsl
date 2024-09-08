
uniform vec3 cameraPositionWorld;
uniform vec3 ambientLight;
uniform vec3 lightDirection;
uniform vec3 lightColor;

uniform sampler2DShadow shadowMap;
uniform float bias;
uniform float shadowSamplingScale;

float CalcShadowFactor(vec4 LightSpacePos)
{
	LightSpacePos.xyz = 0.5*LightSpacePos.xyz+0.5;
	float visibility=textureProj(shadowMap, vec4(LightSpacePos.xy,  LightSpacePos.z-bias,LightSpacePos.w));

	return visibility;
}
#define COUNT 5
//#define COUNT 7
float CalcShadowFactorSoft(vec4 LightSpacePos)
{
	float vis = 0.0;
	float weights[COUNT] = float[COUNT](0.1, 0.2, 0.3, 0.2, 0.1);
	//float weights[COUNT] = float[COUNT](0.1, 0.2, 0.3, 0.4, 0.3, 0.2, 0.1);
	float sum = 0.0;
	for(int i = 0; i < COUNT; i++) {
		for(int j=0; j<COUNT; j++){
			vec4 p = LightSpacePos + vec4(shadowSamplingScale * (i - 2), shadowSamplingScale * (j - 2), 0.0, 0.0);
			vis = vis + weights[i] * weights[j] * CalcShadowFactor(p);
			sum += weights[i] * weights[j];
		}
	}
	return vis / sum;
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

vec3 computeLightingShadow(vec3 positionWorld, vec3 normalWorld, vec3 color_ambient, vec3 color_diffuse, vec3 color_specular, float roughness, vec4 lightSpacePos){
	
	vec3 n = normalize(normalWorld);
	vec3 l = -lightDirection; //l points to light, but because it is at infinity it is the negative direction of lightDirection
	vec3 v = normalize(cameraPositionWorld - positionWorld ); // v points from fragment point to viewer
	//vec3 r = reflect( -l, n );
	vec3 hr = normalize(v + l);
	
	float NDotHr = dot(n, hr);
	float NDotV = dot(n, v);
	//float sDotN = max( dot( s, n ), 0.0 );
	float NDotL = dot( l, n );
	vec3 ambient = ambientLight * color_ambient;
	vec3 diffuse = lightColor * color_diffuse * NDotL;
	float alpha = roughness*roughness;
	vec3 spec = lightColor * color_specular * mModel(alpha, NDotV, NDotHr);
	
	vec3 fc;
	fc =  ambient + CalcShadowFactorSoft(lightSpacePos) * (diffuse + spec); // this is a hack i.e. we would need to actually sample light paths with sampling to compute soft shadow, but this is good enough for our purposes
	return fc;
}