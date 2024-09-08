#version 410 core

#include "lighting.glsl"

in vec3 positionWorld;
in vec3 normalWorld;

uniform vec3 endPos1;
uniform vec3 endPos2;

uniform vec3 color_ambient1;
uniform vec3 color_diffuse1;
uniform vec3 color_specular1;
uniform float roughness1;
uniform float color_alpha1;

uniform vec3 color_ambient2;
uniform vec3 color_diffuse2;
uniform vec3 color_specular2;
uniform float roughness2;
uniform float color_alpha2;

out vec4 FragColor;

void main(){
	vec3 finalColor_ambient;
	vec3 finalColor_Diffuse;
	vec3 finalColor_Specular;
	float finalColor_roughness;
	float finalColor_alpha;
	if(endPos1 != endPos2)
	{
		vec3 v1p = positionWorld - endPos1;
		vec3 v12 = endPos2 - endPos1;
		float v12_length = length(v12);
		float v1p_in_v12_length = dot(v12,v1p) / v12_length;

		float v1_color_percent = v1p_in_v12_length / v12_length;
		float v2_color_percent = 1.0 - v1_color_percent;

		finalColor_ambient = color_ambient1 * v2_color_percent + color_ambient2 * v1_color_percent;
		finalColor_Diffuse = color_diffuse1 * v2_color_percent + color_diffuse2 * v1_color_percent;
		finalColor_Specular = color_specular1 * v2_color_percent + color_specular2 * v1_color_percent;
		finalColor_roughness = roughness1 * v2_color_percent + roughness2 * v1_color_percent;
		finalColor_alpha = color_alpha1 * v2_color_percent + color_alpha2 * v1_color_percent;
	}
	else
	{
		finalColor_ambient = color_ambient1;
		finalColor_Diffuse = color_diffuse1;
		finalColor_Specular = color_specular1;
		finalColor_roughness = roughness1;
		finalColor_alpha = color_alpha1;
	}
	//FragColor.xyz = computeLighting(positionWorld, normalWorld, color_ambient, color_diffuse, color_specular, roughness);
	//FragColor.a = color_alpha;
	FragColor.xyz = computeLighting(positionWorld, normalWorld, finalColor_ambient, finalColor_Diffuse, finalColor_Specular, finalColor_roughness);
	FragColor.a = finalColor_alpha;
}
