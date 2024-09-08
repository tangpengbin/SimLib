#version 410 core

#include "lighting.glsl"

in vec3 positionWorld;
in vec3 normalWorld;
in vec4 colorInstance;

uniform float roughness;

out vec4 FragColor;

void main(){
	
	vec3 color_ambient = colorInstance.xyz;
	vec3 color_diffuse = colorInstance.xyz;
	vec3 color_specular = vec3(1.0,  1.0, 1.0);
	float color_alpha = colorInstance.w;
	
	FragColor.xyz = computeLighting(positionWorld, normalWorld, color_ambient, color_diffuse, color_specular, roughness);
	FragColor.a = color_alpha;
}
