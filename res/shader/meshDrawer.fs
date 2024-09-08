#version 410 core

#include "lighting.glsl"

in vec3 positionWorld;
in vec3 normalWorld;
in vec3 fragmentColor;
uniform float roughness;
uniform float color_alpha;
uniform bool use_lighting;

out vec4 FragColor;

void main(){
	//vec3 color = vec3(0.7,0.7,0.7);
	if(use_lighting)
		FragColor.xyz = computeLighting(positionWorld, normalWorld, fragmentColor, fragmentColor, fragmentColor, roughness);
	else
		FragColor.xyz = fragmentColor;
	FragColor.a = color_alpha;
}
