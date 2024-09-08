#version 410 core

#include "lighting.glsl"

in vec3 positionWorld;
in vec3 normalWorld;
in vec2 vTexCoord;

uniform sampler2D texture_0;

out vec4 FragColor;

void main(){
	//material settings
	vec4 color = texture2D(texture_0, vTexCoord);
	vec3 color_ambient = color.xyz;
	vec3 color_diffuse = color.xyz;
	vec3 color_specular = vec3(1.0, 1.0, 1.0);
	float roughness = 0.1;
	
	FragColor.xyz = computeLighting(positionWorld, normalWorld, color_ambient, color_diffuse, color_specular, roughness);
	FragColor.a = color.a;
}
