#version 410 core

#include "lighting_shadow.glsl"

in vec3 positionWorld;
in vec3 normalWorld;
in vec2 vTexCoord;

in vec4 lightSpacePos;

out vec4 FragColor;

uniform sampler2D texture_0;
uniform float roughness;

void main(){
	//material settings
	vec4 color = texture2D(texture_0, vTexCoord);
	vec3 color_ambient = color.xyz;
	vec3 color_diffuse = color.xyz;
	vec3 color_specular = vec3(1.0, 1.0, 1.0);
	
	FragColor.xyz = computeLightingShadow(positionWorld, normalWorld, color_ambient, color_diffuse, color_specular, roughness, lightSpacePos);
	FragColor.a = color.a;
}