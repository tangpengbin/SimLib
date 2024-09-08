#version 410 core

in vec2 vTexCoord;
uniform sampler2D texture_0;

out vec4 FragColor;

void main(){
	vec4 color = texture2D(texture_0, vTexCoord);
	
	FragColor.xyz = color.xyz;
	FragColor.a = color.a;
}