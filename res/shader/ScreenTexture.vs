#version 410 core

layout(location = 0) in vec3 vertexPosition_modelspace;

out vec3 positionWorld;
out vec2 vTexCoord;

void main(){

	gl_Position = vec4(vertexPosition_modelspace, 1.0);
	vTexCoord = vertexPosition_modelspace.xy * 0.5 + vec2(0.5, 0.5);
}

