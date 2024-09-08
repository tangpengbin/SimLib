#version 410 core

layout(location = 0) in vec3 vertexPosition_modelspace;
layout(location = 1) in vec3 normal_modelspace;

uniform mat4 viewProjection;
uniform mat4 Model;
uniform mat3 ModelInverseTranspose;

out vec3 positionWorld;
out vec3 normalWorld;
out vec2 vTexCoord;

// this function assumes certain uv coordinates
void main(){	

	vec4 posW = Model * vec4(vertexPosition_modelspace, 1.0);
	gl_Position = viewProjection * posW;
	positionWorld = posW.xyz / posW.w;
	normalWorld = normalize(ModelInverseTranspose * normal_modelspace);
	vTexCoord = vec2(vertexPosition_modelspace.x, 1.0 - vertexPosition_modelspace.y);
}

