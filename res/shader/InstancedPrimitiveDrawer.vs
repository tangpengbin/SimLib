#version 410 core

layout(location = 0) in vec3 vertexPosition_modelspace;
layout(location = 1) in vec3 normal_modelspace;
layout(location = 2) in vec3 center;
layout(location = 3) in float radius;
layout(location = 4) in vec4 colorInstanceIn;

uniform mat4 viewProjection;

out vec3 positionWorld;
out vec3 normalWorld;
out vec4 colorInstance;

void main(){	
	mat4 model = mat4(vec4(radius,  0.0, 0.0, 0.0), vec4(0.0, radius, 0.0, 0.0), vec4(0.0, 0.0, radius, 0.0), vec4(center[0], center[1], center[2], 1.0));
	gl_Position = viewProjection * vec4(radius * vertexPosition_modelspace + center, 1.0);
	vec4 pw = vec4(radius * vertexPosition_modelspace + center, 1.0);
	positionWorld = pw.xyz / pw.w;
	normalWorld = normal_modelspace;
	colorInstance = colorInstanceIn;
}

