#version 410 core

uniform vec3 pt0;
uniform vec3 pt1;
uniform vec3 pt2;
uniform vec3 pt3;
uniform float radius1;
uniform float radius2;


layout(location = 0) in vec3 vertexPosition_modelspace;
layout(location = 1) in vec3 normal_modelspace;

uniform mat4 MVP;
uniform mat4 Model;
uniform mat3 ModelInverseTranspose;

out vec3 positionWorld;
out vec3 normalWorld;



void main()
{
	vec4 pos = vec4(vertexPosition_modelspace,1.0);
	
	float t = pos.z;
	float t2 = t*t;
	float t3 = t2*t;

	// position
	vec3 point = 0.5 * ( ( 2.0 * pt1 ) + ( -pt0 + pt2 ) * t +
				( 2.0*pt0 - 5.0*pt1 + 4.0*pt2 - pt3 ) * t2 +
				( -pt0 + 3.0*pt1 - 3.0*pt2 + pt3 ) * t3 );
	// tangent, gradient of position
	vec3 tangent = normalize (0.5 * ( ( -pt0 + pt2 ) +
                ( 2.0*pt0 - 5.0*pt1 + 4.0*pt2 - pt3 ) * 2.0*t +
                ( -pt0 + 3.0*pt1 - 3.0*pt2 + pt3 ) * 3.0*t2 ) );
	/*// Normal
	float tdotu = dot(tangent,vec3(0.0,1.0,0.0));
	vec3 rU = normalize(mix(vec3(1,0,0),vec3(0,1,0),1-tdotu));
	vec3 dx = (cross( rU, tangent ) );
	if( dot(dx,dx) == 0.0 )
	{
		dx = vec3(1.0,0.0,0.0);
	}
	vec3 dy = ( cross(tangent, dx) );
	vec3 normal = normalize( dx*pos.x+dy*pos.y );*/


	// Normal, using parallel transport
	//if the tagent is opposite to the vec3(0.0,0.0,1.0), then the output normal might have a problem
	vec3 m_vTo = tangent;
	vec3 x = vertexPosition_modelspace - vec3(0.0,0.0,t);
	
	float sinTheta = length(cross(vec3(0.0,0.0,1.0),m_vTo));
	float cosTheta = dot(vec3(0.0,0.0,1.0),m_vTo);
	vec3 normal;
	if(abs(cosTheta + 1.0)>1e-4 && abs(cosTheta - 1.0)>1e-4)
	{
		vec3 axis = normalize(cross(vec3(0.0,0.0,1.0),m_vTo));
		normal = cosTheta* x + sinTheta * cross(axis,x) + (1.0-cosTheta)*dot(axis, x)*axis;
	}
	else
	{//for the case of tagent is parallel to the axis of the cylinder, we use its neighbor tagent and normal to compute its current normal
		float t_last = pos.z - 0.01;
		float t2_last = t_last*t_last;
		vec3 tangent_last = normalize (0.5 * ( ( -pt0 + pt2 ) +
					( 2.0*pt0 - 5.0*pt1 + 4.0*pt2 - pt3 ) * 2.0*t_last +
					( -pt0 + 3.0*pt1 - 3.0*pt2 + pt3 ) * 3.0*t2_last ) );
		vec3 m_vTo_last = tangent_last;
		vec3 x_last = x;
		float sinTheta_last = length(cross(vec3(0.0,0.0,1.0),m_vTo_last));
		float cosTheta_last = dot(vec3(0.0,0.0,1.0),m_vTo_last);
		vec3 axis_last = normalize(cross(vec3(0.0,0.0,1.0),m_vTo_last));
		vec3 normal_last = cosTheta_last* x_last + sinTheta_last * cross(axis_last,x_last) + (1.0-cosTheta_last)*dot(axis_last, x_last)*axis_last;
		
		//use the last tangent and normal to parallel transport to the current normal
		float tangent_sinTheta = length(cross(tangent_last,tangent));
		float tangent_cosTheta = dot(tangent_last,tangent);
		vec3 tangent_axis = normalize(cross(tangent_last,tangent));
		normal = tangent_cosTheta* normal_last + tangent_sinTheta * cross(tangent_axis,normal_last) + (1.0-tangent_cosTheta)*dot(tangent_axis, normal_last)*tangent_axis;
	}
	// Displace the surface
	pos.xyz = point + normal*mix(radius1,radius2, t);

	vec4 worldPosition = MVP * vec4(pos.xyz, 1.0);;
	gl_Position = worldPosition;
	vec4 pw = Model * vec4(pos.xyz, 1.0);
	positionWorld = pw.xyz / pw.w;
	
	normalWorld = normalize(ModelInverseTranspose * normal);
	
}
