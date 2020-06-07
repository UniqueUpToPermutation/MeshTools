#version 330 core
layout (location = 0) in vec3 aPosition;
layout (location = 1) in  vec2 aTexcoord;
layout (location = 2) in vec3 aNormal;
layout (location = 3) in  vec3 aColor;

uniform mat4 worldMatrix;
uniform mat4 viewMatrix;
uniform mat4 projectionMatrix;
uniform mat4 worldInverseTranspose;

out vec3 vertColor;
out vec3 normal;
out vec3 position;

void main()
{
	vec4 pos4 = vec4(aPosition, 1.0);
	vec4 norm4 = vec4(aNormal, 0.0);
	vec4 worldPos4 = worldMatrix * pos4;
    
	vertColor = aColor;
	normal = (worldInverseTranspose * norm4).xyz;
	position = worldPos4.xyz;

	gl_Position = projectionMatrix * viewMatrix * worldPos4;
}