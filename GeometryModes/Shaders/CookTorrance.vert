#version 330 core

layout (location = 0) in vec3 position;
layout (location = 1) in vec2 texcoords;
layout (location = 2) in vec3 normal;
layout (location = 3) in vec3 color;

out vec2 vTexcoords; 
out vec3 vNormal;
out vec3 vViewDir;
out vec3 vLightDir;
out float vLightDistance2;
out vec3 vVertexColor;

uniform mat4 model;
uniform mat4 modelInverseTranspose;
uniform mat4 view; 
uniform mat4 projection; 
uniform vec3 eyePosition;
uniform vec3 lightPosition;

void main()
{
    vTexcoords = texcoords;
	vNormal = (modelInverseTranspose * vec4(normal, 0)).xyz;
	vec4 worldPosition = model * vec4(position, 1.0f);
	vViewDir = normalize(eyePosition - worldPosition.xyz);
	vLightDir = lightPosition - worldPosition.xyz;
	vLightDistance2 = length(vLightDir);
	vLightDir /= vLightDistance2;
	vLightDistance2 *= vLightDistance2;
	vVertexColor = color;
    gl_Position = projection * view * worldPosition;
}