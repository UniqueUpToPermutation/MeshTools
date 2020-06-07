#version 330 core
layout (location = 0) in vec3 aPosition;
layout (location = 1) in vec2 texcoord;
layout (location = 2) in vec3 normal;
layout (location = 3) in vec3 aColor;

uniform mat4 worldMatrix;
uniform mat4 viewMatrix;
uniform mat4 projectionMatrix;

out vec3 vertColor;

void main()
{
	vec4 pos4 = vec4(aPosition, 1.0);
    gl_Position = projectionMatrix * viewMatrix * worldMatrix * pos4;
	vertColor = aColor;
}