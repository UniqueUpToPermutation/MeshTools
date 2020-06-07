#version 330 core
out vec4 FragColor;

uniform vec3 specularColor = vec3(1.0, 1.0, 1.0);
uniform vec3 lightColor = vec3(1.0, 1.0, 1.0);
uniform float specularIntensity = 1.0;
uniform float lightIntensity = 1.0;
uniform vec3 lightDirection;
uniform vec3 ambientColor = vec3(1.0, 1.0, 1.0);
uniform float ambientIntensity = 0.0;
uniform float shininess = 16.0;
uniform vec3 eyePosition;

in vec3 vertColor;
in vec3 normal;
in vec3 position;

void main()
{
	vec3 ambientTerm = ambientColor * vertColor * ambientIntensity;

	vec3 viewDir = normalize(position - eyePosition);
	vec3 halfwayDir = -normalize(lightDirection + viewDir);  

    float spec = pow(max(dot(normal, halfwayDir), 0.0),  shininess);
	vec3 specTerm = spec * lightColor * specularIntensity * specularColor * lightIntensity;

	vec3 diffuseTerm = vertColor * lightColor * max(dot(normal, -lightDirection), 0.0) * lightIntensity;

    FragColor = vec4(ambientTerm + specTerm + diffuseTerm, 1.0);
}