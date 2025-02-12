#version 460 core

in vec3 vertexColor;// Color passed from vertex shader

out vec4 FragColor;

void main() {
    FragColor = vec4(vertexColor, 1.0);
}