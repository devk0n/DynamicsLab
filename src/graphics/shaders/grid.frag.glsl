#version 460 core

uniform vec3 u_Color;
out vec4 FragColor;

void main() {
    FragColor = vec4(u_Color, 1.0);
}