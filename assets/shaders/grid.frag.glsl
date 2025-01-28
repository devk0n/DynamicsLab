#version 460 core

out vec4 FragColor;    // Fragment color output (single-precision)
uniform dvec3 u_Color; // Double-precision uniform

void main() {
    // Convert double-precision to single-precision for output
    FragColor = vec4(u_Color, 1.0);
}