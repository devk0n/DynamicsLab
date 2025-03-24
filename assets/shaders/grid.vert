#version 330 core
layout(location = 0) in vec3 aPos;
uniform mat4 u_viewProjection;
out vec3 vWorldPos;

void main() {
    vWorldPos = aPos;  // Pass world position to fragment shader
    gl_Position = u_viewProjection * vec4(aPos, 1.0);
}