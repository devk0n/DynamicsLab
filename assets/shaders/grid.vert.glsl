#version 460 core

layout (location = 0) in dvec3 a_Position; // Input double-precision data

uniform dmat4 u_Model;
uniform dmat4 u_Projection;               // Double-precision uniform
uniform dmat4 u_View;                     // Double-precision uniform

void main() {
    // Convert double-precision to single-precision for gl_Position
    gl_Position = mat4(u_Projection) * mat4(u_View) * mat4(u_Model) * vec4(a_Position, 1.0);
}