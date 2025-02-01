#version 460 core

// Use double precision for inputs.
layout(location = 0) in dvec3 aPos;
layout(location = 1) in dvec3 aNormal;

// Uniform matrices in double precision.
uniform dmat4 u_Model;
uniform dmat4 u_View;
uniform dmat4 u_Projection;

// Output varyings for the fragment shader now use single precision.
out vec3 v_FragPos;
out vec3 v_Normal;

void main()
{
    // Compute world position in double precision.
    dvec4 worldPos = u_Model * dvec4(aPos, 1.0);
    // Convert to float for interpolation.
    v_FragPos = vec3(worldPos);

    // Compute the normal in double precision then convert to float.
    dvec3 normalDouble = mat3(transpose(inverse(u_Model))) * aNormal;
    v_Normal = vec3(normalDouble);

    // Compute clip space position.
    gl_Position = vec4(u_Projection * u_View * worldPos);
}
