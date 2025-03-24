#version 330 core
in vec3 vWorldPos;
out vec4 FragColor;

uniform vec3 u_cameraPos;
uniform float u_time;  // Optional: For animated stars

// Sky colors
#define SKY_COLOR_BOTTOM vec3(0.05, 0.05, 0.1)  // Dark blue at the bottom
#define SKY_COLOR_TOP vec3(0.0, 0.0, 0.0)       // Black at the top
#define STAR_COLOR vec3(1.0, 1.0, 1.0)          // White stars

// Star settings
#define STAR_DENSITY 0.02  // Density of stars
#define STAR_BRIGHTNESS 0.5 // Brightness of stars

// Random function for stars
float random(vec2 st) {
    return fract(sin(dot(st.xy, vec2(12.9898, 78.233))) * 43758.5453);
}

void main() {
    // Normalize world position for sky gradient
    vec3 dir = normalize(vWorldPos - u_cameraPos);
    float gradient = smoothstep(-1.0, 1.0, dir.y);  // Vertical gradient

    // Sky gradient
    vec3 skyColor = mix(SKY_COLOR_BOTTOM, SKY_COLOR_TOP, gradient);

    // Stars
    vec2 starCoord = dir.xz / (1.0 + dir.y);  // Project onto a plane
    starCoord *= 100.0;  // Scale for star density
    float starValue = random(floor(starCoord));
    if (starValue < STAR_DENSITY) {
        float starBrightness = smoothstep(0.0, 1.0, random(starCoord)) * STAR_BRIGHTNESS;
        skyColor += starBrightness * STAR_COLOR;
    }

    // Output final color
    FragColor = vec4(skyColor, 1.0);
}