#version 330 core
in vec3 vWorldPos;
out vec4 FragColor;

uniform vec3 u_cameraPos;
uniform float u_scale;

// Fog parameters
uniform vec3 u_fogColor;
uniform float u_fogStart;
uniform float u_fogEnd;

// Grid parameters
#define MAJOR_GRID_COLOR vec3(0.5)
#define MINOR_GRID_COLOR vec3(0.2)
#define Y_AXIS_COLOR vec3(0.0, 1.0, 1.0)// Red for X-axis
#define X_AXIS_COLOR vec3(1.0, 0.0, 1.0)// Green for Y-axis
#define BACKGROUND vec3(0.1)
#define GRID_TRANSPARENCY 0.1// 10% transparency for space between lines
#define GRID_SIZE 1000.0// Grid size in meters

float computeGrid(vec2 coord, float spacing, float lineWidth) {
    vec2 gridCoord = coord / spacing;
    vec2 derivative = fwidth(gridCoord);
    vec2 gridLine = abs(fract(gridCoord - 0.5) - 0.5);
    gridLine = smoothstep(lineWidth * derivative, vec2(0.0), gridLine);
    return clamp(max(gridLine.x, gridLine.y), 0.0, 1.0);
}

void main() {
    // Discard fragments outside the 250x250 meter boundary
    if (abs(vWorldPos.x) > GRID_SIZE * 0.5 || abs(vWorldPos.y) > GRID_SIZE * 0.5) {
        discard;
    }

    // Align grid with XY plane (Z+ is upwards)
    vec2 coordXY = vec2(vWorldPos.x, vWorldPos.y);

    // Major grid lines (every 10 units)
    float majorGrid = computeGrid(coordXY, 10.0, 1.5);

    // Minor grid lines (every 1 unit)
    float minorGrid = computeGrid(coordXY, 1.0, 0.5);

    // Axis lines (X-axis in red, Y-axis in green)
    vec2 axisWidth = fwidth(coordXY);
    float xAxis = smoothstep(axisWidth.x * 2.0, 0.0, abs(vWorldPos.x));
    float yAxis = smoothstep(axisWidth.y * 2.0, 0.0, abs(vWorldPos.y));

    // Combine grid and axis
    vec3 color = mix(BACKGROUND, MINOR_GRID_COLOR, minorGrid);
    color = mix(color, MAJOR_GRID_COLOR, majorGrid);
    color = mix(color, X_AXIS_COLOR, xAxis);
    color = mix(color, Y_AXIS_COLOR, yAxis);

    // Distance fade
    float cameraDist = distance(u_cameraPos, vWorldPos);
    float fade = 1.0 - smoothstep(0.0, 100.0, cameraDist);
    color *= fade;

    // Set alpha based on grid lines
    float alpha = max(majorGrid, minorGrid);// Use grid lines to determine opacity
    alpha = mix(GRID_TRANSPARENCY, 1.0, alpha);// 10% transparency for space between lines

    // Fog calculation
    float fogFactor = clamp((u_fogEnd - cameraDist) / (u_fogEnd - u_fogStart), 0.0, 1.0);

    // Apply fog to the color
    vec3 finalColor = mix(u_fogColor, color, fogFactor);

    // Apply fog to the alpha channel
    float finalAlpha = mix(1.0, alpha, fogFactor);// Fog reduces transparency

    FragColor = vec4(finalColor, finalAlpha);
}