#ifndef RENDERER_H
#define RENDERER_H

#include <vector>
#include <glm/glm.hpp>

#include "Shader.h"
#include "GroundPoint.h"
#include "RigidBody.h"

class Renderer {
public:
	bool initialize();

	void beginFrame() const;

	void render(const std::vector<GroundPoint> &groundPoints,
	            const std::vector<RigidBody> &rigidBodies,
	            const glm::mat4 &view,
	            const glm::mat4 &projection) const;

	void drawAxes(const glm::mat4 &modelMatrix);

	static void endFrame();

	void shutdown() const;

	void drawAxes(const Eigen::Vector3d &translation, const glm::mat4 &view, const glm::mat4 &projection) const;

	void setClearColor(float r, float g, float b, float a);

	static void captureScreenshot();

	bool wireframeMode = false;

private:
	Shader m_bodyShader, m_axesShader;
	GLuint m_axesVAO, m_axesVBO;

	float m_clearColor[4] = {0.1f, 0.1f, 0.1f, 1.0f};

	bool m_initialized = false;
};


#endif // RENDERER_H
