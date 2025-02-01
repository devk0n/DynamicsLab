#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include "GLFW/glfw3.h"
#include <stdexcept>
#include <iostream>
#include <thread>
#include <iomanip>
#include <filesystem>

#include "application.h"
#include "renderer.h"
#include "imgui_layer.h"
#include "tools.h"

using namespace Eigen;

Application::Application(int width, int height, const char* title)
    : m_window(nullptr, glfwDestroyWindow) {

    if (!glfwInit()) {
        throw std::runtime_error("Failed to initialize GLFW");
    }

    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
    m_window.reset(glfwCreateWindow(width, height, title, nullptr, nullptr));

    if (!m_window) {
        glfwTerminate();
        throw std::runtime_error("Failed to create GLFW window");
    }

    glfwMakeContextCurrent(m_window.get());
    glfwSwapInterval(0); // Disable vsync

    m_renderer = std::make_unique<Renderer>(m_window.get());
    glfwSetWindowUserPointer(m_window.get(), m_renderer.get());

    // Register callbacks
    glfwSetCursorPosCallback(m_window.get(), [](GLFWwindow* window, double xpos, double ypos) {
        auto renderer = static_cast<Renderer*>(glfwGetWindowUserPointer(window));
        if (renderer) {
            renderer->handleMouseMovement(xpos, ypos);
        }
    });
    glfwSetMouseButtonCallback(m_window.get(), [](GLFWwindow* window, int button, int action, int mods) {
        auto renderer = static_cast<Renderer*>(glfwGetWindowUserPointer(window));
        if (renderer) {
            renderer->handleMouseButton(button, action);
        }
    });

    m_dynamics = std::make_unique<Dynamics>();
    m_imguiLayer = std::make_unique<ImGuiLayer>(m_window.get(), m_renderer.get(), m_dynamics.get());

}

Application::~Application() {
    try {
        m_imguiLayer.reset();
        m_renderer.reset();

        if (m_window) {
            m_window.reset();
        }

        glfwTerminate();
    } catch (const std::exception& e) {
        std::cerr << "Exception during Application destruction: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "Unknown error during Application destruction" << std::endl;
    }
}

void Application::run() {
    double lastTime = glfwGetTime();

    auto body1 = std::make_shared<RigidBody>(10.0, Matrix3d::Identity() * 60.0, Vector3d(0.0, 0.0, 0.0), Vector4d(1.0, 0.0, 0.0, 0.0));
    auto body2 = std::make_shared<RigidBody>(10.0, Matrix3d::Identity() * 60.0, Vector3d(0.0, 0.0, 0.0), Vector4d(1.0, 0.0, 0.0, 0.0));
    auto body3 = std::make_shared<RigidBody>(10.0, Matrix3d::Identity() * 60.0, Vector3d(0.0, 0.0, 0.0), Vector4d(1.0, 0.0, 0.0, 0.0));
    auto body4 = std::make_shared<RigidBody>(10.0, Matrix3d::Identity() * 60.0, Vector3d(0.0, 0.0, 0.0), Vector4d(1.0, 0.0, 0.0, 0.0));
    auto body5 = std::make_shared<RigidBody>(10.0, Matrix3d::Identity() * 60.0, Vector3d(0.0, 0.0, 0.0), Vector4d(1.0, 0.0, 0.0, 0.0));
    auto body6 = std::make_shared<RigidBody>(10.0, Matrix3d::Identity() * 60.0, Vector3d(0.0, 0.0, 0.0), Vector4d(1.0, 0.0, 0.0, 0.0));
    auto body7 = std::make_shared<RigidBody>(10.0, Matrix3d::Identity() * 60.0, Vector3d(0.0, 0.0, 0.0), Vector4d(1.0, 0.0, 0.0, 0.0));
    auto body8 = std::make_shared<RigidBody>(10.0, Matrix3d::Identity() * 60.0, Vector3d(0.0, 0.0, 0.0), Vector4d(1.0, 0.0, 0.0, 0.0));
    auto body9 = std::make_shared<RigidBody>(10.0, Matrix3d::Identity() * 60.0, Vector3d(0.0, 0.0, 0.0), Vector4d(1.0, 0.0, 0.0, 0.0));
    auto body10 = std::make_shared<RigidBody>(10.0, Matrix3d::Identity() * 60.0, Vector3d(0.0, 0.0, 0.0), Vector4d(1.0, 0.0, 0.0, 0.0));
    auto body11 = std::make_shared<RigidBody>(10.0, Matrix3d::Identity() * 60.0, Vector3d(0.0, 0.0, 0.0), Vector4d(1.0, 0.0, 0.0, 0.0));
    auto body12 = std::make_shared<RigidBody>(10.0, Matrix3d::Identity() * 60.0, Vector3d(0.0, 0.0, 0.0), Vector4d(1.0, 0.0, 0.0, 0.0));


    m_dynamics->addBody(body1);
    m_dynamics->addBody(body2);
    m_dynamics->addBody(body3);
    m_dynamics->addBody(body4);
    m_dynamics->addBody(body5);
    m_dynamics->addBody(body6);
    m_dynamics->addBody(body7);
    m_dynamics->addBody(body8);
    m_dynamics->addBody(body9);
    m_dynamics->addBody(body10);
    m_dynamics->addBody(body11);
    m_dynamics->addBody(body12);


    while (!glfwWindowShouldClose(m_window.get())) {
        double currentTime = glfwGetTime();
        double deltaTime = currentTime - lastTime;
        lastTime = currentTime;

        processInput();

        // Step the physics simulation
        if (m_dynamics) {
             m_dynamics->step();
        }

        update();
        render();

        glfwSwapBuffers(m_window.get());
        glfwPollEvents();
    }
}

void Application::render() {
    m_renderer->clearScreen({0.1, 0.1, 0.1, 1.0});
    m_renderer->draw(m_dynamics.get());
    m_imguiLayer->renderUI();
}

void Application::update() {
    // Pass camera data to ImGui
    if (m_renderer && m_imguiLayer) {
        glm::dvec3 cameraPos = m_renderer->getCameraPosition();
        glm::dvec3 cameraOrientation = m_renderer->getCameraOrientation();
        double cameraSpeed = m_renderer->getCameraSpeed();
        m_imguiLayer->updateCameraData(cameraPos, cameraOrientation, cameraSpeed);
    }
}

void Application::captureScreenshot() {
    int width, height;
    glfwGetFramebufferSize(m_window.get(), &width, &height);

    // Allocate buffer for pixel data (RGBA format)
    std::vector<unsigned char> pixels(width * height * 4);

    // Read the pixels from the framebuffer
    glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());

    // Flip the image vertically (OpenGL's origin is bottom-left)
    for (int y = 0; y < height / 2; ++y) {
        for (int x = 0; x < width * 4; ++x) {
            std::swap(pixels[y * width * 4 + x], pixels[(height - y - 1) * width * 4 + x]);
        }
    }

    // Create the directory if it doesn't exist
    const std::string screenshotDir = "../assets/images";
    std::filesystem::create_directories(screenshotDir);

    // Generate a timestamped filename
    std::ostringstream filename;
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    filename << screenshotDir << "/screenshot_" << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".png";

    // Save the PNG
    if (stbi_write_png(filename.str().c_str(), width, height, 4, pixels.data(), width * 4)) {
        std::cout << "Screenshot saved to " << filename.str() << std::endl;
    } else {
        std::cerr << "Failed to save screenshot!" << std::endl;
    }
}

void Application::processInput() {
    static double lastFrameTime = 0.0;
    double currentFrameTime = glfwGetTime();
    double deltaTime = currentFrameTime - lastFrameTime;
    lastFrameTime = currentFrameTime;

    if (m_window && glfwGetKey(m_window.get(), GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(m_window.get(), true);
    }

    // Capture screenshot with F12
    if (glfwGetKey(m_window.get(), GLFW_KEY_F12) == GLFW_PRESS) {
        captureScreenshot();
    }

    if (m_renderer) {
        m_renderer->handleKeyboardInput(m_window.get(), deltaTime);
    }
}
