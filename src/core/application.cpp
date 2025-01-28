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


Application::Application(int width, int height, const char* title)
    : m_Window(nullptr, glfwDestroyWindow) {

    if (!glfwInit()) {
        throw std::runtime_error("Failed to initialize GLFW");
    }

    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
    m_Window.reset(glfwCreateWindow(width, height, title, nullptr, nullptr));

    if (!m_Window) {
        glfwTerminate();
        throw std::runtime_error("Failed to create GLFW window");
    }

    glfwMakeContextCurrent(m_Window.get());
    glfwSwapInterval(0); // Disable vsync


    m_Renderer = std::make_unique<Renderer>(m_Window.get());
    glfwSetWindowUserPointer(m_Window.get(), m_Renderer.get());

    // Register callbacks
    glfwSetCursorPosCallback(m_Window.get(), [](GLFWwindow* window, double xpos, double ypos) {
        auto renderer = static_cast<Renderer*>(glfwGetWindowUserPointer(window));
        if (renderer) {
            renderer->handleMouseMovement(xpos, ypos);
        }
    });

    glfwSetMouseButtonCallback(m_Window.get(), [](GLFWwindow* window, int button, int action, int mods) {
        auto renderer = static_cast<Renderer*>(glfwGetWindowUserPointer(window));
        if (renderer) {
            renderer->handleMouseButton(button, action);
        }
    });

    m_ImGuiLayer = std::make_unique<ImGuiLayer>(m_Window.get(), m_Renderer.get());
}


Application::~Application() {
    try {
        m_ImGuiLayer.reset();
        m_Renderer.reset();

        if (m_Window) {
            m_Window.reset();
        }

        glfwTerminate();
    } catch (const std::exception& e) {
        std::cerr << "Exception during Application destruction: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "Unknown error during Application destruction" << std::endl;
    }
}


void Application::run() {
    while (!glfwWindowShouldClose(m_Window.get())) {

        // Main loop tasks
        processInput();
        update();  // Use the actual elapsed time for simulation
        render();

        glfwSwapBuffers(m_Window.get());
        glfwPollEvents();
    }
}


// New method to capture and save a screenshot
void Application::captureScreenshot() {
    int width, height;
    glfwGetFramebufferSize(m_Window.get(), &width, &height);

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

    if (m_Window && glfwGetKey(m_Window.get(), GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(m_Window.get(), true);
    }

    // Capture screenshot with F12
    if (glfwGetKey(m_Window.get(), GLFW_KEY_F12) == GLFW_PRESS) {
        captureScreenshot();
    }

    if (m_Renderer) {
        m_Renderer->handleKeyboardInput(m_Window.get(), deltaTime);
    }
}


void Application::update() {
    // Pass camera data to ImGui
    if (m_Renderer && m_ImGuiLayer) {
        glm::dvec3 cameraPos = m_Renderer->getCameraPosition();
        glm::dvec3 cameraOrientation = m_Renderer->getCameraOrientation();
        double cameraSpeed = m_Renderer->getCameraSpeed();
        m_ImGuiLayer->updateCameraData(cameraPos, cameraOrientation, cameraSpeed);
    }
}


void Application::render() {
    m_Renderer->clearScreen({0.1, 0.1, 0.1, 1.0});
    m_Renderer->draw();
    m_ImGuiLayer->renderUI();
}

