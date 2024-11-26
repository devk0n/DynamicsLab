#include <vulkan/vulkan.h>
#include <iostream>

int main() {
    VkInstance instance;

    // Vulkan instance creation info
    VkInstanceCreateInfo createInfo{};
    createInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;

    // Create Vulkan instance
    if (vkCreateInstance(&createInfo, nullptr, &instance) != VK_SUCCESS) {
        std::cerr << "Failed to create Vulkan instance!" << std::endl;
        return -1;
    }

    std::cout << "Vulkan instance created successfully!" << std::endl;

    // Clean up
    vkDestroyInstance(instance, nullptr);
    return 0;
}
