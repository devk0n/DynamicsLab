#include <iostream>

#include "core/Application.h"

int main() {

  Application app;
  if (!app.initialize()) {
    return EXIT_FAILURE;
  }

  app.run();

  return EXIT_SUCCESS;
}