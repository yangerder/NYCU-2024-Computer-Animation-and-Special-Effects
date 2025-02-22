#include <algorithm>
#include <cassert>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <GLFW/glfw3.h>
#define GLAD_GL_IMPLEMENTATION
#include <glad/gl.h>
#undef GLAD_GL_IMPLEMENTATION

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include "hw1.h"
int alignSize = 256;
bool isWindowSizeChanged = true;
bool mouseBinded = false;

int uboAlign(int i) { return ((i + 1 * (alignSize - 1)) / alignSize) * alignSize; }

// Control the pin of each corner
bool pin[4] = {true, true, true, true};
int cornerIndices[4] = {0, particlesPerEdge - 1, particlesPerEdge*(particlesPerEdge - 1),
                        particlesPerEdge* particlesPerEdge - 1};

// Velocity of the sphere
Eigen::Vector4f vel(0, 0, 0, 0);

void keyCallback(GLFWwindow* window, int key, int, int action, int) {
  // There are three actions: press, release, hold
  if (action != GLFW_PRESS) return;
  // Press ESC to close the window.
  if (key == GLFW_KEY_ESCAPE) {
    glfwSetWindowShouldClose(window, GLFW_TRUE);
    return;
  } else if (key == GLFW_KEY_F9) {
    // Disable / enable mouse cursor.
    if (mouseBinded)
      glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    else
      glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    mouseBinded = !mouseBinded;
  } else if (key == GLFW_KEY_1) {
    pin[0] = !pin[0];
  } else if (key == GLFW_KEY_2) {
    pin[1] = !pin[1];
  } else if (key == GLFW_KEY_3) {
    pin[2] = !pin[2];
  } else if (key == GLFW_KEY_4) {
    pin[3] = !pin[3];
  } else if (key == GLFW_KEY_LEFT) {
    vel[0] = 5.0f;
  } else if (key == GLFW_KEY_RIGHT) {
    vel[0] = -5.0f;
  } else if (key == GLFW_KEY_UP) {
    vel[2] = 5.0f;
  } else if (key == GLFW_KEY_DOWN) {
    vel[2] = -5.0f;
  } else if (key == GLFW_KEY_SPACE) {
    vel = Eigen::Vector4f::Zero();
  }
}

void framebufferSizeCallback(GLFWwindow*, int width, int height) {
  // Minimize event guard
  if (width == 0 && height == 0) return;
  windowWidth = width;
  windowHeight = height;
  glViewport(0, 0, width, height);
  isWindowSizeChanged = true;
}

void loadTexture(unsigned int& texture, const char* fileName) {
  int width, height, nChannels;
  stbi_uc* data = stbi_load(fileName, &width, &height, &nChannels, 0);
  if (data == nullptr) {
    std::cerr << fileName << " not found!" << std::endl;
    return;
  }
  int colorFormat = (nChannels == 4) ? GL_RGBA : GL_RGB;
  glActiveTexture(GL_TEXTURE0);
  glGenTextures(1, &texture);
  glBindTexture(GL_TEXTURE_2D, texture);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
  glTexImage2D(GL_TEXTURE_2D, 0, colorFormat, width, height, 0, colorFormat, GL_UNSIGNED_BYTE, data);
  glGenerateMipmap(GL_TEXTURE_2D);
  stbi_image_free(data);
}

int main() {
  // Initialize OpenGL context.
  OpenGLContext& context = OpenGLContext::getContext();
  // TODO: change the title to your student ID
  GLFWwindow* window = context.createWindow("HW1 111550149", 1280, 720, GLFW_OPENGL_CORE_PROFILE);
  glfwSetKeyCallback(window, keyCallback);
  glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);

#ifndef NDEBUG
  context.printSystemInfo();
  context.enableDebugCallback();
#endif

  glfwSwapInterval(1);
  glfwGetFramebufferSize(window, &windowWidth, &windowHeight);
  glViewport(0, 0, windowWidth, windowHeight);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_CULL_FACE);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glClearColor(0, 0, 0, 1);
  glGetIntegerv(GL_UNIFORM_BUFFER_OFFSET_ALIGNMENT, &alignSize);

  speedMultiplier = (240 / context.getRefreshRate());
  simulationPerFrame *= speedMultiplier;
  GUI gui(window, context.getOpenGLVersion());
  // Initialize shaders
  ShaderProgram sphereRenderer, particleRenderer;
  {
    VertexShader vs;
    FragmentShader fs;
    vs.fromFile(findPath("sphere.vert"));
    fs.fromFile(findPath("sphere.frag"));
    sphereRenderer.attach(&vs, &fs);
    sphereRenderer.link();
    sphereRenderer.detach(&vs, &fs);
    sphereRenderer.use();
    sphereRenderer.setUniform("color", sphereColor);
    sphereRenderer.uniformBlockBinding("model", 0);
    sphereRenderer.uniformBlockBinding("camera", 1);
  }
  {
    VertexShader vs;
    FragmentShader fs;
    vs.fromFile(findPath("particle.vert"));
    fs.fromFile(findPath("particle.frag"));
    particleRenderer.attach(&vs, &fs);
    particleRenderer.link();
    particleRenderer.detach(&vs, &fs);
    particleRenderer.use();

    particleRenderer.uniformBlockBinding("model", 0);
    particleRenderer.uniformBlockBinding("camera", 1);
  }

  // Create softbody
  Cloth cloth;
  cloth.computeNormal();
  UniformBuffer meshUBO;
  int meshOffset = uboAlign(32 * sizeof(GLfloat));
  meshUBO.allocate(2 * meshOffset);
  meshUBO.load(0, 16 * sizeof(GLfloat), cloth.getModelMatrix().data());
  meshUBO.load(16 * sizeof(GLfloat), 16 * sizeof(GLfloat), cloth.getNormalMatrix().data());
  unsigned int texture;
  loadTexture(texture, findPath("Textures/fabric.png").string().c_str());

  Spheres& spheres = Spheres::initSpheres();
  spheres.addSphere(Eigen::Vector4f(0, 0, 0, 1), 1.0f);
  meshUBO.load(meshOffset, 16 * sizeof(GLfloat), spheres.getModelMatrix().data());
  meshUBO.load(meshOffset + 16 * sizeof(GLfloat), 16 * sizeof(GLfloat), spheres.getNormalMatrix().data());

  Camera camera(Eigen::Vector4f(0, 2, -10, 1));
  UniformBuffer cameraUBO;
  cameraUBO.allocate(uboAlign(20 * sizeof(GLfloat)));
  cameraUBO.load(0, 16 * sizeof(GLfloat), camera.viewProjectionMatrix().data());
  cameraUBO.load(16 * sizeof(GLfloat), 4 * sizeof(GLfloat), camera.position().data());
  cameraUBO.bindUniformBlockIndex(1, 0, uboAlign(20 * sizeof(GLfloat)));
  // Do one step simulation, used in some implicit methods
  std::function<void(void)> simulateOneStep = [&]() {
    cloth.computeExternalForce();
    cloth.computeSpringForce();
    spheres.collide(&cloth);
  };

  ExplicitEuler explicitEuler;
  ImplicitEuler implicitEuler;
  MidpointEuler midpointEuler;
  RungeKuttaFourth rk4;
  Integrator* integrator = &explicitEuler;

  std::vector<Particles*> particles{&cloth.particles(), &spheres.particles()};
  // Backup initial state
  Particles initialCloth = cloth.particles();
  Particles initialSpheres = spheres.particles();

  while (!glfwWindowShouldClose(window)) {
    // Polling events.
    glfwPollEvents();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    bool cameraChanged = mouseBinded ? camera.move(window) : false;
    if (isWindowSizeChanged) {
      isWindowSizeChanged = false;
      camera.updateProjection();
      cameraChanged = true;
    }
    if (cameraChanged) {
      cameraUBO.load(0, 16 * sizeof(GLfloat), camera.viewProjectionMatrix().data());
      cameraUBO.load(16 * sizeof(GLfloat), 4 * sizeof(GLfloat), camera.position().data());
    }
    // Check which integrator is selected in GUI.
    switch (currentIntegrator) {
      case 0: integrator = &explicitEuler; break;
      case 1: integrator = &implicitEuler; break;
      case 2: integrator = &midpointEuler; break;
      case 3: integrator = &rk4; break;
      default: break;
    }

    // Fix corners
    for (int i = 0; i < 4; i++) {
      int idx = cornerIndices[i];
      if (pin[i]) {
        cloth.particles().mass(idx) = 0.0f;
        cloth.particles().velocity(idx).setZero();
        cloth.particles().acceleration(idx).setZero();
      } else {
        cloth.particles().mass(idx) = 1.0f;
      }
    }

    // Set velocity of the sphere
    spheres.setVelocity(0, vel);

    if (!isPaused) {
      // Stop -> Start: Restore initial state
      if (isStateSwitched) {
        cloth.particles() = initialCloth;
        spheres.particles() = initialSpheres;
      }
      // Simulate one step and then integrate it.
      for (int i = 0; i < simulationPerFrame; i++) {
        simulateOneStep();
        integrator->integrate(particles, simulateOneStep);
      }
    }

    particleRenderer.use();
    meshUBO.bindUniformBlockIndex(0, 0, meshOffset);
    if (isDrawingStructuralSprings) {
      particleRenderer.setUniform("color", Eigen::Vector4f(0, 1, 1, 1));
      cloth.draw(Cloth::DrawType::STRUCTURAL);
    }
    if (isDrawingShearSprings) {
      particleRenderer.setUniform("color", Eigen::Vector4f(1, 0, 1, 1));
      cloth.draw(Cloth::DrawType::SHEAR);
    }
    if (isDrawingBendSprings) {
      particleRenderer.setUniform("color", Eigen::Vector4f(1, 1, 0, 1));
      cloth.draw(Cloth::DrawType::BEND);
    }
    if (isDrawingCloth) {
      glDisable(GL_CULL_FACE);
      // This is very slow because it is done in CPU. Since GL4.1 doesn't support compute shader.
      cloth.computeNormal();
      particleRenderer.setUniform("isSurface", 1);
      particleRenderer.setUniform("useTexture", 1);
      particleRenderer.setUniform("diffuseTexture", 0);
      cloth.draw(Cloth::DrawType::FULL);
      particleRenderer.setUniform("useTexture", 0);
      glEnable(GL_CULL_FACE);
    } else {
      particleRenderer.setUniform("isSurface", 0);
      particleRenderer.setUniform("color", Eigen::Vector4f(1, 0, 0, 1));
      cloth.draw(Cloth::DrawType::PARTICLE);
    }

    sphereRenderer.use();
    if (isSphereColorChange) sphereRenderer.setUniform("color", sphereColor);
    meshUBO.bindUniformBlockIndex(0, meshOffset, meshOffset);
    spheres.draw();

    gui.render();
#ifdef __APPLE__
    glFlush();
#endif
    glfwSwapBuffers(window);
  }
  glfwDestroyWindow(window);
  return 0;
}
