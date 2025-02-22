/*
Computer animation assignment 3: Motion Graph

First step: Search TODO comments to find the methods that you need to implement.
    - src/simulation/kinematics.cpp
*/
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>

#include "Eigen/Core"
#include "GLFW/glfw3.h"
#define GLAD_GL_IMPLEMENTATION
#include "glad/gl.h"
#undef GLAD_GL_IMPLEMENTATION

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "acclaim.h"
#include "graphics.h"
#include "icons.h"
#include "simulation.h"
#include "util.h"
#include "acclaim/motion_graph.h"
using namespace acclaim;


// Global variables are evil!
namespace {
// Shadow texture size, default is 2048 * 2048
int shadowTextureSize = 2048;
// Scene size
int g_ScreenWidth = 1024, g_ScreenHeight = 768;
// Camera controlled by panel
graphics::DefaultCamera defaultCamera;
// Camera controlled by keyboard and mouse
graphics::FreeCamera freeCamera;
// Current camera
graphics::Camera* currentCamera = &defaultCamera;

// Switch for render camera control panel
bool isUsingCameraPanel = false;
// Is free camera?
bool isUsingFreeCamera = false;
// Mouse is disabled?
bool isMouseBinded = false;
// Animation Selection
const char* animations[] = {"walk_fwd_circle.amc", "walk_fwd_curve1.amc", "walk_fwd_curve2.amc"};
static char* current_animations[] = {(char*)animations[0], (char*)animations[1], (char*)animations[2]};
bool changed[] = {false, false, false};
// Frame Control
std::vector<int> animationFrames;
int currentFrame = 0;
int curSegIndex = 0;
int nextSegIndex = 1;
bool needUpdateAniation = false;
bool isPlaying = true;

// Motion Graph
int segmentSize = 200;
int blendingFrame = 50;
int threshold = 185.0;
bool needRegenerate = false;

// IK "root" bone
int start_bone = 11;
// Fonts' range
constexpr const ImWchar icon_ranges[] = {ICON_MIN, ICON_MAX, 0};
}// namespace

/**
 * @brief When resizing window, we need to update viewport and camera's aspect
 * ratio.
 * @param window current GLFW context
 * @param screenWidth Screen width after resize.
 * @param screenHeight Screen height after resize.
 */
void reshape(GLFWwindow* window, int screenWidth, int screenHeight);

/**
 * @brief Initialize globals and OpenGL
 *
 * @return `GLFWwindow*` The current GLFW context window
 */
GLFWwindow* initialize();
/**
 * @brief Callback function for the debug camera.
 */
void cameraKeyboard(GLFWwindow* window, int key, int scancode, int action, int mode);
/**
 * @brief Reset pointer of std::unique_ptr to call destructor before calling
 * glfwTerminate() to avoid segfault since some destructor contains OpenGL
 * functions.
 *
 * @param window The context to be destroyed
 */
void shutdown();
/**
 * @brief Dear-ImGui main control panel
 *
 */
void mainPanel();
/**
 * @brief Dear-ImGui camera control panel
 *
 * @param window: Current context
 */
void cameraPanel(GLFWwindow* window);
/**
 * @brief Put panels between ImGui::NewFrame() and ImGui::Render() to render
 *
 * @param window: Current context
 */
void renderUI(GLFWwindow* window);

int main() {
    GLFWwindow* window = initialize();
    // No window created
    if (window == nullptr) return 1;
    // Shader programs
    graphics::Program renderProgram;
    graphics::Program skyboxRenderProgram;
    graphics::Program shadowProgram;
    // Texture for shadow mapping
    graphics::ShadowMapTexture shadow(shadowTextureSize);
    // The skybox
    graphics::Box skybox;
    // Ground
    graphics::Plane plane;
    // Animation
    auto acclaim_folder = util::PathFinder::find("Acclaim");
    auto fkskeleton = std::make_unique<acclaim::Skeleton>(acclaim_folder / "walker.asf", 0.2);
    acclaim::Motion firstAnimation =
        acclaim::Motion(acclaim_folder / "walk_fwd_circle.amc",std::make_unique<acclaim::Skeleton>(*fkskeleton));
    acclaim::Motion secondAnimation = 
        acclaim::Motion(acclaim_folder / "walk_fwd_curve1.amc", std::make_unique<acclaim::Skeleton>(*fkskeleton));
    acclaim::Motion thirdAnimation =
        acclaim::Motion(acclaim_folder / "walk_fwd_curve2.amc", std::make_unique<acclaim::Skeleton>(*fkskeleton));
    animationFrames.push_back(firstAnimation.getFrameNum());
    animationFrames.push_back(secondAnimation.getFrameNum());
    animationFrames.push_back(thirdAnimation.getFrameNum());
    std::vector<Motion> tmps;
    MotionGraph* motionGraph = new MotionGraph(std::vector<Motion>{firstAnimation, secondAnimation, thirdAnimation}, segmentSize, blendingFrame, threshold);
    motionGraph->constructGraph();
    motionGraph->traverse();
    // Load assets, setup textures
    {
        // Shader
        auto shader_folder = util::PathFinder::find("Shader");
        graphics::Shader shadowVertexShader(shader_folder / "shadow.vert", GL_VERTEX_SHADER);
        graphics::Shader shadowFragmentShader(shader_folder / "shadow.frag", GL_FRAGMENT_SHADER);
        graphics::Shader renderVertexShader(shader_folder / "render.vert", GL_VERTEX_SHADER);
        graphics::Shader renderFragmentShader(shader_folder / "render.frag", GL_FRAGMENT_SHADER);
        graphics::Shader skyboxVertexShader(shader_folder / "skybox.vert", GL_VERTEX_SHADER);
        graphics::Shader skyboxFragmentShader(shader_folder / "skybox.frag", GL_FRAGMENT_SHADER);
        // Setup shaders, these objects can be destroyed after linkShader()
        renderProgram.attachLinkShader(renderVertexShader, renderFragmentShader);
        shadowProgram.attachLinkShader(shadowVertexShader, shadowFragmentShader);
        skyboxRenderProgram.attachLinkShader(skyboxVertexShader, skyboxFragmentShader);
        // Texture
        auto texture_folder = util::PathFinder::find("Texture");
        auto wood = std::make_shared<graphics::Texture>(texture_folder / "wood.png");
        plane.setTexture(std::move(wood));
        auto sky = std::make_shared<graphics::CubeTexture>(std::array<util::fs::path, 6>{
            texture_folder / "ocean0.png", texture_folder / "ocean1.png", texture_folder / "ocean2.png",
            texture_folder / "ocean3.png", texture_folder / "ocean4.png", texture_folder / "ocean5.png"});
        skybox.setTexture(std::move(sky));
    }

    // Setup light, uniforms are persisted.
    {
        Eigen::Vector4f lightPosition(11.1f, 24.9f, -14.8f, 0.0f);
        Eigen::Matrix4f lightSpaceMatrix = util::ortho(-30.0f, 30.0f, -30.0f, 30.0f, -75.0f, 75.0f);
        lightSpaceMatrix *= util::lookAt(lightPosition, Eigen::Vector4f::Zero(), Eigen::Vector4f::UnitY());
        // Shader program should be use atleast once before setting up uniforms
        shadowProgram.use();
        shadowProgram.setUniform("lightSpaceMatrix", lightSpaceMatrix);

        renderProgram.use();
        renderProgram.setUniform("lightSpaceMatrix", lightSpaceMatrix);
        renderProgram.setUniform("shadowMap", shadow.getIndex());
        renderProgram.setUniform("lightPos", lightPosition);
    }
    while (!glfwWindowShouldClose(window)) {
        // Moving camera only if debug camera is on.
        if (isUsingFreeCamera) {
            if (isMouseBinded) freeCamera.moveSight(window);
            freeCamera.moveCamera(window);
        }
        currentCamera->update();
        if (changed[0]) {
            firstAnimation = acclaim::Motion(acclaim_folder / current_animations[0],
                                             std::make_unique<acclaim::Skeleton>(*fkskeleton));
            animationFrames[0] = firstAnimation.getFrameNum();
        }
        if (changed[1]) {
            secondAnimation = acclaim::Motion(acclaim_folder / current_animations[1],
                                              std::make_unique<acclaim::Skeleton>(*fkskeleton));
            animationFrames[1] = secondAnimation.getFrameNum();
        }
        if (changed[2]) {
            thirdAnimation = acclaim::Motion(acclaim_folder / current_animations[2],
                                             std::make_unique<acclaim::Skeleton>(*fkskeleton));
            animationFrames[2] = thirdAnimation.getFrameNum();
        }
        if (needUpdateAniation)
        {
            currentFrame = 0;
            needUpdateAniation = false;
            isPlaying = true;
            motionGraph = new MotionGraph(std::vector<Motion>{firstAnimation, secondAnimation, thirdAnimation},
                                          segmentSize, blendingFrame, threshold);
            motionGraph->constructGraph();
            motionGraph->traverse();
            curSegIndex = motionGraph->currIdx;
            nextSegIndex = motionGraph->nextIdx;
        }
        for (int i = 0;i<3;i++)
            changed[i] = false;
        
        // Regenerate Matrix
        if (needRegenerate)
        {
            motionGraph = new MotionGraph(std::vector<Motion>{firstAnimation, secondAnimation, thirdAnimation},
                                          segmentSize, blendingFrame, threshold);
            motionGraph->constructGraph();
            motionGraph->traverse();
            curSegIndex = motionGraph->currIdx;
            nextSegIndex = motionGraph->nextIdx;
            // segmentSize + blendingFrame + threshold;
            needRegenerate = false;
        }
        
        // Call to decide next Frame
        if (isPlaying) {
            ++currentFrame;
            if (currentFrame >= motionGraph->currSegment.getFrameNum()) {
                motionGraph->traverse();
                currentFrame = 0;
                curSegIndex = motionGraph->currIdx;
                nextSegIndex = motionGraph->nextIdx;
            }
            motionGraph->currSegment.forwardkinematics(currentFrame);
            // Current Frame update
        }
        /*if (currentFrame < firstAnimation.getFrameNum())
            firstAnimation.forwardkinematics(currentFrame);
        else if (currentFrame < firstAnimation.getFrameNum() + secondAnimation.getFrameNum())
            secondAnimation.forwardkinematics(currentFrame - firstAnimation.getFrameNum());
        else
            thirdAnimation.forwardkinematics(currentFrame - firstAnimation.getFrameNum() - secondAnimation.getFrameNum());*/

        // 1. Render shadow to texture
        glViewport(0, 0, shadow.getShadowSize(), shadow.getShadowSize());
        glCullFace(GL_FRONT);
        shadowProgram.use();
        shadow.bindFrameBuffer();
        glClear(GL_DEPTH_BUFFER_BIT);
        plane.render(&shadowProgram);
        motionGraph->currSegment.getSkeleton()->render(&shadowProgram);
        /*if (currentFrame < firstAnimation.getFrameNum())
            firstAnimation.getSkeleton()->render(&shadowProgram);
        else if (currentFrame < firstAnimation.getFrameNum() + secondAnimation.getFrameNum())
            secondAnimation.getSkeleton()->render(&shadowProgram);
        else
            thirdAnimation.getSkeleton()->render(&shadowProgram);*/
        shadow.unbindFrameBuffer();
        glCullFace(GL_BACK);
        // 2. Render scene
        glViewport(0, 0, g_ScreenWidth, g_ScreenHeight);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        renderProgram.use();
        renderProgram.setUniform("viewPos", currentCamera->getPosition());
        renderProgram.setUniform("VP", currentCamera->getViewWithProjectionMatrix());

        plane.render(&renderProgram);
        motionGraph->currSegment.getSkeleton()->render(&renderProgram);

        /*if (currentFrame < firstAnimation.getFrameNum())
            firstAnimation.getSkeleton()->render(&renderProgram);
        else if (currentFrame < firstAnimation.getFrameNum() + secondAnimation.getFrameNum())
            secondAnimation.getSkeleton()->render(&renderProgram);
        else
            thirdAnimation.getSkeleton()->render(&renderProgram);*/
        
        // 3. Render the skybox .
        skyboxRenderProgram.use();
        skyboxRenderProgram.setUniform("projection", currentCamera->getProjectionMatrix());
        skyboxRenderProgram.setUniform("view", currentCamera->getViewMatrix());
        skybox.render(&skyboxRenderProgram);
        // 4. Render ImGui UI
        renderUI(window);


        glFlush();
        glfwSwapBuffers(window);
        // Keyboard and mouse inputs.
        glfwPollEvents();
    }
    shutdown();
    glfwDestroyWindow(window);
    return 0;
}

void reshape(GLFWwindow*, int screenWidth, int screenHeight) {
    g_ScreenWidth = screenWidth;
    g_ScreenHeight = screenHeight;
    glViewport(0, 0, g_ScreenWidth, g_ScreenHeight);
    defaultCamera.setAspectRatio(g_ScreenWidth, g_ScreenHeight);
    freeCamera.setAspectRatio(g_ScreenWidth, g_ScreenHeight);
}

GLFWwindow* initialize() {
    // Initialize GLFW
    if (glfwInit() == GLFW_FALSE) {
        std::cerr << "Failed to initialize GLFW!" << std::endl;
        return nullptr;
    }
    std::atexit(glfwTerminate);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    // Create GLFW context
    GLFWwindow* window = glfwCreateWindow(g_ScreenWidth, g_ScreenHeight, "111550149", nullptr, nullptr);
    if (window == nullptr) {
        std::cerr << "Failed to create OpenGL 4.1 window!" << std::endl;
        return nullptr;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    // Initialize glad
    if (!gladLoadGL(glfwGetProcAddress)) {
        std::cerr << "Failed to initialize OpenGL context" << std::endl;
        return nullptr;
    }
    // Find assets folder
    if (!util::PathFinder::initialize()) {
        std::cerr << "Cannot find assets!" << std::endl;
        return nullptr;
    }
    // OK, everything works fine
    // ----------------------------------------------------------
    // For high dpi monitors
    glfwGetFramebufferSize(window, &g_ScreenWidth, &g_ScreenHeight);
    GLFWmonitor* moniter = glfwGetPrimaryMonitor();
    const GLFWvidmode* vidMode = glfwGetVideoMode(moniter);
    int maxTextureSize = 1024;
    glGetIntegerv(GL_MAX_TEXTURE_SIZE, &maxTextureSize);
    shadowTextureSize = std::min(shadowTextureSize, maxTextureSize);
    // Print some system information
    std::cout << std::left << std::setw(26) << "Current OpenGL renderer"
              << ": " << glGetString(GL_RENDERER) << std::endl;
    std::cout << std::left << std::setw(26) << "Current OpenGL context"
              << ": " << glGetString(GL_VERSION) << std::endl;
    std::cout << std::left << std::setw(26) << "Moniter refresh rate"
              << ": " << vidMode->refreshRate << " Hz" << std::endl;
    std::cout << std::left << std::setw(26) << "Max texture size support"
              << ": " << maxTextureSize << " * " << maxTextureSize << std::endl;
    std::cout << std::left << std::setw(26) << "Shadow texture size"
              << ": " << shadowTextureSize << " * " << shadowTextureSize << std::endl;
    // Setup Opengl
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glCullFace(GL_BACK);
    glFrontFace(GL_CCW);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    // Setup GLFW
    reshape(window, g_ScreenWidth, g_ScreenHeight);
    glfwSetFramebufferSizeCallback(window, reshape);
    // Initialize dear-ImGui
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 410 core");

    // Setup Icon fonts
    ImGuiIO& io = ImGui::GetIO();
    io.Fonts->AddFontDefault();
    ImFontConfig config;
    config.MergeMode = true;
    config.PixelSnapH = true;
    config.GlyphMinAdvanceX = 13.0f;  // monospaced
    // This is a temporary solution
    void* temp = std::malloc(sizeof(forkawesome));
    std::memcpy(temp, forkawesome, sizeof(forkawesome));
    io.Fonts->AddFontFromMemoryTTF(temp, 1, 13.0f, &config, icon_ranges);

    return window;
}

void cameraKeyboard(GLFWwindow* window, int key, int, int action, int) {
    if (action == GLFW_PRESS && key == GLFW_KEY_F9) {
        if (glfwGetInputMode(window, GLFW_CURSOR) == GLFW_CURSOR_DISABLED) {
            // Show the mouse cursor
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
            isMouseBinded = false;
        } else {
            // Reset delta x and delta y to avoid view teleporting
            freeCamera.reset();
            // Hide the mouse cursor
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
            isMouseBinded = true;
        }
    }
}

void shutdown() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}

void mainPanel() {
    // Main Panel
    ImGui::SetNextWindowSize(ImVec2(800.0f, 140.0f), ImGuiCond_Once);
    ImGui::SetNextWindowCollapsed(0, ImGuiCond_Once);
    ImGui::SetNextWindowPos(ImVec2(60.0f, 550.0f), ImGuiCond_Once);
    ImGui::SetNextWindowBgAlpha(0.2f);

    if (ImGui::Begin("Motion Graph")) {
        // x ^= 1 === x = !x
        if (ImGui::Button("Camera Panel")) isUsingCameraPanel ^= true;
        /*
            ImGui::SameLine();
        ImGui::SetNextItemWidth(100);
        if (ImGui::SliderInt("Segment Size", &segmentSize, 20, 50))
            // chageSegemnt
            needRegenerate = true;
            ImGui::SameLine();
        ImGui::SetNextItemWidth(100);
        if (ImGui::SliderInt("Blending Frame Size", &blendingFrame, 8, segmentSize)) needRegenerate = true;
            ImGui::SameLine();
        ImGui::SetNextItemWidth(100);
        if (ImGui::SliderInt("Threshold", &threshold, 1, 10)) needRegenerate = true;
        */
        int accumulatedFrame = 0;
        for (int i = 0; i < 3; i++) {
            std::string animationName = "Animation " + std::to_string(i);
            ImGui::SetNextItemWidth(200);
            if (ImGui::BeginCombo(animationName.c_str(), current_animations[i])) {
                for (int n = 0; n < IM_ARRAYSIZE(animations); n++) {
                    bool is_selected =
                        (current_animations[i] == animations[n]);  // You can store your selection however you
                                                                   // want, outside or inside your objects
                    if (ImGui::Selectable(animations[n], is_selected)) {
                        current_animations[i] = (char*)animations[n];
                        changed[i] = true;
                        needUpdateAniation = true;
                    }
                    if (is_selected)
                        ImGui::SetItemDefaultFocus();  // You may set the initial focus when opening the combo
                                                       // (scrolling + for keyboard navigation support)
                }
                ImGui::EndCombo();
            }
            // Show Frame
            ImVec4 lightBlueColor = ImVec4(0.6f, 0.8f, 1.0f, 1.0f);  // Current
            ImVec4 darkBlueColor = ImVec4(0.0f, 0.0f, 0.5f, 1.0f);   // Next
            ImVec4 darkGrayColor = ImVec4(0.2f, 0.2f, 0.2f, 1.0f);   // Remaings
            for (int j = accumulatedFrame; j < accumulatedFrame + animationFrames[i] + segmentSize - 1;
                 j += segmentSize) {
                ImGui::SameLine();
                if (j/segmentSize == nextSegIndex)
                    ImGui::PushStyleColor(ImGuiCol_Button, darkBlueColor);
                else if (j / segmentSize == curSegIndex)
                    ImGui::PushStyleColor(ImGuiCol_Button, lightBlueColor);
                else
                    ImGui::PushStyleColor(ImGuiCol_Button, darkGrayColor);
                ImGui::Button(" ");
                ImGui::PopStyleColor();
            }
            accumulatedFrame += animationFrames[i];
        }  // Simulation Control Panel is disabled now

        if (ImGui::Button(ICON_PLAY)) {
            isPlaying = true;
        }
        ImGui::SameLine();
        if (ImGui::Button(ICON_PAUSE)) {
            isPlaying = false;
        }
        ImGui::SameLine();
        if (ImGui::Button(ICON_STOP)) {
            isPlaying = false;
            currentFrame = 0;
        }
    }
    ImGui::End();
}
void cameraPanel(GLFWwindow* window) {
    // Camera control
    // Create imgui window
    ImGui::SetNextWindowSize(ImVec2(500.0f, 210.0f), ImGuiCond_Once);
    ImGui::SetNextWindowCollapsed(0, ImGuiCond_Once);
    ImGui::SetNextWindowPos(ImVec2(370.0f, 950.0f), ImGuiCond_Once);
    ImGui::SetNextWindowBgAlpha(0.2f);
    if (ImGui::Begin("Camera", &isUsingCameraPanel)) {
        if (!isUsingFreeCamera) ImGui::Text("Use this panel to control the camera");
        Eigen::Vector4f cameraPosition = currentCamera->getPosition();
        ImGui::Text("Camera position : (%f, %f, %f)", cameraPosition[0], cameraPosition[1], cameraPosition[2]);
        Eigen::Vector4f cameraCenter = currentCamera->getCenter();
        ImGui::Text("Camera lookat : (%f, %f, %f)", cameraCenter[0], cameraCenter[1], cameraCenter[2]);
        if (!isUsingFreeCamera) {
            ImGui::SliderFloat("Camera rotation angle", defaultCamera.getCameraRotationAnglePointer(), 0.0f, 360.0f);
            ImGui::SliderFloat("Camera rotation radius", defaultCamera.getCameraRotationRadiusPointer(), 0.125f, 50.0f);
            ImGui::SliderFloat("Camera Y Offset", defaultCamera.getCameraYOffsetPointer(), -10.0f, 10.0f);
            if (ImGui::Button("Debug Mode")) {
                isUsingFreeCamera = true;
                currentCamera = &freeCamera;
                glfwSetKeyCallback(window, cameraKeyboard);
            }
        } else {
            ImGui::InputFloat("Mouse sensitivity", freeCamera.getMouseSensitivityPointer(), 0.01f, 0.05f);
            ImGui::InputFloat("Move speed", freeCamera.getMoveSpeedPointer(), 0.01f, 0.05f);
            ImGui::Text("Use W A S D CTRL SPACE to move");
            ImGui::Text("Press F9 to bind / unbind mouse");
            ImGui::Text("Bind mouse to control view");
            if (ImGui::Button("Leave debug Mode")) {
                isUsingFreeCamera = false;
                currentCamera = &defaultCamera;
                //glfwSetCursorPosCallback(window, nullptr);
                //glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
            }
        }
    }
    ImGui::End();
}

void renderUI(GLFWwindow* window) {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    mainPanel();
    if (isUsingCameraPanel) cameraPanel(window);
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

Eigen::Vector3f screenToWorldRay(GLFWwindow* window, double xpos, double ypos) {
    int screenWidth, screenHeight;
    glfwGetFramebufferSize(window, &screenWidth, &screenHeight);

    double x_ndc = (2.0 * xpos) / screenWidth - 1.0;
    double y_ndc = 1.0 - (2.0 * ypos) / screenHeight;

    Eigen::Matrix4f viewProjectionMatrix = currentCamera->getViewWithProjectionMatrix();

    Eigen::Vector4f viewportPos(x_ndc, y_ndc, -1.0, 1.0);
    Eigen::Vector4f worldPos = viewProjectionMatrix.inverse() * viewportPos;
    worldPos /= worldPos[3];

    Eigen::Vector3f cameraPos = currentCamera->getPosition().head<3>();
    Eigen::Vector3f rayDirection = (worldPos.head<3>() - cameraPos).normalized();

    return rayDirection;
}

Eigen::Vector4d calculateIntersectionPoint(const Eigen::Vector3f& rayOrigin, const Eigen::Vector3f& rayDirection,
                                           const Eigen::Vector3f& planeNormal, float planeDistance) {
    float denominator = rayDirection.dot(planeNormal);
    if (denominator == 0.0f) {
        return Eigen::Vector4d::Zero();
    } else {
        float t = -(rayOrigin.dot(planeNormal) + planeDistance) / denominator;
        return Eigen::Vector4d(rayOrigin.x() + t * rayDirection.x(), rayOrigin.y() + t * rayDirection.y(),
                               rayOrigin.z() + t * rayDirection.z(), 0.0);
    }
}
