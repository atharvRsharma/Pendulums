#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>
#include <vector>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

const unsigned int h = 800, w = 800;

float G = 9.81f;
float PENDULUM_RADIUS = 0.04f;
float INITIAL_LENGTH = 0.7f;
float INITIAL_MASS = 1.0f;
int PATH_LIMIT = 2000;

float dt = 0.01f;

const char* vertexShaderSource = R"(
#version 330 core
layout(location = 0) in vec2 aPos;
uniform mat4 projection;
void main()
{
    gl_Position = projection * vec4(aPos, 0.0, 1.0);
}
)";

const char* fragmentShaderSource = R"(
#version 330 core
out vec4 FragColor;
void main()
{
    FragColor = vec4(1.0, 0.0, 0.0, 1.0);
}
)";

glm::mat4 projection;
std::vector<float> pathVertices;
std::vector<glm::vec2> pendulums;
std::vector<float> theta;
std::vector<float> omega;

void initialize() {
    glViewport(0, 0, w, h);
    projection = glm::ortho(-2.0f, 2.0f, -2.0f, 2.0f, -1.0f, 1.0f);

    pendulums.push_back(glm::vec2(INITIAL_LENGTH, INITIAL_MASS));
    theta.push_back(M_PI / 1.0f);
    omega.push_back(0.0f);

    omega[0] = 0.5f;
}

void computePhysics() {
    if (pendulums.size() < 2) {
        float L1 = pendulums[0].x;
        float M1 = pendulums[0].y;

        float a1 = (-G / L1) * sin(theta[0]);

        omega[0] += a1 * dt;
        theta[0] += omega[0] * dt;

        float baseX = 0.0f;
        float baseY = 0.5f;
        float x = baseX + L1 * sin(theta[0]);
        float y = baseY - L1 * cos(theta[0]);

        if (pathVertices.size() >= PATH_LIMIT * 2) {
            pathVertices.erase(pathVertices.begin(), pathVertices.begin() + 2);
        }
        pathVertices.push_back(x);
        pathVertices.push_back(y);
    }
    else {
        for (size_t i = 0; i < pendulums.size(); ++i) {
            float L1 = pendulums[i].x;
            float M1 = pendulums[i].y;
            float L2 = (i + 1 < pendulums.size()) ? pendulums[i + 1].x : 0.0f;
            float M2 = (i + 1 < pendulums.size()) ? pendulums[i + 1].y : 0.0f;

            if (i + 1 < pendulums.size()) {
                float deltaTheta = theta[i + 1] - theta[i];
                float denom1 = (M1 + M2) * L1 - M2 * L1 * cos(deltaTheta) * cos(deltaTheta);
                float denom2 = (L2 / L1) * denom1;

                float a1 = (M2 * L1 * omega[i] * omega[i] * sin(deltaTheta) * cos(deltaTheta)
                    + M2 * G * sin(theta[i + 1]) * cos(deltaTheta)
                    + M2 * L2 * omega[i + 1] * omega[i + 1] * sin(deltaTheta)
                    - (M1 + M2) * G * sin(theta[i])) / denom1;

                float a2 = (-L1 / L2 * omega[i] * omega[i] * sin(deltaTheta) * cos(deltaTheta)
                    + G * sin(theta[i]) * cos(deltaTheta)
                    - G * sin(theta[i + 1])) / denom2;

                omega[i] += a1 * dt;
                omega[i + 1] += a2 * dt;
                theta[i] += omega[i] * dt;
                theta[i + 1] += omega[i + 1] * dt;
            }
        }

        if (!pendulums.empty()) {
            size_t lastIdx = pendulums.size() - 1;
            float baseX = 0.0f;
            float baseY = 0.5f;
            float x = baseX;
            float y = baseY;

            for (size_t i = 0; i <= lastIdx; ++i) {
                x += pendulums[i].x * sin(theta[i]);
                y -= pendulums[i].x * cos(theta[i]);
            }

            if (pathVertices.size() >= PATH_LIMIT * 2) {
                pathVertices.erase(pathVertices.begin(), pathVertices.begin() + 2);
            }
            pathVertices.push_back(x);
            pathVertices.push_back(y);
        }
    }
}

std::vector<float> generateCircleVertices(float cx, float cy, float radius, int segments) {
    std::vector<float> vertices;
    float angleStep = 2.0f * M_PI / segments;
    for (int i = 0; i <= segments; ++i) {
        float angle = i * angleStep;
        float x = cx + radius * cos(angle);
        float y = cy + radius * sin(angle);
        vertices.push_back(x);
        vertices.push_back(y);
    }
    return vertices;
}

void render(GLFWwindow* window, unsigned int VAO, unsigned int VBO, unsigned int shaderProgram) {
    glClear(GL_COLOR_BUFFER_BIT);

    glUseProgram(shaderProgram);

    unsigned int projLoc = glGetUniformLocation(shaderProgram, "projection");
    glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projection));

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);

    float baseX = 0.0f;
    float baseY = 0.5f;

    float prevX = baseX;
    float prevY = baseY;

    std::vector<float> lineVertices;

    for (size_t i = 0; i < pendulums.size(); ++i) {
        float x = prevX + pendulums[i].x * sin(theta[i]);
        float y = prevY - pendulums[i].x * cos(theta[i]);

        if (i >= 0) {
            lineVertices.push_back(prevX);
            lineVertices.push_back(prevY);
            lineVertices.push_back(x);
            lineVertices.push_back(y);
        }

        int circleSegments = 30;
        float circleRadius = PENDULUM_RADIUS;
        std::vector<float> circleVertices = generateCircleVertices(x, y, circleRadius, circleSegments);
        glBufferData(GL_ARRAY_BUFFER, circleVertices.size() * sizeof(float), circleVertices.data(), GL_DYNAMIC_DRAW);
        glDrawArrays(GL_TRIANGLE_FAN, 0, circleSegments + 1);

        prevX = x;
        prevY = y;
    }

    if (!lineVertices.empty()) {
        glBufferData(GL_ARRAY_BUFFER, lineVertices.size() * sizeof(float), lineVertices.data(), GL_DYNAMIC_DRAW);
        glDrawArrays(GL_LINES, 0, lineVertices.size() / 2);
    }

    if (!pathVertices.empty()) {
        glBufferData(GL_ARRAY_BUFFER, pathVertices.size() * sizeof(float), pathVertices.data(), GL_DYNAMIC_DRAW);
        glDrawArrays(GL_LINE_STRIP, 0, pathVertices.size() / 2);
    }

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
        if (!pendulums.empty()) {
            float baseX = 0.0f;
            float baseY = 0.75f;
            float x = baseX;
            float y = baseY;

            for (size_t i = 0; i < pendulums.size(); ++i) {
                x += pendulums[i].x * sin(theta[i]);
                y -= pendulums[i].x * cos(theta[i]);
            }

            pendulums.push_back(glm::vec2(INITIAL_LENGTH, INITIAL_MASS));
            theta.push_back(M_PI / 4.0f);
            omega.push_back(0.0f);
        }
    }
    else if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS) {
        if (pendulums.size() > 1) {
            pendulums.pop_back();
            theta.pop_back();
            omega.pop_back();
        }
        else if (pendulums.size() == 1) {
            pathVertices.clear();
        }
    }
}

int main() {
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(w, h, "Pendulum System", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);

    glfwSetMouseButtonCallback(window, mouseButtonCallback);

    unsigned int vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexShaderSource, nullptr);
    glCompileShader(vertexShader);
    
    unsigned int fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, nullptr);
    glCompileShader(fragmentShader);

    unsigned int shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);
    
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    unsigned int VAO, VBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, 6 * sizeof(float), nullptr, GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    initialize();
    glfwSwapInterval(1);

    while (!glfwWindowShouldClose(window)) {
        computePhysics();
        render(window, VAO, VBO, shaderProgram);
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
