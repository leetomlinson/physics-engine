/******************************************************************************
 * Copyright (C) 2017 Lee Tomlinson
 *
 * This file is part of Physics Engine.
 *
 * Physics Engine is made available under the terms of the
 * Attribution-NonCommercial-NoDerivs 3.0 Unported (CC BY-NC-ND 3.0)
 * licence: https://creativecommons.org/licenses/by-nc-nd/3.0/
 ******************************************************************************/

#define GLEW_STATIC

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <memory>
#include "object.h"
#include "scene.h"
#include "aux.h"
#include "config.h"


GLFWwindow *InitWindow(int width, int height, const char title[]);

GLuint CompileShaderSource(std::string const &source, GLenum type);

GLuint BuildShaderProgram(std::string const &vertex_shader_source, std::string const &fragment_shader_source);


int main() {
    GLFWwindow *window = InitWindow(800, 600, "Physics Engine");
    if (window == nullptr) return -1;

    std::string vertex_shader_source = LoadShaderSource(PROJECT_SOURCE_DIR "/vertex.glsl");
    std::string fragment_shader_source = LoadShaderSource(PROJECT_SOURCE_DIR "/fragment.glsl");
    GLuint shader_program = BuildShaderProgram(vertex_shader_source, fragment_shader_source);

    Bitmap image = LoadTexture(PROJECT_SOURCE_DIR "/res/12ball.bmp");

    GLuint texture;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image.width, image.height, 0, GL_BGR, GL_UNSIGNED_BYTE,
                 &(image.bgr_data[0]));
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glBindTexture(GL_TEXTURE_2D, 0);

    auto sphere_mesh = LoadWavefrontModel(PROJECT_SOURCE_DIR "/res/poolball.obj");

    // Create render components
    auto render_sphere_1 = std::make_shared<RenderComponent>();
    render_sphere_1->SetMesh(sphere_mesh);
    render_sphere_1->SetProgram(shader_program);

    auto render_sphere_2 = std::make_shared<RenderComponent>();
    render_sphere_2->SetMesh(sphere_mesh);
    render_sphere_2->SetProgram(shader_program);

    // Create physics components
    auto phys_sphere_1 = std::make_shared<SolidSphere>(1.0, 1.0);
    phys_sphere_1->SetPosition(Vector3(-2.0, 0.5, 0.0));
    phys_sphere_1->SetVelocity(Vector3(0.5, 0.0, 0.0));
    phys_sphere_1->SetAngularVelocity(Vector3(0.0, 0.0, 1.0));

    auto phys_sphere_2 = std::make_shared<SolidSphere>(1.0, 1.0);
    phys_sphere_2->SetPosition(Vector3(2.0, -0.5, 0.0));
    phys_sphere_2->SetVelocity(Vector3(-0.5, 0.0, 0.0));
    phys_sphere_2->SetAngularVelocity(Vector3(0.0, -1.0, 2.0));

    // Compose objects from render and physics components
    auto sphere_1 = std::make_shared<AnObject>();
    sphere_1->AddRenderComponent(render_sphere_1);
    sphere_1->AddPhysicsComponent(phys_sphere_1);

    auto sphere_2 = std::make_shared<AnObject>();
    sphere_2->AddRenderComponent(render_sphere_2);
    sphere_2->AddPhysicsComponent(phys_sphere_2);

    // Add objects to the scene manager
    auto scene = Scene::GetInstance(800, 600, shader_program, window);
    scene->AddObject(sphere_1);
    scene->AddObject(sphere_2);
    scene->AddLight(glm::vec3(-4.0f, 4.0f, 4.0f));

    // Add objects to the physics engine
    PhysicsEngine phys;
    phys.AddObject(phys_sphere_1.get());
    phys.AddObject(phys_sphere_2.get());

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        // Clear screen
        glClearColor(0.0f, 0.2f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Render scene
        glBindTexture(GL_TEXTURE_2D, texture);
        phys.Update();
        scene->Update();
        scene->Render();
        glBindTexture(GL_TEXTURE_2D, 0);

        glfwSwapBuffers(window);
    }

    glfwTerminate();
    return 0;
}

/**
 * Returns handle to shader compiled from source.
 *
 * @param source
 * @param type
 * @return
 */
GLuint CompileShaderSource(std::string const &source, GLenum type) {
    const GLchar *const src = source.c_str();
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &src, NULL);
    glCompileShader(shader);

    GLint success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        GLchar info_log[512];
        glGetShaderInfoLog(shader, 512, NULL, info_log);
        std::cerr << "Error: Shader compilation failed: " << info_log << '\n';
    }

    return shader;
}

/**
 * Returns handle to linked shader program compiled from vertex and fragment sources.
 *
 * @param vertex_shader_source
 * @param fragment_shader_source
 * @return
 */
GLuint BuildShaderProgram(std::string const &vertex_shader_source, std::string const &fragment_shader_source) {
    GLuint vertex_shader = CompileShaderSource(vertex_shader_source, GL_VERTEX_SHADER);
    GLuint fragment_shader = CompileShaderSource(fragment_shader_source, GL_FRAGMENT_SHADER);

    GLuint program = glCreateProgram();
    glAttachShader(program, vertex_shader);
    glAttachShader(program, fragment_shader);
    glLinkProgram(program);

    glDeleteShader(vertex_shader);
    glDeleteShader(fragment_shader);

    GLint success;
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success) {
        GLchar info_log[512];
        glGetProgramInfoLog(program, 512, NULL, info_log);
        std::cerr << "Error: Unable to link shader program: " << info_log << '\n';
    }

    return program;
}

/**
 * Initializes everything necessary and returns a pointer to window context.  It is the caller's responsibility to also
 * call glfwTerminate() when finished.
 *
 * @param width
 * @param height
 * @param title
 * @return
 */
GLFWwindow *InitWindow(int width, int height, const char title[]) {
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

    GLFWwindow *window;
    window = glfwCreateWindow(width, height, title, nullptr, nullptr);

    if (window == nullptr) {
        std::cerr << "Failed to create GLFW window\n";
        glfwTerminate();
        return nullptr;
    }
    glfwMakeContextCurrent(window);
//    glfwSetKeyCallback();

    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK) {
        std::cerr << "Failed to initialize GLEW\n";
        glfwTerminate();
        return nullptr;
    }

    int tmp_width, tmp_height;
    glfwGetFramebufferSize(window, &tmp_width, &tmp_height);
    glViewport(0, 0, tmp_width, tmp_height);
    glEnable(GL_DEPTH_TEST);

    return window;
}
