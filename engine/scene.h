/******************************************************************************
 * Copyright (C) 2017 Lee Tomlinson
 *
 * This file is part of Physics Engine.
 *
 * Physics Engine is made available under the terms of the
 * Attribution-NonCommercial-NoDerivs 3.0 Unported (CC BY-NC-ND 3.0)
 * licence: https://creativecommons.org/licenses/by-nc-nd/3.0/
 ******************************************************************************/

#ifndef PHYSICS_ENGINE_SCENE_H
#define PHYSICS_ENGINE_SCENE_H

#include <vector>
#include <memory>
#include <GLFW/glfw3.h>
#include "object.h"
#include "camera.h"


/**
 * TODO: Tidy up this documentation.
 *
 * The scene is managed by an instance of Scene, which contains information pertinent to everything that may be part of
 * the simulation, including e.g. rendered and physical components.
 *
 * A design consideration has been made here, owing to GLFW being a C library.  GLFW uses function pointers to call
 * callback functions in order to handle user input.
 *
 * The GLFW function glfwSetKeyCallback takes a function pointer as its second argument, and this callback function must
 * have the signature void (*)(GLFWwindow*,int,int,int,int) (aka GLFWkeyfun).
 *
 * Since one must use pointers to callback functions, yet these functions need to manipulate data pertinent to an
 * instance of Scene, one needs to use static functions (as pointers-member-functions take an implicit first argument
 * and are not supported in C, which knows nothing of classes or the 'this' object).  Also, globals are not preferable.
 *
 * Moreover, it makes sense that only one instance of Scene should be made, as this is the 'universe' in this
 * simulation, so to speak.
 *
 * The result is a decision to design Scene using the singleton pattern.
 */
class Scene {
private:
    using ObjectPtr = std::shared_ptr<AnObject>;
    std::vector<ObjectPtr> object_arr_;
    glm::mat4 projection_;
    Camera camera_;
    GLuint program_;
    glm::vec3 light_arr_;

    static std::shared_ptr<Scene> inst;

    Scene(int width, int height, GLuint program, GLFWwindow *window) : program_{program} {
        this->projection_ = glm::perspective(45.0f, (float) width / (float) height, 0.1f, 100.f);
        glfwSetKeyCallback(window, Scene::KeyCallback);
        glfwSetCursorPosCallback(window, Scene::CursorPosCallback);
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    }

    double xpos_prev_;
    double ypos_prev_;

    Camera &GetCamera() {
        return this->camera_;
    }

public:
    void AddObject(ObjectPtr object) {
        object_arr_.push_back(object);
    }

    void AddLight(glm::vec3 const &position) {
        this->light_arr_ = position;
    }

    void Update() {
        for (ObjectPtr o : this->object_arr_) {
            o->Update();
        }
    }

    void Render() {
        // Set up lighting
        GLint light_location = glGetUniformLocation(this->program_, "light");
        glUniform3fv(light_location, 1, glm::value_ptr(this->light_arr_));

        // Render all meshes
        for (ObjectPtr o : this->object_arr_) {
            o->Render(this->camera_.GetViewMatrix(), this->projection_);
        }
    }

    /**
     * Handle user input.  See http://www.glfw.org/docs/latest/input_guide.html
     *
     * @param window
     * @param key
     * @param scancode
     * @param action
     * @param mods
     */
    static void KeyCallback(GLFWwindow *window, int key, int scancode, int action, int mods) {
        // TODO: Consider if it is really appropriate to have the callback function here
        if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
            glfwSetWindowShouldClose(window, GLFW_TRUE);
        }
        if (key == GLFW_KEY_LEFT) {
            inst->GetCamera().MoveLeft();
        }
        if (key == GLFW_KEY_RIGHT) {
            inst->GetCamera().MoveRight();
        }
        if (key == GLFW_KEY_UP) {
            inst->GetCamera().MoveUp();
        }
        if (key == GLFW_KEY_DOWN) {
            inst->GetCamera().MoveDown();
        }
    }

    /**
     * Handle mouse input.  See http://www.glfw.org/docs/latest/input_guide.html#input_mouse
     *
     * @param window
     * @param xpos
     * @param ypos
     */
    static void CursorPosCallback(GLFWwindow *window, double xpos, double ypos) {
        // TODO: Figure out how mouse movement works!
//        auto mat = glm::rotate(glm::mat4(1.0f), (float) xpos, glm::vec3(0.0f, 1.0f, 0.0f));
//        inst->GetCamera().Move(glm::vec3(xpos / 10000.0f, ypos / 10000.0f, 0.0f));
        inst->GetCamera().RotateAboutSubject((float) (xpos - inst->xpos_prev_), (float) (ypos - inst->ypos_prev_));
        inst->xpos_prev_ = xpos;
        inst->ypos_prev_ = ypos;
    }

    static std::shared_ptr<Scene> GetInstance(int width, int height, GLuint program, GLFWwindow *window);
};

std::shared_ptr<Scene> Scene::inst = nullptr;

std::shared_ptr<Scene> Scene::GetInstance(int width, int height, GLuint program, GLFWwindow *window) {
    if (!inst) {
        // Scene::this_ = std::make_unique<Scene>(); // C++14
        // this_ = std::unique_ptr<Scene>(new Scene(width, height, program, window));
        inst = std::shared_ptr<Scene>(new Scene(width, height, program, window));
    }
    return inst;
}


#endif //PHYSICS_ENGINE_SCENE_H
