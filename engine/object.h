/******************************************************************************
 * Copyright (C) 2017 Lee Tomlinson
 *
 * This file is part of Physics Engine.
 *
 * Physics Engine is made available under the terms of the
 * Attribution-NonCommercial-NoDerivs 3.0 Unported (CC BY-NC-ND 3.0)
 * licence: https://creativecommons.org/licenses/by-nc-nd/3.0/
 ******************************************************************************/

#ifndef PHYSICS_ENGINE_OBJECT_H
#define PHYSICS_ENGINE_OBJECT_H

#include <bits/shared_ptr.h>
#include "mesh.h"
#include "glm/glm.hpp"
#include "physics.h"


class RenderComponent {
private:
    // What we are describing here is a 'render' object.  Ultimately I might refactor this such that an object
    // contains a render component, a phyiscs component, etc.
    std::shared_ptr<Mesh> mesh_;
    glm::mat4 model_mat_;
    GLuint program_;
    glm::vec3 pos_;
    glm::vec3 about_;
    float angle_;

public:
    RenderComponent() : mesh_{nullptr}, model_mat_{1.0f}, program_{0} {
        this->model_mat_ = glm::translate(this->model_mat_, glm::vec3(0.25f, 0.25f, -3.0f));
        this->model_mat_ = glm::rotate(this->model_mat_, 0.3f, glm::vec3(1.0f, 1.0f, 0.0f));
    }

    void SetMesh(std::shared_ptr<Mesh> mesh) {
        this->mesh_ = mesh;
    }

    void SetProgram(GLuint program) {
        this->program_ = program;
    }

    /**
     * Renders the model described by this object, using the input for view and projection matrices.  The view and
     * projection matrices must be provided externally because they are separate from the definition of the model, which
     * includes the mesh(es), shader(s), model matrix/ces, etc.
     *
     * @param view
     * @param projection
     */
    void Render(glm::mat4 const &view, glm::mat4 const &projection) {
        if (this->mesh_ != nullptr && this->program_ != 0) {
            glUseProgram(this->program_);

            GLint model_mat_location = glGetUniformLocation(this->program_, "model");
            glUniformMatrix4fv(model_mat_location, 1, GL_FALSE, glm::value_ptr(this->model_mat_));

            GLint view_mat_location = glGetUniformLocation(this->program_, "view");
            glUniformMatrix4fv(view_mat_location, 1, GL_FALSE, glm::value_ptr(view));

            GLint perspective_mat_location = glGetUniformLocation(this->program_, "projection");
            glUniformMatrix4fv(perspective_mat_location, 1, GL_FALSE, glm::value_ptr(projection));

            mesh_->Draw();
        }
    }

    void UpdateModelMatrix() {
        this->model_mat_ = glm::mat4();
        this->model_mat_ = glm::translate(this->model_mat_, this->pos_);
        this->model_mat_ = glm::rotate(this->model_mat_, this->angle_, this->about_);
    }

    void SetPosition(Vector3 position) {
        this->pos_ = glm::vec3(position.x, position.y, position.z);
        this->UpdateModelMatrix();
    }

    void SetRotation(Vector3 theta) {
        if (theta.x == 0.0 && theta.y == 0.0 && theta.z == 0.0) {
            this->about_ = glm::vec3(1.0, 0.0, 0.0);
            this->angle_ = 0.0;
        } else {
            double mag = std::sqrt(Dot(theta, theta));
            this->about_ = glm::vec3(theta.x / mag, theta.y / mag, theta.z / mag);
            this->angle_ = (float) mag;
        }
        this->UpdateModelMatrix();
    }
};


class AnObject {
private:
    std::shared_ptr<RenderComponent> render_component_;
    std::shared_ptr<PhysicsObject> physics_component_;

public:
    AnObject() : render_component_{nullptr}, physics_component_{nullptr} {
    }

    void Render(glm::mat4 const &view, glm::mat4 const &projection) {
        if (this->render_component_ != nullptr) {
            this->render_component_->Render(view, projection);
        }
    }

    void Update() {
        if (this->physics_component_ != nullptr) {
            // Now need to communicate update to the render component
            auto pos = this->physics_component_->GetPosition();
            this->render_component_->SetPosition(pos);

            auto ang = this->physics_component_->GetRotation();
            this->render_component_->SetRotation(ang);
        }
    }

    void AddRenderComponent(std::shared_ptr<RenderComponent> render_component) {
        this->render_component_ = render_component;
    }

    void AddPhysicsComponent(std::shared_ptr<PhysicsObject> physics_component) {
        this->physics_component_ = physics_component;
    }
};


#endif //PHYSICS_ENGINE_OBJECT_H
