/******************************************************************************
 * Copyright (C) 2017 Lee Tomlinson
 *
 * This file is part of Physics Engine.
 *
 * Physics Engine is made available under the terms of the
 * Attribution-NonCommercial-NoDerivs 3.0 Unported (CC BY-NC-ND 3.0)
 * licence: https://creativecommons.org/licenses/by-nc-nd/3.0/
 ******************************************************************************/

#ifndef PHYSICS_ENGINE_CAMERA_H
#define PHYSICS_ENGINE_CAMERA_H

#include <chrono>
#include <glm/glm.hpp>


class Camera {
private:
    glm::vec3 subject_pos_;
    glm::vec3 camera_pos_;
    glm::mat4 view_;
    glm::vec3 velocity_;

    Vector3 angular_velocity_;

    float speed_;
    std::chrono::time_point<std::chrono::steady_clock> time_;

    void UpdateView() {
        // Get time interval since last update
        std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
        std::chrono::duration<float, std::ratio<1, 1>> dt;
        dt = now - this->time_;
        this->time_ = now;

        // Update camera and subject positions
        this->camera_pos_ += dt.count() * this->speed_ * this->velocity_;
        this->subject_pos_ += dt.count() * this->speed_ * this->velocity_;
        this->velocity_ = glm::vec3();

        // TODO: Can this code be simplified?
        if (this->angular_velocity_.x != 0.0 ||
            this->angular_velocity_.y != 0.0 ||
            this->angular_velocity_.z != 0.0) {
            Vector3 axis = dt.count() * this->angular_velocity_;
            this->angular_velocity_ = Vector3();

            Vector3 subject_to_camera(this->camera_pos_.x - this->subject_pos_.x,
                                      this->camera_pos_.y - this->subject_pos_.y,
                                      this->camera_pos_.z - this->subject_pos_.z);
            Matrix3 rotate_camera = RotationMatrix(axis);
            subject_to_camera = Dot(rotate_camera, subject_to_camera);
            this->camera_pos_.x = (float) subject_to_camera.x;
            this->camera_pos_.y = (float) subject_to_camera.y;
            this->camera_pos_.z = (float) subject_to_camera.z;
            this->camera_pos_ += this->subject_pos_;
        }

        // Update view matrix
        this->view_ = glm::lookAt(camera_pos_, subject_pos_, glm::vec3(0.0f, 1.0f, 0.0f));
    }

public:
    Camera() : camera_pos_(0.0f, 0.0f, 8.0f), subject_pos_(0.0f, 0.0f, 0.0f), speed_{5.0f} {
    }

    void MoveLeft() {
        this->velocity_ += glm::vec3(-1.0f, 0.0f, 0.0f);
    }

    void MoveRight() {
        this->velocity_ += glm::vec3(1.0f, 0.0f, 0.0f);
    }

    void MoveUp() {
        this->velocity_ += glm::vec3(0.0f, 1.0f, 0.0f);
    }

    void MoveDown() {
        this->velocity_ += glm::vec3(0.0f, -1.0f, 0.0f);
    }

    void RotateAboutSubject(float dx, float dy) {
        // TODO: Needs to be fixed properly
        this->angular_velocity_ = Vector3(dy, dx, 0.0);
    }

    glm::mat4 const &GetViewMatrix() {
        this->UpdateView();
        return this->view_;
    }
};


#endif //PHYSICS_ENGINE_CAMERA_H
