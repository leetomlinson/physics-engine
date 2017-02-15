/******************************************************************************
 * Copyright (C) 2017 Lee Tomlinson
 *
 * This file is part of Physics Engine.
 *
 * Physics Engine is made available under the terms of the
 * Attribution-NonCommercial-NoDerivs 3.0 Unported (CC BY-NC-ND 3.0)
 * licence: https://creativecommons.org/licenses/by-nc-nd/3.0/
 ******************************************************************************/

#version 330 core

layout (location = 0) in vec3 position;
layout (location = 1) in vec3 color;
layout (location = 2) in vec2 uv;
layout (location = 3) in vec3 normal;
out vec3 c;
out vec2 uv_;
out vec3 normal_;
out vec3 light_;
out vec3 frag_pos_;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform vec3 light;

void main() {
    gl_Position = projection * view * model * vec4(position.x, position.y, position.z, 1.0f);
    frag_pos_ = vec3(model * vec4(position.x, position.y, position.z, 1.0f));
    c = color;
    uv_ = vec2(uv.x, 1.0f - uv.y);
    normal_ = normal;
    light_ = light;
}
