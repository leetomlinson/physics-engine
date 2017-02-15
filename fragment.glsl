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

in vec3 c;
in vec2 uv_;
in vec3 normal_;
in vec3 light_;
in vec3 frag_pos_;
out vec4 color;

uniform sampler2D ourTexture;

void main() {
    vec3 base_color = vec3(texture(ourTexture, uv_)) + c;
    vec3 light_color = vec3(1.0f, 1.0f, 1.0f);
    vec3 diffuse = light_color * max(dot(light_ - frag_pos_, normal_), 0.0f) * base_color;
    vec3 ambiant = vec3(0.8f, 0.8f, 0.8f) * base_color;
    float rsq = dot(frag_pos_ - light_, frag_pos_ - light_);
    color = vec4(ambiant + 5.0 * diffuse / rsq, 1.0f);
}
