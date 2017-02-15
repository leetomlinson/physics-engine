/******************************************************************************
 * Copyright (C) 2017 Lee Tomlinson
 *
 * This file is part of Physics Engine.
 *
 * Physics Engine is made available under the terms of the
 * Attribution-NonCommercial-NoDerivs 3.0 Unported (CC BY-NC-ND 3.0)
 * licence: https://creativecommons.org/licenses/by-nc-nd/3.0/
 ******************************************************************************/

#ifndef PHYSICS_ENGINE_AUX_H
#define PHYSICS_ENGINE_AUX_H

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <unordered_map>
#include "mesh.h"
#include "boost/algorithm/string.hpp"


std::string LoadShaderSource(const char *path);

/**
 * Simple data structure for a bitmap image
 */
struct Bitmap {
    int width;
    int height;
    std::vector<char> bgr_data;
};

/**
 * Auxiliary class for parsing Wavefront files.
 *
 * Usage: Create an object and repeatedly call the ParseLine method with string arguments, each containing subsequent
 * lines from the Wavefront file.  Finally, call GetVertexData and GetIndexData to return vertex and index arrays, from
 * which one can construct a Mesh object.
 */
class WavefrontParser {
private:
    // Arrays of vertex attributes loaded from Wavefront file
    std::vector<glm::vec3> position_arr_;
    std::vector<glm::vec2> uv_arr_;
    std::vector<glm::vec3> normal_arr_;

    // Vertex and index arrays
    std::unordered_map<Vertex, GLuint, VertexHash> vertex_index_map_;
    std::vector<Vertex> vertex_arr_;
    std::vector<GLuint> index_arr_;

    void AppendFace_(std::vector<std::string>::iterator begin, std::vector<std::string>::iterator end);

    void PushVertex_(Vertex vertex);

public:
    void ParseLine(std::string const &line);

    std::vector<Vertex> const &GetVertexData() const {
        return this->vertex_arr_;
    }

    std::vector<GLuint> const &GetIndexData() const {
        return this->index_arr_;
    }
};

Bitmap LoadTexture(std::string const &path);

std::shared_ptr<Mesh> LoadWavefrontModel(std::string const &path);


#endif //PHYSICS_ENGINE_AUX_H
