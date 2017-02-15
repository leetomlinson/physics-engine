/******************************************************************************
 * Copyright (C) 2017 Lee Tomlinson
 *
 * This file is part of Physics Engine.
 *
 * Physics Engine is made available under the terms of the
 * Attribution-NonCommercial-NoDerivs 3.0 Unported (CC BY-NC-ND 3.0)
 * licence: https://creativecommons.org/licenses/by-nc-nd/3.0/
 ******************************************************************************/

#include <fstream>
#include <iostream>
#include <vector>
#include <unordered_map>
#include "aux.h"
#include "boost/format.hpp"


/**
 * Loads shader source from a file and returns it as a string.
 *
 * @param path
 * @return
 */
std::string LoadShaderSource(const char *path) {
    std::ifstream in;
    in.open(path, std::ios_base::in | std::ios_base::ate);
    std::string src;
    if (in.is_open()) {
        std::fpos<mbstate_t> len = in.tellg();
        in.seekg(0);
        // TODO: The following lines are not exception-safe and might result in a memory leak
        char *str = new char[(unsigned int) len + 1];
        in.read(str, len);
        str[len] = 0;
        src.assign(str);
        delete[] str;
        in.close();
    } else {
        // TODO: Consider throwing a more specific exception
        std::string msg = (boost::format("Unable to open %s as shader source\n") % path).str();
        throw std::runtime_error(msg);
    }
    return src;
}

/**
 * Loads bitmap image data from a file and returns a Bitmap structure, containing the width and height of the image,
 * and a character array of the bgr values for each pixel.
 *
 * @param path
 * @return
 */
Bitmap LoadTexture(std::string const &path) {
    // TODO: Implement some sort of error checking
    std::ifstream in;
    in.open(path, std::ios_base::in | std::ios_base::binary | std::ios_base::ate);
    std::vector<char> data;
    Bitmap bmp;
    if (in.is_open()) {
        // Read binary file into data array
        std::fpos<mbstate_t> count = in.tellg();
        data.resize((unsigned long) count);
        in.seekg(0);
        in.read(&data[0], count);

        if (data[0] != 'B' || data[1] != 'M') {
            // Invalid bitmap file
            // TODO: Consider throwing a more specific exception
            std::string msg = "Invalid bitmap image file: " + path;
            throw std::runtime_error(msg);
        }

        // Determine offset of pixel array from bitmap file header
        int offset = *((int *) (&data[0] + 0x0A));

        // Determine width and height of image from DIB header
        bmp.width = *((int *) (&data[0] + 0x12));
        bmp.height = *((int *) (&data[0] + 0x16));

        // Remove 4-byte padding from bitmap data
        int padding = (4 - (bmp.width * 3) % 4) % 4;
        bmp.bgr_data.resize((unsigned long) (bmp.width * bmp.height * 3));

        for (int i = 0; i < bmp.height; ++i) {
            auto it_src_begin = data.begin() + (3 * bmp.width + padding) * i + offset;
            auto it_src_end = it_src_begin + 3 * bmp.width;
            auto it_dest_begin = bmp.bgr_data.begin() + 3 * bmp.width * i;
            std::copy(it_src_begin, it_src_end, it_dest_begin);
        }

        in.close();
    } else {
        // TODO: Consider throwing a more specific exception
        std::string msg = "Unable to open file " + path;
        throw std::runtime_error(msg);
    }

    return bmp;
}

/**
 * Loads vertex data from a Wavefront model file and constructs a Mesh object.
 *
 * @param path
 * @return
 */
std::shared_ptr<Mesh> LoadWavefrontModel(std::string const &path) {
    std::ifstream in;
    in.open(path);
    if (in.is_open()) {
        std::string line;
        WavefrontParser wp;
        while (in.good()) {
            std::getline(in, line, '\n');
            wp.ParseLine(line);
        }
        in.close();
        return std::make_shared<Mesh>(wp.GetVertexData(), wp.GetIndexData());
    } else {
        // TODO: Throw a better exception
        throw std::exception();
    }
}

/**
 * Parses line from Wavefront file and either (1) constructs the appropriate vertex attribute, or (2) constructs a
 * new vertex from existing attributes (if such a vertex does not already exist) and appends the index array with
 * the index of this vertex.
 *
 * @param line
 */
void WavefrontParser::ParseLine(std::string const &line) {
    std::vector<std::string> tokens;
    boost::split(tokens, line, boost::is_any_of(" "));
    if (tokens[0] == "v") {
        if (tokens.size() < 4) {
            throw std::runtime_error("Error parsing wavefront file");
        }
        this->position_arr_.push_back(glm::vec3(stof(tokens[1]), stof(tokens[2]), stof(tokens[3])));
    } else if (tokens[0] == "vt") {
        if (tokens.size() < 3) {
            throw std::runtime_error("Error parsing wavefront file");
        }
        this->uv_arr_.push_back(glm::vec2(stof(tokens[1]), stof(tokens[2])));
    } else if (tokens[0] == "vn") {
        if (tokens.size() < 4) {
            throw std::runtime_error("Error parsing wavefront file");
        }
        this->normal_arr_.push_back(glm::vec3(stof(tokens[1]), stof(tokens[2]), stof(tokens[3])));
    } else if (tokens[0] == "f") {
        if (tokens.size() < 4) {
            throw std::runtime_error("Error parsing wavefront file");
        }
        this->AppendFace_(tokens.begin() + 1, tokens.end());
    }
}

/**
 * Constructs a vertex from indices for vertex attributes already parsed and loaded, and appends vertex and index
 * arrays.
 *
 * @param begin
 * @param end
 */
void WavefrontParser::AppendFace_(std::vector<std::string>::iterator begin, std::vector<std::string>::iterator end) {
    for (auto it = begin; it != end; ++it) {
        std::vector<std::string> indices;
        boost::algorithm::split(indices, *it, boost::is_any_of("/"));
        Vertex v;
        try {
            v.position = this->position_arr_[stoi(indices[0]) - 1];
            v.uv = this->uv_arr_[stoi(indices[1]) - 1];
            v.normal = this->normal_arr_[stoi(indices[2]) - 1];
        }
        catch (std::out_of_range &e) {
            std::string msg = "Error parsing wavefront file: ";
            msg.append(e.what());
            throw std::out_of_range(msg);
        }
        this->PushVertex_(v);
    }
}

/**
 * Appends the vertex array with this vertex if it does not already exist in the array, and appends the index array
 * with the index of this vertex in the vertex array.
 *
 * @param vertex
 */
void WavefrontParser::PushVertex_(Vertex vertex) {
    GLuint index;
    if (this->vertex_index_map_.find(vertex) != this->vertex_index_map_.end()) {
        index = this->vertex_index_map_.at(vertex);
    } else {
        index = (GLuint) vertex_arr_.size();
        this->vertex_index_map_[vertex] = index;
        this->vertex_arr_.push_back(vertex);
    }
    this->index_arr_.push_back(index);
}
