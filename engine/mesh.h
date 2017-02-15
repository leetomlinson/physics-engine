/******************************************************************************
 * Copyright (C) 2017 Lee Tomlinson
 *
 * This file is part of Physics Engine.
 *
 * Physics Engine is made available under the terms of the
 * Attribution-NonCommercial-NoDerivs 3.0 Unported (CC BY-NC-ND 3.0)
 * licence: https://creativecommons.org/licenses/by-nc-nd/3.0/
 ******************************************************************************/

#ifndef PHYSICS_ENGINE_MESH_H
#define PHYSICS_ENGINE_MESH_H

#include <GL/glew.h>
#include <vector>
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "boost/functional/hash.hpp"


struct Vertex {
    glm::vec3 position;
    glm::vec3 color;
    glm::vec2 uv;
    glm::vec3 normal;
};

bool operator==(Vertex const &lhs, Vertex const &rhs);

/**
 * Hash function for an object of type Vertex.
 */
struct VertexHash {
    std::size_t operator()(Vertex const &vertex) const {
        std::size_t hash = 0;
        // Interpret Vertex object as a contiguous array of floats.
        for (int i = 0; i < sizeof(Vertex) / sizeof(float); ++i) {
            float val = ((float *) &vertex)[i];
            std::size_t float_hash = std::hash<float>{}(val);
//            boost::hash_combine(hash, val);
            hash ^= float_hash;
        }
        return hash;
    }
};

class Mesh {
private:
    GLuint vao_;
    GLuint vbo_;
    GLuint ebo_;
    unsigned long count_;

    bool valid_indices_(unsigned long size, std::vector<GLuint> const &indices) {
        for (GLuint a : indices) if (a >= size) return false;
        return true;
    }

public:
    /**
     * Generate vertex array and buffer objects, and send vertex data to GPU.  Requires a shader program which
     * interprets vertex attribute locations 0 to 3 as defined in Vertex structure.
     *
     * @param vertices
     * @param indices
     */
    Mesh(std::vector<Vertex> const &vertices, std::vector<GLuint> const &indices) : count_{indices.size()} {
        // Validate input
        if (!valid_indices_(vertices.size(), indices)) {
            throw std::out_of_range{"Mesh vertex index out of range"};
        }

        // Generate buffers and send vertex data to GPU
        glGenVertexArrays(1, &vao_);
        glBindVertexArray(vao_);

        glGenBuffers(1, &vbo_);
        glBindBuffer(GL_ARRAY_BUFFER, vbo_);
        glBufferData(GL_ARRAY_BUFFER, sizeof(Vertex) * vertices.size(), &vertices[0], GL_STATIC_DRAW);

        // Vertex coordinates
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid *) 0);
        glEnableVertexAttribArray(0);

        // Vertex colours
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid *) (3 * sizeof(GLfloat)));
        glEnableVertexAttribArray(1);

        // Texture coordinates
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid *) (6 * sizeof(GLfloat)));
        glEnableVertexAttribArray(2);

        // Vertex normals
        glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid *) (8 * sizeof(GLfloat)));
        glEnableVertexAttribArray(3);

        glGenBuffers(1, &ebo_);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint) * indices.size(), &indices[0], GL_STATIC_DRAW);

        glBindVertexArray(0);
    }

    ~Mesh() {
        // TODO: I think this needs to move to the end of the constructor
        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    }

    void Draw() {
        glBindVertexArray(vao_);
        glDrawElements(GL_TRIANGLES, (GLsizei) this->count_, GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
    }
};


#endif //PHYSICS_ENGINE_MESH_H
