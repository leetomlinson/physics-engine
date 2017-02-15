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
#include "gtest/gtest.h"
#include "aux.h"
#include "config.h"


TEST(test_load_shader_source, basic_success) {
    std::string str = LoadShaderSource(PROJECT_SOURCE_DIR "/engine_test/shader.glsl");
    std::string str_exp = "#version 330 core\nout vec4 color;";
    EXPECT_EQ(str, str_exp);
}


TEST(test_load_shader_source, file_not_found) {
    try {
        std::string str = LoadShaderSource(PROJECT_SOURCE_DIR "/engine_test/asdf.glsl");
        FAIL();
    }
    catch (std::exception &e) {
        // Should LoadShaderSource throw an exception if shader source cannot be found?
    }
}

TEST(test_load_texture, load_orange_success) {
    // orange.bmp is a 16x9 bitmap image with uniform rgb values 255, 127, 0
    Bitmap bmp = LoadTexture(PROJECT_SOURCE_DIR "/engine_test/orange.bmp");

    EXPECT_EQ(bmp.width, 16);
    EXPECT_EQ(bmp.height, 9);
    EXPECT_EQ(bmp.bgr_data.size(), 16 * 9 * 3);

    EXPECT_EQ(bmp.bgr_data[0], 0);
    EXPECT_EQ(bmp.bgr_data[1], 127);
    EXPECT_EQ(bmp.bgr_data[2], (char) 255);

    EXPECT_EQ(bmp.bgr_data[429], 0);
    EXPECT_EQ(bmp.bgr_data[430], 127);
    EXPECT_EQ(bmp.bgr_data[431], (char) 255);
}

TEST(test_load_texture, load_rygb_success) {
    // rygb.bmp is a 2x2 bitmap image with the colours red, yellow, green, and blue
    Bitmap bmp = LoadTexture(PROJECT_SOURCE_DIR "/engine_test/rygb.bmp");

    EXPECT_EQ(bmp.width, 2);
    EXPECT_EQ(bmp.height, 2);
    EXPECT_EQ(bmp.bgr_data.size(), 12);

    // Green
    EXPECT_EQ(bmp.bgr_data[0], (char) 0);
    EXPECT_EQ(bmp.bgr_data[1], (char) 255);
    EXPECT_EQ(bmp.bgr_data[2], (char) 0);

    // Blue
    EXPECT_EQ(bmp.bgr_data[3], (char) 255);
    EXPECT_EQ(bmp.bgr_data[4], (char) 0);
    EXPECT_EQ(bmp.bgr_data[5], (char) 0);

    // Red
    EXPECT_EQ(bmp.bgr_data[6], (char) 0);
    EXPECT_EQ(bmp.bgr_data[7], (char) 0);
    EXPECT_EQ(bmp.bgr_data[8], (char) 255);

    // Yellow
    EXPECT_EQ(bmp.bgr_data[9], (char) 0);
    EXPECT_EQ(bmp.bgr_data[10], (char) 255);
    EXPECT_EQ(bmp.bgr_data[11], (char) 255);
}

TEST(test_load_wavefront_model, load_square_success) {
    // Loads mesh data from the file square.obj.  This mesh is a square in the x-y plane with side-length two,
    // and is centred on the origin.  It is made from four vertices, each with a z component equal to zero.  All
    // normal vectors point in the positive z direction, and all texture coordinates are equal to the position of
    // the corresponding vertex.  The parsing of this file should result in two triangles made from these vertices,
    // in a 'lower-left and upper-right' configuration.
    std::ifstream in(PROJECT_SOURCE_DIR "/engine_test/square.obj");
    WavefrontParser wp;
    if (in.is_open()) {
        std::string line;
        while (in.good()) {
            std::getline(in, line, '\n');
            wp.ParseLine(line);
        }
    }
    in.close();

    std::vector<GLuint> index_arr_exp{0, 1, 2, 1, 3, 2};
    std::vector<Vertex> vertex_arr_exp{
            {{-1.0f, -1.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {-1.0f, -1.0f}, {0.0f, 0.0f, 1.0f}},
            {{1.0f,  -1.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {1.0f,  -1.0f}, {0.0f, 0.0f, 1.0f}},
            {{-1.0f, 1.0f,  0.0f}, {0.0f, 0.0f, 0.0f}, {-1.0f, 1.0f},  {0.0f, 0.0f, 1.0f}},
            {{1.0f,  1.0f,  0.0f}, {0.0f, 0.0f, 0.0f}, {1.0f,  1.0f},  {0.0f, 0.0f, 1.0f}}
    };

    EXPECT_EQ(wp.GetVertexData(), vertex_arr_exp);
    EXPECT_EQ(wp.GetIndexData(), index_arr_exp);
}
