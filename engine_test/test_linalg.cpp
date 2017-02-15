/******************************************************************************
 * Copyright (C) 2017 Lee Tomlinson
 *
 * This file is part of Physics Engine.
 *
 * Physics Engine is made available under the terms of the
 * Attribution-NonCommercial-NoDerivs 3.0 Unported (CC BY-NC-ND 3.0)
 * licence: https://creativecommons.org/licenses/by-nc-nd/3.0/
 ******************************************************************************/

#include <gtest/gtest.h>
#include <linalg.h>


TEST(test_transpose_matrix, basic_test) {
    Matrix3 mat(3, 1, 4, 1, 5, 9, 2, 6, 5);
    Matrix3 mat_trans_expected(3, 1, 2, 1, 5, 6, 4, 9, 5);
    Matrix3 mat_trans_actual = Transpose(mat);
    EXPECT_EQ(mat_trans_actual, mat_trans_expected);
}

TEST(test_matrix_dot_vector, basic_test) {
    Matrix3 mat(3, 1, 4, 1, 5, 9, 2, 6, 5);
    Vector3 vec(2, 7, 1);
    Vector3 prod_expected(17, 46, 51);
    Vector3 prod_actual = Dot(mat, vec);
    EXPECT_EQ(prod_actual, prod_expected);

}
