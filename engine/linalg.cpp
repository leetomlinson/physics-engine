/******************************************************************************
 * Copyright (C) 2017 Lee Tomlinson
 *
 * This file is part of Physics Engine.
 *
 * Physics Engine is made available under the terms of the
 * Attribution-NonCommercial-NoDerivs 3.0 Unported (CC BY-NC-ND 3.0)
 * licence: https://creativecommons.org/licenses/by-nc-nd/3.0/
 ******************************************************************************/

#include <cmath>
#include "linalg.h"


/**
 * Multiplies a vector by a scalar.
 *
 * @param scalar
 * @param vec
 * @return
 */
Vector3 operator*(double scalar, Vector3 vec) {
    return vec *= scalar;
}

/**
 * Divides a vector by a scalar.
 *
 * @param vec
 * @param scalar
 * @return
 */
Vector3 operator/(Vector3 vec, double scalar) {
    return vec /= scalar;
}

/**
 * Adds vec2 to vec1.
 *
 * @param vec1
 * @param vec2
 * @return
 */
Vector3 operator+(Vector3 vec1, Vector3 const &vec2) {
    // Uses the fact that a copy of vec1 is made.
    return vec1 += vec2;
}

/**
 * Subtracts vec2 from vec1.
 *
 * @param vec1
 * @param vec2
 * @return
 */
Vector3 operator-(Vector3 vec1, Vector3 const &vec2) {
    // Uses the fact that a copy of vec1 is made.
    return vec1 -= vec2;
}

/**
 * Returns the dot product of two vectors.
 *
 * @param vec1
 * @param vec2
 * @return
 */
double Dot(Vector3 const &vec1, Vector3 const &vec2) {
    return vec1.x * vec2.x + vec1.y * vec2.y + vec1.z * vec2.z;
}

/**
 * Returns the cross product of two vectors.
 *
 * @param vec1
 * @param vec2
 * @return
 */
Vector3 Cross(Vector3 const &vec1, Vector3 const &vec2) {
    Vector3 prod;
    prod.x = vec1.y * vec2.z - vec1.z * vec2.y;
    prod.y = vec1.z * vec2.x - vec1.x * vec2.z;
    prod.z = vec1.x * vec2.y - vec1.y * vec2.x;
    return prod;
}

/**
 * Returns the transpose of a matrix.
 *
 * @param mat
 * @return
 */
Matrix3 Transpose(Matrix3 const &mat) {
    Matrix3 trans(mat);
    trans.xy = mat.yx;
    trans.xz = mat.zx;
    trans.yx = mat.xy;
    trans.yz = mat.zy;
    trans.zx = mat.xz;
    trans.zy = mat.yz;
    return trans;
}

/**
 * Returns the product of a matrix and a column vector.
 *
 * @param mat
 * @param vec
 * @return
 */
Vector3 Dot(Matrix3 const &mat, Vector3 const &vec) {
    Vector3 prod;
    prod.x = mat.xx * vec.x + mat.xy * vec.y + mat.xz * vec.z;
    prod.y = mat.yx * vec.x + mat.yy * vec.y + mat.yz * vec.z;
    prod.z = mat.zx * vec.x + mat.zy * vec.y + mat.zz * vec.z;
    return prod;
}

/**
 * Returns a rotation matrix.  The rotation is about an axis defined by the vector, and the angle of rotation is given
 * by the magitude of the vector.
 *
 * @param axis
 * @return
 */
Matrix3 RotationMatrix(Vector3 axis) {
    double theta = std::sqrt(Dot(axis, axis));
    double cos_theta = std::cos(theta);
    double sin_theta = std::sin(theta);

    axis /= theta;

    Matrix3 rot;

    rot.xx = cos_theta + axis.x * axis.x * (1.0 - cos_theta);
    rot.yy = cos_theta + axis.y * axis.y * (1.0 - cos_theta);
    rot.zz = cos_theta + axis.z * axis.z * (1.0 - cos_theta);

    rot.xy = axis.x * axis.y * (1.0 - cos_theta) - axis.z * sin_theta;
    rot.yz = axis.y * axis.z * (1.0 - cos_theta) - axis.x * sin_theta;
    rot.zx = axis.z * axis.x * (1.0 - cos_theta) - axis.y * sin_theta;

    rot.yx = axis.x * axis.y * (1.0 - cos_theta) + axis.z * sin_theta;
    rot.zy = axis.y * axis.z * (1.0 - cos_theta) + axis.x * sin_theta;
    rot.xz = axis.z * axis.x * (1.0 - cos_theta) + axis.y * sin_theta;

    return rot;
}
