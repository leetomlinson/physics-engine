/******************************************************************************
 * Copyright (C) 2017 Lee Tomlinson
 *
 * This file is part of Physics Engine.
 *
 * Physics Engine is made available under the terms of the
 * Attribution-NonCommercial-NoDerivs 3.0 Unported (CC BY-NC-ND 3.0)
 * licence: https://creativecommons.org/licenses/by-nc-nd/3.0/
 ******************************************************************************/

#ifndef PHYSICS_ENGINE_LINALG_H
#define PHYSICS_ENGINE_LINALG_H


class Vector3 {
public:
    double x;
    double y;
    double z;

    Vector3() : x{0.0}, y{0.0}, z{0.0} {
    }

    Vector3(double x, double y, double z) : x{x}, y{y}, z{z} {
    }

    /**
     * Defines multiplication of a vector by a scalar.
     *
     * @param scalar
     * @return
     */
    Vector3 const &operator*=(double scalar) {
        this->x *= scalar;
        this->y *= scalar;
        this->z *= scalar;
        return (*this);
    }

    /**
     * Defines division of a vector by a scalar.
     *
     * @param scalar
     * @return
     */
    Vector3 const &operator/=(double scalar) {
        this->x /= scalar;
        this->y /= scalar;
        this->z /= scalar;
        return (*this);
    }

    /**
     * Defines vector addition.
     *
     * @param vec
     * @return
     */
    Vector3 const &operator+=(Vector3 const &vec) {
        this->x += vec.x;
        this->y += vec.y;
        this->z += vec.z;
        return (*this);
    }

    /**
     * Defines vector subtraction.
     *
     * @param vec
     * @return
     */
    Vector3 const &operator-=(Vector3 const &vec) {
        this->x -= vec.x;
        this->y -= vec.y;
        this->z -= vec.z;
        return (*this);
    }

    bool operator==(Vector3 const &rhs) const {
        return this->x == rhs.x && this->y == rhs.y && this->z == rhs.z;
    }
};


class Matrix3 {
public:
    double xx;
    double xy;
    double xz;
    double yx;
    double yy;
    double yz;
    double zx;
    double zy;
    double zz;

    Matrix3() : xx{0.0}, xy{0.0}, xz{0.0},
                yx{0.0}, yy{0.0}, yz{0.0},
                zx{0.0}, zy{0.0}, zz{0.0} {
    }

    /**
     * Constructs the matrix from three column vectors.
     *
     * @param vec1
     * @param vec2
     * @param vec3
     */
    Matrix3(Vector3 const &vec1, Vector3 const &vec2, Vector3 const &vec3) {
        xx = vec1.x;
        yx = vec1.y;
        zx = vec1.z;
        xy = vec2.x;
        yy = vec2.y;
        zy = vec2.z;
        xz = vec3.x;
        yz = vec3.y;
        zz = vec3.z;
    }

    /**
     * Constructs a diagonal matrix from one numbers.
     *
     * @param number
     */
    Matrix3(double number) : xx{number}, xy{0.0}, xz{0.0},
                             yx{0.0}, yy{number}, yz{0.0},
                             zx{0.0}, zy{0.0}, zz{number} {

    }

    /**
     * Constructs a diagonal matrix from three numbers.
     *
     * @param xx
     * @param yy
     * @param zz
     */
    Matrix3(double xx, double yy, double zz) : xx{xx}, xy{0.0}, xz{0.0},
                                               yx{0.0}, yy{yy}, yz{0.0},
                                               zx{0.0}, zy{0.0}, zz{zz} {

    }

    Matrix3(double xx, double xy, double xz,
            double yx, double yy, double yz,
            double zx, double zy, double zz) :
            xx{xx}, xy{xy}, xz{xz},
            yx{yx}, yy{yy}, yz{yz},
            zx{zx}, zy{zy}, zz{zz} {
    }

    // Use implicitly defined copy constructor.

    bool operator==(Matrix3 const &rhs) const {
        return this->xx == rhs.xx && this->xy == rhs.xy && this->xz == rhs.xz &&
               this->yx == rhs.yx && this->yy == rhs.yy && this->yz == rhs.yz &&
               this->zx == rhs.zx && this->zy == rhs.zy && this->zz == rhs.zz;
    }
};


Vector3 operator*(double scalar, Vector3 vec);

Vector3 operator/(Vector3 vec, double scalar);

Vector3 operator+(Vector3 vec1, Vector3 const &vec2);

Vector3 operator-(Vector3 vec1, Vector3 const &vec2);

double Dot(Vector3 const &vec1, Vector3 const &vec2);

Vector3 Cross(Vector3 const &vec1, Vector3 const &vec2);

Matrix3 Transpose(Matrix3 const &mat);

Vector3 Dot(Matrix3 const &mat, Vector3 const &vec);

Matrix3 RotationMatrix(Vector3 axis);


#endif //PHYSICS_ENGINE_LINALG_H
