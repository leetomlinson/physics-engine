/******************************************************************************
 * Copyright (C) 2017 Lee Tomlinson
 *
 * This file is part of Physics Engine.
 *
 * Physics Engine is made available under the terms of the
 * Attribution-NonCommercial-NoDerivs 3.0 Unported (CC BY-NC-ND 3.0)
 * licence: https://creativecommons.org/licenses/by-nc-nd/3.0/
 ******************************************************************************/

#ifndef PHYSICS_ENGINE_COLLISION_H
#define PHYSICS_ENGINE_COLLISION_H

#include <glm/vec3.hpp>


/**
 * Inhomogeneous linear equations
 */

struct Coeffs3 {
    double a1, a2, a3, b1, b2, b3, c1, c2, c3;

    double &operator[](int i) {
        return ((double *) this)[i];
    }
};

// TODO: Consider calling this a Point
struct Tuple3 {
    double A, B, C;

    double &operator[](int i) {
        return ((double *) this)[i];
    }

    double const &operator[](int i) const {
        return ((double *) this)[i];
    }

    Tuple3 const &operator-=(Tuple3 const &rhs) {
        this->A -= rhs.A;
        this->B -= rhs.B;
        this->C -= rhs.C;
        return (*this);
    }

    Tuple3 const &operator+=(Tuple3 const &rhs) {
        this->A += rhs.A;
        this->B += rhs.B;
        this->C += rhs.C;
        return (*this);
    }

    Tuple3 const &operator*=(double rhs) {
        this->A *= rhs;
        this->B *= rhs;
        this->C *= rhs;
        return (*this);
    }
};

bool operator==(Tuple3 const &lhs, Tuple3 const &rhs) {
    return lhs.A == rhs.A && lhs.B == rhs.B && lhs.C && rhs.C;
}

Tuple3 operator-(Tuple3 lhs, Tuple3 const &rhs) {
    lhs -= rhs;
    return lhs;
}

Tuple3 operator+(Tuple3 lhs, Tuple3 const &rhs) {
    lhs += rhs;
    return lhs;
}

Tuple3 operator*(double coeff, Tuple3 rhs) {
    rhs *= coeff;
    return rhs;
}

/**
 * Interprets M as a 3-by-3 matrix in row-major representation and returns its determinant.
 *
 * @param M
 * @return
 */
double Det(Coeffs3 const &M) {
    return M.a1 * (M.b2 * M.c3 - M.b3 * M.c2) +
           M.a2 * (M.b3 * M.c1 - M.b1 * M.c3) +
           M.a3 * (M.b1 * M.c2 - M.b2 * M.c1);
}

/**
 * Solves a system of three inhomogeneous linear equations using Cramer's rule.
 *
 * @param coeffs
 * @param rhs
 * @return
 */
Tuple3 SolveInhomogeneous(Coeffs3 const &coeffs, Tuple3 const &rhs) {
    double det[4];

    for (int i = 0; i < 3; ++i) {
        Coeffs3 new_coeffs = coeffs;

        // Replace the ith column of new_coeffs with the column vector rhs
        for (int j = 0; j < 3; ++j) {
            new_coeffs[i + 3 * j] = rhs[j];
        }
        det[i] = Det(new_coeffs);
    }
    det[3] = Det(coeffs);

    Tuple3 soln = {det[0] / det[3], det[1] / det[3], det[2] / det[3]};
    return soln;
}

/**
 * Here is an algorithm which will determine if two objects are in collision.  That is, if the meshes of the two objects
 * overlap in some way.  There are subtles which arise, such as whether an object may be inside another object,
 * providing their meshes are not touching/overlapping.
 */

struct Line {
    // TODO: Consider renaming to Segment or Displacement
    Tuple3 start;
    Tuple3 end;
};

struct Triangle {
    Tuple3 vtx_a;
    Tuple3 vtx_b;
    Tuple3 vtx_c;
};

/**
 * TODO: Consider the semantics of the language used here.  Am I really concerned with a triangle?
 *
 * Returns the solution to A + b (B - A) + c (C - A) == S + t (E - S) in terms of the real parameters b, c, and t.  A,
 * B, C are three points in (3D) space which define a plane.  S and E are two points in space which define a line.  The
 * solution to this equation are the values of the parameters at the intersection between the line and the plane, which
 * is assumed to exist (no checks are performed for the line being parallel to the plane, yet).
 *
 * @param line
 * @param triangle
 * @return
 */
Tuple3 LineIntercectsPlane(Line line, Triangle triangle) {
    Coeffs3 coeffs;
    Tuple3 rhs;

    for (int row = 0; row < 3; ++row) {
        coeffs[row * 3] = triangle.vtx_b[row] - triangle.vtx_a[row]; // Column 1
        coeffs[row * 3 + 1] = triangle.vtx_c[row] - triangle.vtx_a[row]; // Column 2
        coeffs[row * 3 + 2] = line.start[row] - line.end[row]; // Column 3
        rhs[row] = line.start[row] - triangle.vtx_a[row];
    }

    return SolveInhomogeneous(coeffs, rhs);
}

struct Intersection {
    Tuple3 intercept;
    bool overlap;
};

/**
 *
 * @param line
 * @param triangle
 * @return
 */
Intersection GetSegmentTriangleIntersection(Line line, Triangle triangle) {
    Tuple3 params = LineIntercectsPlane(line, triangle);
    double b = params.A;
    double c = params.B;
    double t = params.C;

    Intersection out;
    out.overlap = false;

    if (b >= 0.0 && c >= 0.0 && b + c <= 1.0 && t >= 0.0 && t <= 1.0) {
        out.intercept = line.start + t * (line.end - line.start);
        out.overlap = true;
    }

    return out;
}

/**
 * Returns true if a line segment intersects a triangle.
 *
 * @param line
 * @param triangle
 * @return
 */
bool SegmentIntersectsTriangle(Line line, Triangle triangle) {
    Tuple3 params = LineIntercectsPlane(line, triangle);
    double b = params.A;
    double c = params.B;
    double t = params.C;
    return b >= 0.0 && c >= 0.0 && b + c <= 1.0 && t >= 0.0 && t <= 1.0;
}

/**
 * Returns true if two triangles intersect; i.e. if there is a set of points which exist simultaneously in the sets
 * comprising triangle 1 and triangle 2.
 *
 * @param t1
 * @param t2
 * @return
 */
bool TrianglesIntersect(Triangle t1, Triangle t2) {
    Line edge;
    edge.start = t1.vtx_a;

    // Does edge AB of Triangle t1 intersect Triangle t2?
    edge.end = t1.vtx_b;
    if (SegmentIntersectsTriangle(edge, t2)) return true;

    // Does edge AC of Triangle t1 intersect Triangle t2?
    edge.end = t1.vtx_c;
    if (SegmentIntersectsTriangle(edge, t2)) return true;

    // Does edge BC of Triangle t1 intersect Triangle t2?
    edge.start = t1.vtx_b;
    return SegmentIntersectsTriangle(edge, t2);

}


#endif //PHYSICS_ENGINE_COLLISION_H
