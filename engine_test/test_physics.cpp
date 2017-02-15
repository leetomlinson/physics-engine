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
#include "collision.h"
#include "physics.h"


TEST(test_solve_inhomogeneous, basic_test) {
    Coeffs3 coeffs = {17, 42, 23, 25, 47, 5, 6, 29, 1};
    Tuple3 rhs = {33, 47, 16};
    Tuple3 soln_actual = SolveInhomogeneous(coeffs, rhs);
    Tuple3 xyz_expected = {12205.0 / 8733.0, 2318.0 / 8733.0, -724.0 / 8733.0};
    EXPECT_EQ(soln_actual.A, xyz_expected.A);
    EXPECT_EQ(soln_actual.B, xyz_expected.B);
    EXPECT_EQ(soln_actual.C, xyz_expected.C);
    EXPECT_EQ(soln_actual, xyz_expected);
}

TEST(test_solve_inhomogeneous, more_complicated_test) {
    Coeffs3 coeffs = {-3.5970, 4.1820, -1.3020e+1,
                      -3.1140, -1.0260e+1, -2.9010,
                      7.0840, 4.0050e-2, -5.3690};
    Tuple3 rhs = {5.1210, -2.9520, -3.3500};
    Tuple3 soln_actual = SolveInhomogeneous(coeffs, rhs);
    Tuple3 xyz_expected = {-1569111651149.0 / 2886928659313.0,
                           1380018966880.0 / 2886928659313.0,
                           -258727168638.0 / 2886928659313.0};
    EXPECT_DOUBLE_EQ(soln_actual.A, xyz_expected.A);
    EXPECT_DOUBLE_EQ(soln_actual.B, xyz_expected.B);
    EXPECT_DOUBLE_EQ(soln_actual.C, xyz_expected.C);
    EXPECT_EQ(soln_actual, xyz_expected);
}

TEST(test_line_intersects_plane, basic_test) {
    // TODO: Finish code/test
    Line line;
    line.start = {0.0, 0.0, 0.0};
    line.end = {3.0, 3.0, 3.0};
    Triangle triangle;
    triangle.vtx_a = {3.0, 0.0, 0.0};
    triangle.vtx_b = {0.0, 3.0, 0.0};
    triangle.vtx_c = {0.0, 0.0, 3.0};
    Tuple3 res = LineIntercectsPlane(line, triangle);
    FAIL();
}

TEST(test_segment_intersects_triangle, basic_test) {
    Line line;
    line.start = {0.0, 0.0, 0.0};
    line.end = {3.0, 3.0, 3.0};
    Triangle triangle;
    triangle.vtx_a = {3.0, 0.0, 0.0};
    triangle.vtx_b = {0.0, 3.0, 0.0};
    triangle.vtx_c = {0.0, 0.0, 3.0};
    Intersection i = GetSegmentTriangleIntersection(line, triangle);
    EXPECT_EQ(i.intercept.A, 1.0);
    EXPECT_EQ(i.intercept.B, 1.0);
    EXPECT_EQ(i.intercept.C, 1.0);
    EXPECT_EQ(i.overlap, true);
}

TEST(test_segment_intersects_triangle, basic_test_2) {
    Line line;
    line.start = {-1.0, 1.25, 1.25};
    line.end = {1.0, 1.25, 1.25};
    Triangle triangle;
    triangle.vtx_a = {0.0, 1.0, 1.0};
    triangle.vtx_b = {0.0, 2.0, 1.0};
    triangle.vtx_c = {0.0, 1.0, 2.0};
    Intersection i = GetSegmentTriangleIntersection(line, triangle);
    EXPECT_EQ(i.intercept.A, 0.0);
    EXPECT_EQ(i.intercept.B, 1.25);
    EXPECT_EQ(i.intercept.C, 1.25);
    EXPECT_EQ(i.overlap, true);
}

TEST(test_segment_intersects_triangle, basic_test_no_intercept) {
    Line line;
    line.start = {-1.0, 1.75, 1.75};
    line.end = {1.0, 1.75, 1.75};
    Triangle triangle;
    triangle.vtx_a = {0.0, 1.0, 1.0};
    triangle.vtx_b = {0.0, 2.0, 1.0};
    triangle.vtx_c = {0.0, 1.0, 2.0};
    Intersection i = GetSegmentTriangleIntersection(line, triangle);
    EXPECT_EQ(i.overlap, false);
}

TEST(test_triangle_intersect, basic_test) {
    Triangle t1;
    t1.vtx_a = {1.0, 0.0, 0.0};
    t1.vtx_b = {0.0, 1.0, 0.0};
    t1.vtx_c = {0.0, 0.0, 1.0};
    Triangle t2;
    t2.vtx_a = {0.0, 0.0, 0.0};
    t2.vtx_b = {1.0, 1.0, 0.0};
    t2.vtx_c = {1.0, 1.0, 1.0};
    EXPECT_EQ(TrianglesIntersect(t1, t2), true);
}

TEST(test_triangle_intersect, basic_test_no_intersection) {
    Triangle t1;
    t1.vtx_a = {1.0, 0.0, 0.0};
    t1.vtx_b = {0.0, 1.0, 0.0};
    t1.vtx_c = {0.0, 0.0, 1.0};
    Triangle t2;
    t2.vtx_a = {0.5, 0.0, 0.0};
    t2.vtx_b = {0.0, 0.5, 0.0};
    t2.vtx_c = {0.0, 0.0, 0.5};
    EXPECT_EQ(TrianglesIntersect(t1, t2), false);
}

TEST(test_vector3, vector_addition) {
    Vector3 v1 = {1.0, 1.5, 0.6};
    Vector3 v2 = {2.0, 4.0, 8.0};
    auto v3 = v1 + v2;
    EXPECT_EQ(v3.x, 3.0);
    EXPECT_EQ(v3.y, 5.5);
    EXPECT_EQ(v3.z, 8.6);
}

TEST(test_vector3, vector_subtraction) {
    Vector3 v1 = {1.0, 1.5, 0.6};
    Vector3 v2 = {2.0, 4.0, 8.0};
    auto v3 = v1 - v2;
    EXPECT_EQ(v3.x, -1.0);
    EXPECT_EQ(v3.y, -2.5);
    EXPECT_EQ(v3.z, -7.4);
}


TEST(test_physics_component, test_update) {
    Particle pc(1.0);
    pc.SetVelocity(Vector3{-0.4, 2.5, 0.0});
    pc.SetPosition(Vector3{1.0, 1.2, -1.9});
    pc.Update(4.3);
    auto p = pc.GetPosition();
    EXPECT_EQ(p.x, -0.72);
    EXPECT_EQ(p.y, 11.95);
    EXPECT_EQ(p.z, -1.9);
}

TEST(test_physics_component, test_update_with_forces) {
    Particle pc(1.0);
    pc.SetMass(2.0);
    pc.SetPosition(Vector3{5.0, 0.0, 0.0});
    pc.SetVelocity(Vector3{3.0, 0.0, 0.0});
    pc.ApplyForce(Vector3{-19.6, 0.0, 0.0});
    pc.Update(6.0);
    EXPECT_DOUBLE_EQ(pc.GetPosition().x, -153.4);
    EXPECT_DOUBLE_EQ(pc.GetVelocity().x, -55.8);
}
