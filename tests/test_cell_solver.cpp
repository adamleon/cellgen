#include <gtest/gtest.h>
#include "core/cell_solver.hpp"

#include <cmath>

using namespace cellgen;

// ── Intersection ──────────────────────────────────────────────────────────────

TEST(CellSolver, IntersectionAxisAligned) {
    BeltLines belts{ .input_z_m = 0.0f, .output_x_m = 0.5f };
    auto sol = solveRobotPlacement(belts, 0.5f, 2.0f);

    EXPECT_FLOAT_EQ(sol.intersection_x_m, 0.5f);
    EXPECT_FLOAT_EQ(sol.intersection_z_m, 0.0f);
}

TEST(CellSolver, IntersectionOffCenter) {
    BeltLines belts{ .input_z_m = 0.2f, .output_x_m = 0.8f };
    auto sol = solveRobotPlacement(belts, 0.3f, 2.5f);

    EXPECT_FLOAT_EQ(sol.intersection_x_m, 0.8f);
    EXPECT_FLOAT_EQ(sol.intersection_z_m, 0.2f);
}

// ── Robot position: inscribed circle in NW corner ────────────────────────────

TEST(CellSolver, RobotPositionPerpendicular) {
    // Robot should be at (output_x - r, input_z - r)
    BeltLines belts{ .input_z_m = 0.0f, .output_x_m = 0.5f };
    constexpr float r = 0.5f;
    auto sol = solveRobotPlacement(belts, r, 2.0f);

    EXPECT_FLOAT_EQ(sol.robot_x_m, 0.5f - r);   // west  of output belt
    EXPECT_FLOAT_EQ(sol.robot_z_m, 0.0f - r);   // north of input  belt
}

TEST(CellSolver, RobotDistanceFromBelts) {
    // By construction the robot must be exactly demo_radius_m from each line.
    BeltLines belts{ .input_z_m = 0.1f, .output_x_m = 0.6f };
    constexpr float r = 0.4f;
    auto sol = solveRobotPlacement(belts, r, 2.0f);

    // Distance from robot to input belt  (z = input_z)
    const float dist_input  = std::abs(sol.robot_z_m - belts.input_z_m);
    // Distance from robot to output belt (x = output_x)
    const float dist_output = std::abs(sol.robot_x_m - belts.output_x_m);

    EXPECT_NEAR(dist_input,  r, 1e-5f);
    EXPECT_NEAR(dist_output, r, 1e-5f);
}

// ── Input belt inner extent ───────────────────────────────────────────────────

TEST(CellSolver, InputBeltReachesRobotTangentPoint) {
    // Inner end of the belt must reach x = robot_x (the tangent point on L1).
    BeltLines belts{ .input_z_m = 0.0f, .output_x_m = 0.5f };
    constexpr float r  = 0.5f;
    constexpr float hw = 2.0f;
    auto sol = solveRobotPlacement(belts, r, hw);

    // Belt inner end:  x_inner = -hw + inner_extent
    const float x_inner = -hw + sol.input_belt_inner_m;
    EXPECT_NEAR(x_inner, sol.robot_x_m, 1e-5f);
}

TEST(CellSolver, InputBeltMinimumExtent) {
    // Even with a tiny cell the inner extent is clamped to at least 0.3 m.
    BeltLines belts{ .input_z_m = 0.0f, .output_x_m = -2.0f }; // robot far west
    auto sol = solveRobotPlacement(belts, 0.5f, 0.5f);

    EXPECT_GE(sol.input_belt_inner_m, 0.3f);
}

TEST(CellSolver, DemoRadiusStoredInSolution) {
    BeltLines belts{ .input_z_m = 0.0f, .output_x_m = 0.5f };
    constexpr float r = 0.75f;
    auto sol = solveRobotPlacement(belts, r, 2.0f);
    EXPECT_FLOAT_EQ(sol.demo_radius_m, r);
}
