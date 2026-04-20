#include "core/cell_solver.hpp"

#include <algorithm>

namespace cellgen {

CellSolution solveRobotPlacement(const BeltLines& belts,
                                 float demo_radius_m,
                                 float cell_half_width_m)
{
    CellSolution sol{};
    sol.demo_radius_m = demo_radius_m;

    // ── 1. Intersection ───────────────────────────────────────────────────────
    // L1 : z = belts.input_z_m   (input belt,  direction (1,0) in XZ)
    // L2 : x = belts.output_x_m  (output belt, direction (0,1) in XZ)
    //
    // Two axis-aligned perpendicular lines always intersect at one point.
    sol.intersection_x_m = belts.output_x_m;
    sol.intersection_z_m = belts.input_z_m;

    // ── 2. Inscribed-circle centre (NW corner) ────────────────────────────────
    // The two lines create four 90° corners at the intersection point.
    // We choose the NW corner  { x ≤ output_x } ∩ { z ≤ input_z }  because it
    // places the robot between the input-belt inner end (which lies at some
    // x > −cell_half_width) and the output belt, on the north side of the input
    // belt where arm clearance is greatest.
    //
    // For a circle of radius r inscribed in a right-angle corner:
    //   centre = corner_vertex + r * bisector_unit
    //   bisector_unit (NW) = (−1, −1) / √2
    //   distance along bisector from vertex = r / sin(45°) = r√2
    //   ⟹  centre = (output_x − r,  input_z − r)
    sol.robot_x_m = belts.output_x_m - demo_radius_m;
    sol.robot_z_m = belts.input_z_m  - demo_radius_m;

    // ── 3. Input-belt inner extent ────────────────────────────────────────────
    // The tangent point from the robot centre onto L1 is the foot of the
    // perpendicular from (robot_x, robot_z) to the line  z = input_z:
    //   tangent_point = (robot_x,  input_z)
    //
    // The input belt starts at the west-wall face x = −cell_half_width_m and
    // extends eastward by inner_extent.  Its inner end is at:
    //   x_inner = −cell_half_width_m + inner_extent
    //
    // Require x_inner ≥ robot_x  so the belt reaches the robot's tangent point:
    //   inner_extent ≥ cell_half_width_m + robot_x_m
    const float required = cell_half_width_m + sol.robot_x_m;
    sol.input_belt_inner_m = std::max(required, 0.3f);  // at least 300 mm

    return sol;
}

} // namespace cellgen
