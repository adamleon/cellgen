#pragma once

namespace cellgen {

// Belt centre-line positions in the world XZ plane (Y-up coordinate system).
//   Input belt  runs E-W: the centre-line is the line  z = input_z_m.
//   Output belt runs N-S: the centre-line is the line  x = output_x_m.
struct BeltLines {
    float input_z_m;   // Z-coordinate of input  belt centre-line
    float output_x_m;  // X-coordinate of output belt centre-line
};

// Result produced by solveRobotPlacement().
struct CellSolution {
    // Where the two (infinite) belt lines cross.
    float intersection_x_m;
    float intersection_z_m;

    // Recommended robot-base centre (on the floor plane, Y = 0).
    float robot_x_m;
    float robot_z_m;

    // Radius of the tangent work-circle.
    // By construction this equals the perpendicular distance from the robot
    // centre to each belt centre-line.
    float demo_radius_m;

    // How far the input belt must reach *inside* the cell from the west wall.
    float input_belt_inner_m;
};

// Compute robot placement from belt geometry.
//
// Algorithm
// ---------
// 1. Intersect the two belt lines (always possible for the current
//    axis-aligned perpendicular layout: L1  z = const,  L2  x = const).
//
// 2. Place the robot at the centre of the circle inscribed in the most-acute
//    corner formed by the two lines.
//    For the current perpendicular layout all four corners are 90°, so the
//    "most acute" criterion degenerates to a fixed choice: the robot is placed
//    in the NW corner  { x ≤ output_x } ∩ { z ≤ input_z }  which positions it
//    between the inner end of the input belt and the output belt.
//
//    For two perpendicular lines the inscribed-circle centre is at distance r
//    from each line along the bisector:
//       robot_x = output_x − r
//       robot_z = input_z  − r
//
// 3. Set the input-belt inner extent so that the belt's inner end reaches the
//    robot's tangent point on the input belt (x = robot_x):
//       inner_extent = cell_half_width + robot_x
//
// Parameters
// ----------
//   belts              – snapped belt centre-line world coordinates
//   demo_radius_m      – desired tangent-circle radius (metres)
//   cell_half_width_m  – half the current cell E-W width (metres)
CellSolution solveRobotPlacement(const BeltLines& belts,
                                 float demo_radius_m,
                                 float cell_half_width_m);

} // namespace cellgen
