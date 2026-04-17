#pragma once

#include <threepp/threepp.hpp>
#include "visualization/wall_system.hpp"

namespace cellgen {

// Manages the two conveyor belts in the palletizing cell.
//
//  Input belt  – enters through the WEST wall, runs east-west.
//                Delivers work pieces to the robot pick zone.
//
//  Output belt – enters through the SOUTH wall and exits through the NORTH
//                wall, runs north-south.
//                Empty pallets arrive from the south; full pallets leave
//                through the north.
//
// Each belt is represented as a flat box (rectangle) as a placeholder.
// Belt positions are stored as (wall_id, offset_along_wall) so that, in the
// future, the hole in the cell wall can be made draggable: the user moves the
// hole, and both the wall openings and belt geometry update automatically.
//
// Call onCellResized() whenever the cell dimensions change so belt lengths
// and the wall openings stay in sync.
class ConveyorSystem {
public:
    // ── Tweakable layout constants ───────────────────────────────────────────
    // These will eventually become runtime-configurable per-belt properties.

    // Input belt: world Z position of belt centre (negative = north of origin).
    static constexpr float kInputOffsetZ_m   =  0.0f;
    // Output belt: world X position of belt centre (positive = east of origin).
    static constexpr float kOutputOffsetX_m  =  0.5f;

    static constexpr float kBeltWidth_m      = 0.60f;   // 600 mm
    static constexpr float kBeltThickness_m  = 0.08f;   // 80 mm visual depth
    static constexpr float kBeltSurfaceH_m   = 0.40f;   // 400 mm above floor
    // How far each belt extends outside the cell walls.
    static constexpr float kExternalExtent_m = 1.00f;
    // How far the input belt reaches inside the cell from the west wall.
    static constexpr float kInternalExtent_m = 1.50f;
    // Extra clearance (per side) added to the opening width in the wall.
    static constexpr float kOpeningClearance = 0.05f;

    ConveyorSystem(threepp::Scene& scene,
                   WallSystem&     walls,
                   int             initial_width_mm,
                   int             initial_depth_mm);
    ~ConveyorSystem();

    // Rebuilds belt geometry and re-registers wall openings.
    // Must be called whenever the cell is resized.
    void onCellResized(int width_mm, int depth_mm);

private:
    void rebuild(int width_mm, int depth_mm);

    threepp::Scene& scene_;
    WallSystem&     walls_;

    std::shared_ptr<threepp::Object3D> input_belt_;
    std::shared_ptr<threepp::Object3D> output_belt_;
};

} // namespace cellgen
