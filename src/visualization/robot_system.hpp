#pragma once

#include <threepp/threepp.hpp>
#include <threepp/objects/Robot.hpp>

#include <filesystem>
#include <memory>
#include <string>

namespace cellgen {

// Loads a single URDF robot, places it at the cell centre, and removes it
// from the scene whenever the cell is too small to contain it.
//
// "Fits" definition: the smaller of (width, depth) must exceed the robot's
// rated reach.  This ensures the robot can swing its full arm without hitting
// a wall on the short side.
//
// Call onCellResized() whenever width_mm or depth_mm changes (e.g. from the
// main animate loop); the class handles add/remove automatically.
class RobotSystem {
public:
    struct Spec {
        std::filesystem::path urdf_path;  // absolute path to the .urdf file
        int                   reach_mm;   // rated arm reach (determines min cell)
        std::string           name;       // display / log name
    };

    RobotSystem(threepp::Scene& scene, const Spec& spec);
    ~RobotSystem();

    // Call once per frame (or whenever dimensions change).
    void onCellResized(int width_mm, int depth_mm);

    bool isInScene() const { return in_scene_; }
    bool loadedOk()  const { return robot_ != nullptr; }

private:
    bool fitsInCell(int width_mm, int depth_mm) const;

    threepp::Scene& scene_;
    Spec            spec_;

    std::shared_ptr<threepp::Robot> robot_;
    bool in_scene_ = false;

    int last_width_mm_  = 0;
    int last_depth_mm_  = 0;
};

} // namespace cellgen
