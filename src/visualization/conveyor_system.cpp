#include "visualization/conveyor_system.hpp"

using namespace threepp;
using namespace cellgen;

namespace {

constexpr unsigned int kColorBeltBody    = 0x2c2c2c;
constexpr unsigned int kColorBeltSurface = 0x484848;

// Build a belt box with a thin surface stripe.
// length_m runs along the belt direction; width_m is across it.
std::shared_ptr<Object3D> makeBeltMesh(float length_m,
                                        float width_m,
                                        float thickness_m,
                                        float surface_h_m,
                                        bool  runs_along_x)
{
    auto group = Group::create();

    const float body_cy   = surface_h_m - thickness_m * 0.5f;
    const float surface_t = 0.012f;
    const float surface_cy = surface_h_m - surface_t * 0.5f;

    {
        auto geo = runs_along_x
            ? BoxGeometry::create(length_m, thickness_m, width_m)
            : BoxGeometry::create(width_m,  thickness_m, length_m);
        auto mat = MeshPhongMaterial::create();
        mat->color     = Color(kColorBeltBody);
        mat->shininess = 30;
        auto mesh = Mesh::create(geo, mat);
        mesh->position.y = body_cy;
        group->add(mesh);
    }
    {
        const float stripe_w = width_m * 0.80f;
        auto geo = runs_along_x
            ? BoxGeometry::create(length_m, surface_t, stripe_w)
            : BoxGeometry::create(stripe_w, surface_t, length_m);
        auto mat = MeshPhongMaterial::create();
        mat->color     = Color(kColorBeltSurface);
        mat->shininess = 15;
        auto mesh = Mesh::create(geo, mat);
        mesh->position.y = surface_cy;
        group->add(mesh);
    }

    return group;
}

} // namespace

// ---------------------------------------------------------------------------

ConveyorSystem::ConveyorSystem(Scene& scene, WallSystem& walls,
                               int initial_width_mm, int initial_depth_mm)
    : scene_(scene), walls_(walls)
{
    rebuild(initial_width_mm, initial_depth_mm);
}

ConveyorSystem::~ConveyorSystem() {
    if (input_belt_)  scene_.remove(*input_belt_);
    if (output_belt_) scene_.remove(*output_belt_);
}

void ConveyorSystem::onCellResized(int width_mm, int depth_mm) {
    rebuild(width_mm, depth_mm);
}

// ---------------------------------------------------------------------------

void ConveyorSystem::rebuild(int width_mm, int depth_mm) {
    if (input_belt_)  { scene_.remove(*input_belt_);  input_belt_.reset();  }
    if (output_belt_) { scene_.remove(*output_belt_); output_belt_.reset(); }

    const float hw = width_mm  * 0.0005f;
    const float hd = depth_mm  * 0.0005f;
    const float opening_w = kBeltWidth_m + 2.0f * kOpeningClearance;

    // ── Step 1: register WallComponents with preferred positions ─────────────
    // setComponents() triggers buildAllWalls(), which solves all fence segments
    // and writes the snapped actual_center_m back into each component.
    walls_.setComponents({
        { WEST,  { "input_belt",    opening_w, kInputOffsetZ_m  } },
        { NORTH, { "output_belt",   opening_w, kOutputOffsetX_m } },
        { SOUTH, { "output_belt",   opening_w, kOutputOffsetX_m } },
    });

    // ── Step 2: read back snapped positions ──────────────────────────────────
    // actual_center_m is in wall-axis world coords:
    //   WEST  wall → world Z   (negative = north)
    //   NORTH wall → world X   (positive = east)
    float input_z  = kInputOffsetZ_m;
    float output_x = kOutputOffsetX_m;

    for (const auto& c : walls_.componentsForWall(WEST))
        if (c.id == "input_belt")  input_z  = c.actual_center_m;
    for (const auto& c : walls_.componentsForWall(NORTH))
        if (c.id == "output_belt") output_x = c.actual_center_m;

    // ── Step 3: build belt meshes at actual positions ─────────────────────────

    // Input belt (WEST wall → runs along X, east-west).
    // Extends kExternalExtent_m outside and kInternalExtent_m inside the cell.
    {
        const float x_start = -(hw + kExternalExtent_m);
        const float x_end   = -(hw - kInternalExtent_m);
        const float length  = x_end - x_start;
        const float cx      = (x_start + x_end) * 0.5f;

        input_belt_ = makeBeltMesh(length, kBeltWidth_m, kBeltThickness_m,
                                   kBeltSurfaceH_m, /*runs_along_x=*/true);
        input_belt_->position.set(cx, 0.0f, input_z);
        scene_.add(input_belt_);
    }

    // Output belt (SOUTH → NORTH, runs along Z, north-south).
    // Empty pallets enter from the south; full pallets exit through the north.
    {
        const float z_start = -(hd + kExternalExtent_m);
        const float z_end   = +(hd + kExternalExtent_m);
        const float length  = z_end - z_start;

        output_belt_ = makeBeltMesh(length, kBeltWidth_m, kBeltThickness_m,
                                    kBeltSurfaceH_m, /*runs_along_x=*/false);
        output_belt_->position.set(output_x, 0.0f, 0.0f);
        scene_.add(output_belt_);
    }
}
