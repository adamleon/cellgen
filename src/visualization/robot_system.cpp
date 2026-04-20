#include "visualization/robot_system.hpp"

#include <threepp/loaders/AssimpLoader.hpp>
#include <threepp/loaders/URDFLoader.hpp>

#include <algorithm>
#include <iostream>

using namespace threepp;
using namespace cellgen;

RobotSystem::RobotSystem(Scene& scene, const Spec& spec)
    : scene_(scene), spec_(spec) {

    URDFLoader urdf;
    urdf.setGeometryLoader(std::make_shared<AssimpLoader>());

    robot_ = urdf.load(spec_.urdf_path);

    if (!robot_) {
        std::cerr << "[RobotSystem] Failed to load URDF: " << spec_.urdf_path << "\n";
        return;
    }

    robot_->showColliders(false);

    // Assimp 5.x embeds a Rx(-PI/2) in the root node of every Z_UP COLLADA
    // file, and the threepp URDFLoader bakes an additional Rx(-PI/2) into
    // .dae geometry vertices.  Together they produce Rx(-PI) on the mesh
    // data.  When the robot-level Rx(-PI/2) is then applied (step below),
    // the total is Rx(-3PI/2) = Rx(PI/2), which flips meshes underground.
    // Applying Rx(PI) here cancels those two extra conversions so that
    // only the robot rotation remains as the single Z-up → Y-up transform.
    robot_->traverseType<Mesh>([](Mesh& mesh) {
        auto geo = mesh.geometry();
        geo->boundingBox.reset();
        geo->boundingSphere.reset();
        geo->applyMatrix4(Matrix4().makeRotationX(math::PI));
        mesh.material()->side = Side::Double;
    });

    // URDF is Z-up; threepp scene is Y-up.  A -PI/2 rotation around X
    // brings the robot upright so it stands on the Y=0 floor plane.
    robot_->rotation.x = -math::PI / 2.0f;

    std::cout << "[RobotSystem] Loaded " << spec_.name
              << "  DOF=" << robot_->numDOF()
              << "  reach=" << spec_.reach_mm << " mm\n";
}

RobotSystem::~RobotSystem() {
    if (in_scene_) {
        if (robot_)       scene_.remove(*robot_);
        if (work_circle_) scene_.remove(*work_circle_);
    }
}

// ── public ────────────────────────────────────────────────────────────────────

void RobotSystem::setPosition(float x_m, float z_m, float work_radius_m) {
    if (robot_) {
        robot_->position.x = x_m;
        robot_->position.z = z_m;
    }

    // Rebuild work-circle if radius changed (or on first call).
    if (work_radius_m != work_radius_m_) {
        if (work_circle_ && in_scene_) scene_.remove(*work_circle_);
        work_circle_.reset();
        work_radius_m_ = work_radius_m;

        if (work_radius_m > 0.0f) {
            auto geo  = CircleGeometry::create(work_radius_m, 64);
            auto mat  = MeshBasicMaterial::create();
            mat->color       = Color(0x22ff88);
            mat->transparent = true;
            mat->opacity     = 0.15f;
            mat->depthWrite  = false;
            mat->side        = Side::Double;
            work_circle_ = Mesh::create(geo, mat);
            // CircleGeometry lies in XY; rotate to lie flat on the XZ floor.
            work_circle_->rotation.x = -math::PI / 2.0f;
            if (in_scene_) scene_.add(*work_circle_);
        }
    }

    if (work_circle_) {
        work_circle_->position.set(x_m, 0.005f, z_m);
    }
}

void RobotSystem::onCellResized(int width_mm, int depth_mm) {
    if (!robot_) return;
    if (width_mm == last_width_mm_ && depth_mm == last_depth_mm_) return;

    last_width_mm_ = width_mm;
    last_depth_mm_ = depth_mm;

    const bool fits = fitsInCell(width_mm, depth_mm);

    if (fits && !in_scene_) {
        scene_.add(*robot_);
        if (work_circle_) scene_.add(*work_circle_);
        in_scene_ = true;
        std::cout << "[RobotSystem] " << spec_.name << " added to scene"
                  << "  (" << width_mm << " x " << depth_mm << " mm)\n";
    } else if (!fits && in_scene_) {
        scene_.remove(*robot_);
        if (work_circle_) scene_.remove(*work_circle_);
        in_scene_ = false;
        std::cout << "[RobotSystem] " << spec_.name << " removed — cell too small"
                  << "  (" << width_mm << " x " << depth_mm << " mm"
                  << ", need >" << spec_.reach_mm << " mm on short side)\n";
    }
}

// ── private ───────────────────────────────────────────────────────────────────

bool RobotSystem::fitsInCell(int width_mm, int depth_mm) const {
    // The robot is centred in the cell.  The short side must exceed the rated
    // reach so the arm can sweep without clipping a fence wall.
    return std::min(width_mm, depth_mm) > spec_.reach_mm;
}
