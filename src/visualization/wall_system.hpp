#pragma once

#include <threepp/threepp.hpp>
#include <threepp/controls/OrbitControls.hpp>
#include <threepp/input/MouseListener.hpp>
#include <threepp/loaders/OBJLoader.hpp>
#include <threepp/math/Plane.hpp>

#include "core/fence_catalog.hpp"

#include <array>
#include <filesystem>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace cellgen {

// ── Wall identity ─────────────────────────────────────────────────────────────
enum WallId { NORTH = 0, SOUTH = 1, EAST = 2, WEST = 3 };

// ── WallComponent ─────────────────────────────────────────────────────────────
// A named slot that claims a fixed-width opening in a fence wall.
// Examples: conveyor-belt penetration, door, window, safety-gate.
//
// preferred_center_m: desired opening centre in wall-axis world coordinates.
//   • N/S walls run along X  →  preferred_center_m is a world-X value.
//   • E/W walls run along Z  →  preferred_center_m is a world-Z value.
//
// After WallSystem::setComponents() is called, actual_center_m is written back
// with the snapped/nudged position that results from the panel-layout pass.
// Callers (e.g. ConveyorSystem) can read this to align belt meshes exactly.
struct WallComponent {
    std::string id;                   // unique identifier within the wall
    float       width_m;              // opening width along the wall run direction
    float       preferred_center_m;   // desired centre (world coord along run axis)
    float       actual_center_m = 0.0f; // set by layout algorithm after solving
};

// ── WallSystem ────────────────────────────────────────────────────────────────
// Manages the four cell walls: builds panel meshes, handles wall selection,
// and provides a drag gizmo to resize the cell.
//
// Coordinate system (metres, Y-up):
//   North wall  →  z = -depth/2  (along X)
//   South wall  →  z = +depth/2  (along X)
//   East  wall  →  x = +width/2  (along Z)
//   West  wall  →  x = -width/2  (along Z)
//
// Wall layout with components
// ────────────────────────────
// A wall is a sequence of alternating fence segments and components:
//
//   [fence seg 0] [comp 0] [fence seg 1] [comp 1] … [fence seg N]
//
// Each fence segment is solved independently by FenceCatalog::solve().
// Components are placed at their preferred position and may be nudged by up to
// one panel-width so the neighbouring segments each snap to a valid panel combo.
// The actual snapped position is written to WallComponent::actual_center_m.
class WallSystem : public threepp::MouseListener {
public:
    // catalog_dir: directory containing catalog.json and OBJ files.
    WallSystem(threepp::Scene& scene,
               threepp::Camera& camera,
               threepp::Canvas& canvas,
               threepp::OrbitControls& controls,
               std::filesystem::path catalog_dir,
               int width_mm = 4000,
               int depth_mm = 3000);

    ~WallSystem() override;

    int widthMm() const { return width_mm_; }
    int depthMm() const { return depth_mm_; }

    // Replace all components across every wall, then rebuild.
    // Each pair is {WallId, WallComponent}.  After the call,
    // WallComponent::actual_center_m in the stored list is updated.
    // Use componentsForWall() to read them back.
    void setComponents(std::vector<std::pair<WallId, WallComponent>> all_components);

    // Read-only access to the (post-layout) component list for a wall.
    const std::vector<WallComponent>& componentsForWall(WallId wall) const {
        return components_[wall];
    }

private:
    struct WallData {
        WallId id;
        std::shared_ptr<threepp::Object3D> group;
        WallLayout layout;           // representative layout (first non-empty segment)
        float visual_half_m = 0.0f;  // true visual half-length after layout pass
        std::vector<std::shared_ptr<threepp::Object3D>> panel_groups;
        std::vector<std::shared_ptr<threepp::MeshPhongMaterial>> materials;
    };

    enum class State { IDLE, SELECTED, DRAGGING };

    // ── Build ────────────────────────────────────────────────────────────────
    void buildAllWalls();
    void buildWall(WallData& wall);

    // ── Gizmo ────────────────────────────────────────────────────────────────
    void showGizmo(WallId id);
    void hideGizmo();
    void repositionGizmo(WallId id);

    threepp::Vector3 gizmoWorldPos(WallId id) const;
    threepp::Vector3 wallOutwardNormal(WallId id) const;

    // ── Selection ────────────────────────────────────────────────────────────
    void highlightWall(WallId id, bool on);

    // ── MouseListener ────────────────────────────────────────────────────────
    void onMouseDown(int button, const threepp::Vector2& pos) override;
    void onMouseMove(const threepp::Vector2& pos) override;
    void onMouseUp(int button, const threepp::Vector2& pos) override;

    // ── Utility ──────────────────────────────────────────────────────────────
    threepp::Vector2 toNDC(const threepp::Vector2& screen) const;

    // ── Members ──────────────────────────────────────────────────────────────
    threepp::Scene&         scene_;
    threepp::Camera&        camera_;
    threepp::Canvas&        canvas_;
    threepp::OrbitControls& controls_;
    std::filesystem::path   catalog_dir_;
    FenceCatalog            catalog_;
    std::unordered_map<std::string, std::shared_ptr<threepp::Group>> obj_cache_;

    int width_mm_;
    int depth_mm_;
    static constexpr int   kWallHeightMm = 750;
    static constexpr int   kMinDimMm     = 500;

    std::array<WallData, 4>                   walls_;
    std::array<std::vector<WallComponent>, 4> components_; // per-wall component lists

    std::unordered_map<threepp::Object3D*, WallId> panel_to_wall_;

    std::shared_ptr<threepp::Object3D> gizmo_root_;
    std::shared_ptr<threepp::Mesh>     gizmo_handle_;

    State state_{State::IDLE};
    std::optional<WallId> selected_wall_;

    WallId           drag_wall_{};
    threepp::Plane   drag_plane_;
    threepp::Vector3 drag_start_hit_;
    int              drag_start_dim_;

    threepp::Raycaster raycaster_;
};

} // namespace cellgen
