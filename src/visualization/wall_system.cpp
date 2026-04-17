#include "visualization/wall_system.hpp"

#include <threepp/loaders/OBJLoader.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>

using namespace threepp;
using namespace cellgen;

// ---------------------------------------------------------------------------
// Visual constants
// ---------------------------------------------------------------------------
namespace {

constexpr unsigned int kColorPanelA    = 0xa8a8a8;
constexpr unsigned int kColorPanelB    = 0x949494;
constexpr unsigned int kColorSelected  = 0x88bb88;
constexpr unsigned int kColorPost      = 0x707070;
constexpr unsigned int kColorGizmo     = 0xff8800;
constexpr unsigned int kColorHandle    = 0xffdd00;

constexpr float kPanelThickness = 0.04f;
constexpr float kPostSize       = 0.050f;
constexpr float kPanelGap       = kPostSize + 0.005f; // post + 5 mm clearance
constexpr float kGizmoOffset    = 0.50f;
constexpr float kHandleSize     = 0.18f;

std::shared_ptr<Object3D> makeArrow(std::shared_ptr<Mesh>& handleOut) {
    auto root = Object3D::create();

    auto shaftGeo = CylinderGeometry::create(0.02f, 0.02f, 0.40f, 12);
    auto shaftMat = MeshPhongMaterial::create();
    shaftMat->color = Color(kColorGizmo);
    auto shaft = Mesh::create(shaftGeo, shaftMat);
    shaft->position.y = 0.20f;
    root->add(shaft);

    auto headGeo = ConeGeometry::create(0.06f, 0.18f, 12);
    auto headMat = MeshPhongMaterial::create();
    headMat->color = Color(kColorGizmo);
    auto head = Mesh::create(headGeo, headMat);
    head->position.y = 0.40f + 0.09f;
    root->add(head);

    auto handleGeo = BoxGeometry::create(kHandleSize, kHandleSize, kHandleSize);
    auto handleMat = MeshPhongMaterial::create();
    handleMat->color   = Color(kColorHandle);
    handleMat->emissive = Color(0x443300);
    auto handle = Mesh::create(handleGeo, handleMat);
    handle->position.y = 0.40f + 0.18f + kHandleSize * 0.5f;
    root->add(handle);

    handleOut = handle;
    return root;
}

} // namespace

// ---------------------------------------------------------------------------
// Construction / destruction
// ---------------------------------------------------------------------------

WallSystem::WallSystem(Scene& scene, Camera& camera, Canvas& canvas,
                       OrbitControls& controls,
                       std::filesystem::path catalog_dir,
                       int width_mm, int depth_mm)
    : scene_(scene), camera_(camera), canvas_(canvas),
      controls_(controls),
      catalog_dir_(std::move(catalog_dir)),
      catalog_(catalog_dir_ / "catalog.json"),
      width_mm_(width_mm), depth_mm_(depth_mm)
{
    OBJLoader loader;
    loader.useCache = false;
    for (const auto& panel : catalog_.panels()) {
        if (panel.mesh_file.empty()) continue;
        if (obj_cache_.contains(panel.mesh_file)) continue;
        const auto path = catalog_dir_ / panel.mesh_file;
        auto group = loader.load(path, false);
        if (group) {
            obj_cache_[panel.mesh_file] = group;
        } else {
            std::cerr << "[WallSystem] Failed to load: " << path << "\n";
        }
    }

    for (int i = 0; i < 4; ++i) {
        walls_[i].id    = static_cast<WallId>(i);
        walls_[i].group = Object3D::create();
        scene_.add(walls_[i].group);
    }

    buildAllWalls();
    canvas_.addMouseListener(*this);
}

WallSystem::~WallSystem() {
    canvas_.removeMouseListener(*this);
    hideGizmo();
    for (auto& w : walls_) scene_.remove(*w.group);
}

// ---------------------------------------------------------------------------
// Component registration
// ---------------------------------------------------------------------------

void WallSystem::setComponents(
    std::vector<std::pair<WallId, WallComponent>> all_components)
{
    for (auto& v : components_) v.clear();
    for (auto& [wall, comp] : all_components)
        components_[wall].push_back(comp);

    buildAllWalls();
    if (selected_wall_.has_value()) repositionGizmo(*selected_wall_);
}

// ---------------------------------------------------------------------------
// Build
// ---------------------------------------------------------------------------

void WallSystem::buildAllWalls() {
    panel_to_wall_.clear();

    buildWall(walls_[NORTH]);
    buildWall(walls_[SOUTH]);
    buildWall(walls_[EAST]);
    buildWall(walls_[WEST]);

    // Position wall groups using their actual visual half-lengths so corners
    // always meet regardless of panel count or component positions.
    const float hw = walls_[NORTH].visual_half_m;
    const float hd = walls_[EAST].visual_half_m;
    walls_[NORTH].group->position.set( 0,  0, -hd);
    walls_[SOUTH].group->position.set( 0,  0,  hd);
    walls_[EAST].group->position.set(  hw, 0,   0);
    walls_[WEST].group->position.set( -hw, 0,   0);
}

// ---------------------------------------------------------------------------
// Wall build — segment-based layout
// ---------------------------------------------------------------------------
//
// Layout model:
//   [seg 0] [comp 0] [seg 1] [comp 1] … [comp N-1] [seg N]
//
// Each fence segment is solved independently by the catalog.  The boundary
// between a segment and a component is a post-in-gap, exactly like the
// boundary between two fence panels.
//
// Post-in-gap visual convention:
//   panel right-edge → kPanelGap (post centred here) → next panel left-edge
//   same convention applies at segment↔component boundaries.
//
// Cursor convention:
//   After placing a panel the cursor is at its right edge.
//   After an inter-panel (or boundary) gap the cursor is at the next element's
//   left edge.  Corner posts sit at ±visual_half (coincident with the wall
//   panel edges).
// ---------------------------------------------------------------------------

void WallSystem::buildWall(WallData& wall) {
    for (auto& pg : wall.panel_groups) wall.group->remove(*pg);
    wall.panel_groups.clear();
    wall.materials.clear();

    const bool  isXWall       = (wall.id == NORTH || wall.id == SOUTH);
    const int   target_len_mm = isXWall ? width_mm_ : depth_mm_;
    const float target_len_m  = target_len_mm * 0.001f;

    // ── Sort components left-to-right along the wall run axis ────────────────
    // Work on a local copy so the stored list preserves insertion order while
    // the layout computation uses sorted order.
    auto comps = components_[wall.id];
    std::sort(comps.begin(), comps.end(),
        [](const WallComponent& a, const WallComponent& b) {
            return a.preferred_center_m < b.preferred_center_m;
        });
    const int n_comps = static_cast<int>(comps.size());

    // ── Compute segment target lengths ────────────────────────────────────────
    // Component centres are in wall-axis world coords (same as cursor coords).
    // Wall extends from -target_len_m/2 to +target_len_m/2, i.e. the left edge
    // is at offset 0 when working with from-left accounting:
    //   from_left(center_m) = center_m + target_len_m/2
    //
    // Components are clamped so they stay entirely within the wall.
    std::vector<int> seg_targets_mm;
    seg_targets_mm.reserve(n_comps + 1);
    {
        float cursor_m = 0.0f; // from-left cursor
        for (auto& comp : comps) {
            float left_m = comp.preferred_center_m - comp.width_m * 0.5f
                           + target_len_m * 0.5f;
            // Clamp: component must start at or after cursor and end before wall edge.
            left_m = std::clamp(left_m, cursor_m,
                                target_len_m - comp.width_m);
            seg_targets_mm.push_back(
                static_cast<int>(std::round((left_m - cursor_m) * 1000.0f)));
            cursor_m = left_m + comp.width_m;
        }
        seg_targets_mm.push_back(
            static_cast<int>(std::round((target_len_m - cursor_m) * 1000.0f)));
    }

    // ── Solve each fence segment ──────────────────────────────────────────────
    const int n_segs = static_cast<int>(seg_targets_mm.size()); // == n_comps + 1
    std::vector<WallLayout> seg_layouts(n_segs);
    for (int i = 0; i < n_segs; ++i) {
        const int t = seg_targets_mm[i];
        if (t > 0) {
            seg_layouts[i] = catalog_.solve(t, kWallHeightMm);
        } else {
            // Empty segment: no panels (component at/near wall edge).
            seg_layouts[i].height_mm       = kWallHeightMm;
            seg_layouts[i].actual_width_mm = 0;
        }
    }

    // Store representative layout for any external callers that want height.
    wall.layout = seg_layouts[0];
    for (const auto& sl : seg_layouts)
        if (!sl.panels.empty()) { wall.layout = sl; break; }

    // ── Visual geometry constants ─────────────────────────────────────────────
    const float h      = kWallHeightMm * 0.001f;
    const float y_ctr  = h * 0.5f;
    const float post_h = h + 0.05f;
    const float post_cy = post_h * 0.5f;
    const bool  sel    = selected_wall_.has_value() && *selected_wall_ == wall.id;

    // Segment visual length: panel widths + inter-panel gaps (no trailing gap).
    auto segVisualLen = [](const WallLayout& lay) -> float {
        if (lay.panels.empty()) return 0.0f;
        return lay.actual_width_mm * 0.001f
               + static_cast<float>(std::max(0, lay.panel_count() - 1)) * kPanelGap;
    };

    // ── Total visual wall length and component actual positions ───────────────
    // Each component slot contributes: kPanelGap (right boundary) + width_m
    //                                + kPanelGap (left boundary of next segment).
    float total_visual = 0.0f;
    for (int i = 0; i < n_segs; ++i)  total_visual += segVisualLen(seg_layouts[i]);
    for (const auto& c : comps)       total_visual += c.width_m + 2.0f * kPanelGap;

    const float visual_half = total_visual * 0.5f;
    wall.visual_half_m = visual_half;

    // Compute actual centres (used by ConveyorSystem to align belt meshes).
    {
        float c = -visual_half;
        for (int i = 0; i < n_comps; ++i) {
            c += segVisualLen(seg_layouts[i]) + kPanelGap;
            comps[i].actual_center_m = c + comps[i].width_m * 0.5f;
            c += comps[i].width_m + kPanelGap;
        }
    }
    // Write snapped positions back to stored list (match by id).
    for (const auto& sorted_comp : comps) {
        for (auto& stored : components_[wall.id]) {
            if (stored.id == sorted_comp.id) {
                stored.actual_center_m = sorted_comp.actual_center_m;
                break;
            }
        }
    }

    // ── Mesh helpers ──────────────────────────────────────────────────────────
    const auto postMat = MeshPhongMaterial::create();
    postMat->color     = Color(kColorPost);
    postMat->shininess = 60;

    // Place a post centred at local coordinate cx_local.
    auto placePost = [&](float cx_local) {
        auto geo  = BoxGeometry::create(kPostSize, post_h, kPostSize);
        auto mesh = Mesh::create(geo, postMat);
        auto node = Group::create();
        node->add(mesh);
        node->position.set(
            isXWall ? cx_local : 0.0f,
            post_cy,
            isXWall ? 0.0f    : cx_local);
        wall.group->add(node);
        wall.panel_groups.push_back(node);
    };

    // Build the panels of one fence segment, advancing cursor.
    // mat_offset controls the global panel-alternation index.
    auto buildSegment = [&](const WallLayout& lay, float& cursor, int mat_offset) {
        const int n = lay.panel_count();
        for (int j = 0; j < n; ++j) {
            const float pw  = lay.panels[j].width_mm * 0.001f;
            const float cx  = cursor + pw * 0.5f;
            const int   idx = mat_offset + j;

            auto mat = MeshPhongMaterial::create();
            mat->color     = Color(sel ? kColorSelected
                                       : (idx % 2 == 0 ? kColorPanelA : kColorPanelB));
            mat->shininess = 40;
            wall.materials.push_back(mat);

            // Panel mesh — OBJ clone or BoxGeometry fallback.
            std::shared_ptr<Object3D> panel_group;
            const auto it = obj_cache_.find(lay.panels[j].mesh_file);
            if (it != obj_cache_.end()) {
                panel_group = it->second->clone();
                panel_group->scale.set(0.001f, 0.001f, 0.001f);
            }
            if (!panel_group) {
                auto geo = isXWall
                    ? BoxGeometry::create(pw, h, kPanelThickness)
                    : BoxGeometry::create(kPanelThickness, h, pw);
                auto mesh = Mesh::create(geo, mat);
                auto grp  = Group::create();
                grp->add(mesh);
                panel_group = grp;
            }
            panel_group->traverseType<Mesh>([&mat](Mesh& m) {
                const size_t slots = std::max<size_t>(1, m.numMaterials());
                m.setMaterials(
                    std::vector<std::shared_ptr<Material>>(slots, mat));
            });
            if (isXWall) {
                panel_group->position.set(cx, y_ctr, 0.0f);
            } else {
                panel_group->rotation.y = math::PI * 0.5f;
                panel_group->position.set(0.0f, y_ctr, cx);
            }
            wall.group->add(panel_group);
            wall.panel_groups.push_back(panel_group);
            panel_group->traverseType<Mesh>([&](Mesh& m) {
                panel_to_wall_[&m] = wall.id;
            });

            // Invisible hit box for raycasting through mesh gaps.
            {
                auto hitGeo = isXWall
                    ? BoxGeometry::create(pw, h, 0.10f)
                    : BoxGeometry::create(0.10f, h, pw);
                auto hitMat        = MeshBasicMaterial::create();
                hitMat->transparent = true;
                hitMat->opacity     = 0.0f;
                hitMat->depthWrite  = false;
                auto hitMesh = Mesh::create(hitGeo, hitMat);
                hitMesh->position.set(
                    isXWall ? cx   : 0.0f,
                    y_ctr,
                    isXWall ? 0.0f : cx);
                wall.group->add(hitMesh);
                wall.panel_groups.push_back(hitMesh);
                panel_to_wall_[hitMesh.get()] = wall.id;
            }

            cursor += pw;

            // Inter-panel post (not after the last panel in the segment).
            if (j + 1 < n) {
                placePost(cursor + kPanelGap * 0.5f);
                cursor += kPanelGap;
            }
            // Cursor is now at the right edge of the last panel placed.
        }
    };

    // ── Assemble the wall ─────────────────────────────────────────────────────
    float cursor   = -visual_half;
    int   mat_idx  = 0; // global panel alternation counter

    // Left corner post (X-walls own all four corners).
    if (isXWall) placePost(-visual_half);

    for (int i = 0; i < n_segs; ++i) {
        buildSegment(seg_layouts[i], cursor, mat_idx);
        mat_idx += seg_layouts[i].panel_count();

        if (i < n_comps) {
            // ── Boundary post on the right side of this segment ───────────────
            // Cursor is at the right edge of the last panel (or at the segment
            // start if the segment is empty — in that case the post coincides
            // with the previous element's right edge, which is fine).
            placePost(cursor + kPanelGap * 0.5f);
            cursor += kPanelGap;

            // ── Component opening: no fence panels ───────────────────────────
            cursor += comps[i].width_m;

            // ── Boundary post on the left side of the next segment ────────────
            placePost(cursor + kPanelGap * 0.5f);
            cursor += kPanelGap;
        }
    }

    // Right corner post (X-walls only).
    if (isXWall) placePost(+visual_half);
}

// ---------------------------------------------------------------------------
// Gizmo
// ---------------------------------------------------------------------------

Vector3 WallSystem::gizmoWorldPos(WallId id) const {
    const float hw = walls_[NORTH].visual_half_m;
    const float hd = walls_[EAST].visual_half_m;
    const float h  = kWallHeightMm * 0.0005f; // half-height in metres
    switch (id) {
        case NORTH: return {  0, h, -(hd + kGizmoOffset) };
        case SOUTH: return {  0, h,   hd + kGizmoOffset  };
        case EAST:  return {  hw + kGizmoOffset, h, 0 };
        case WEST:  return { -(hw + kGizmoOffset), h, 0 };
    }
    return {};
}

Vector3 WallSystem::wallOutwardNormal(WallId id) const {
    switch (id) {
        case NORTH: return {  0, 0, -1 };
        case SOUTH: return {  0, 0,  1 };
        case EAST:  return {  1, 0,  0 };
        case WEST:  return { -1, 0,  0 };
    }
    return {};
}

void WallSystem::showGizmo(WallId id) {
    hideGizmo();
    gizmo_root_ = makeArrow(gizmo_handle_);
    repositionGizmo(id);
    scene_.add(gizmo_root_);
}

void WallSystem::repositionGizmo(WallId id) {
    if (!gizmo_root_) return;
    Vector3 pos = gizmoWorldPos(id);
    gizmo_root_->position.set(pos.x, pos.y, pos.z);

    gizmo_root_->rotation.set(0, 0, 0);
    switch (id) {
        case EAST:  gizmo_root_->rotation.z = -math::PI * 0.5f; break;
        case WEST:  gizmo_root_->rotation.z =  math::PI * 0.5f; break;
        case SOUTH: gizmo_root_->rotation.x =  math::PI * 0.5f; break;
        case NORTH: gizmo_root_->rotation.x = -math::PI * 0.5f; break;
    }
}

void WallSystem::hideGizmo() {
    if (gizmo_root_) {
        scene_.remove(*gizmo_root_);
        gizmo_root_.reset();
        gizmo_handle_.reset();
    }
}

// ---------------------------------------------------------------------------
// Selection highlight
// ---------------------------------------------------------------------------

void WallSystem::highlightWall(WallId id, bool on) {
    auto& wall = walls_[id];
    for (size_t i = 0; i < wall.materials.size(); ++i) {
        wall.materials[i]->color = Color(
            on ? kColorSelected
               : (i % 2 == 0 ? kColorPanelA : kColorPanelB));
    }
}

// ---------------------------------------------------------------------------
// Mouse events
// ---------------------------------------------------------------------------

Vector2 WallSystem::toNDC(const Vector2& screen) const {
    auto sz = canvas_.size();
    return {
         (screen.x / static_cast<float>(sz.width()))  * 2.0f - 1.0f,
        -(screen.y / static_cast<float>(sz.height())) * 2.0f + 1.0f
    };
}

void WallSystem::onMouseDown(int button, const Vector2& pos) {
    if (button != 0) return;

    raycaster_.setFromCamera(toNDC(pos), camera_);

    if (state_ == State::SELECTED && gizmo_handle_) {
        auto hits = raycaster_.intersectObject(*gizmo_handle_);
        if (!hits.empty()) {
            drag_wall_      = *selected_wall_;
            drag_start_dim_ = (drag_wall_ == EAST || drag_wall_ == WEST)
                                  ? width_mm_ : depth_mm_;

            Vector3 gp = gizmoWorldPos(drag_wall_);
            drag_plane_ = Plane(Vector3(0, 1, 0), -gp.y);
            raycaster_.ray.intersectPlane(drag_plane_, drag_start_hit_);

            state_ = State::DRAGGING;
            controls_.enabled = false;
            return;
        }
    }

    std::vector<Object3D*> panel_ptrs;
    panel_ptrs.reserve(panel_to_wall_.size());
    for (auto& [ptr, _] : panel_to_wall_) panel_ptrs.push_back(ptr);

    auto hits = raycaster_.intersectObjects(panel_ptrs);
    if (!hits.empty()) {
        Object3D* hit_obj = hits[0].object;
        auto it = panel_to_wall_.find(hit_obj);
        if (it != panel_to_wall_.end()) {
            WallId hit_wall = it->second;
            if (selected_wall_ != hit_wall) {
                if (selected_wall_) highlightWall(*selected_wall_, false);
                selected_wall_ = hit_wall;
                highlightWall(hit_wall, true);
                showGizmo(hit_wall);
            }
            state_ = State::SELECTED;
        }
        return;
    }

    if (selected_wall_) {
        highlightWall(*selected_wall_, false);
        selected_wall_.reset();
        hideGizmo();
    }
    state_ = State::IDLE;
}

void WallSystem::onMouseMove(const Vector2& pos) {
    if (state_ != State::DRAGGING) return;

    raycaster_.setFromCamera(toNDC(pos), camera_);

    Vector3 hit;
    raycaster_.ray.intersectPlane(drag_plane_, hit);

    const bool isWidth = (drag_wall_ == EAST || drag_wall_ == WEST);
    float delta_m = isWidth ? (hit.x - drag_start_hit_.x)
                             : (hit.z - drag_start_hit_.z);
    if (drag_wall_ == WEST || drag_wall_ == NORTH) delta_m = -delta_m;

    int new_dim = drag_start_dim_ + static_cast<int>(delta_m * 2000.0f);
    new_dim = std::max(new_dim, kMinDimMm);

    if (isWidth)  width_mm_ = new_dim;
    else          depth_mm_ = new_dim;

    buildAllWalls();
    repositionGizmo(drag_wall_);
}

void WallSystem::onMouseUp(int button, const Vector2& pos) {
    if (button != 0 || state_ != State::DRAGGING) return;
    (void)pos;

    state_ = State::SELECTED;
    controls_.enabled = true;
    repositionGizmo(drag_wall_);
}
