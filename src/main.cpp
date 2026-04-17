#include <threepp/threepp.hpp>
#include <threepp/controls/OrbitControls.hpp>
#include "visualization/cell_renderer.hpp"
#include "visualization/wall_system.hpp"
#include "visualization/conveyor_system.hpp"
#include "visualization/robot_system.hpp"

#include <filesystem>

using namespace threepp;
namespace fs = std::filesystem;

#ifndef CELLGEN_ASSETS_DIR
#define CELLGEN_ASSETS_DIR "assets"
#endif

int main() {
    Canvas canvas("CellGen — Palletizing Cell", {{"aa", 4}});

    GLRenderer renderer(canvas.size());

    auto scene = Scene::create();
    scene->background = Color(0x1a1f2e);

    auto camera = PerspectiveCamera::create(50, canvas.aspect(), 0.1f, 100);
    camera->position.set(5.5f, 4.0f, 6.5f);
    camera->lookAt({0, 0.5f, 0});

    OrbitControls controls{*camera, canvas};
    controls.target = {0, 0.5f, 0};
    controls.update();

    canvas.onWindowResize([&](WindowSize size) {
        camera->aspect = size.aspect();
        camera->updateProjectionMatrix();
        renderer.setSize(size);
    });

    const fs::path assets = CELLGEN_ASSETS_DIR;
    const fs::path catalog_dir = assets / "components/fences/axelent_x-guard";

    CellRenderer cellStatic(*scene);
    cellgen::WallSystem walls(*scene, *camera, canvas, controls, catalog_dir, 4000, 3000);

    // ── Conveyors ────────────────────────────────────────────────────────────
    cellgen::ConveyorSystem conveyors(*scene, walls, walls.widthMm(), walls.depthMm());

    // ── Robot ────────────────────────────────────────────────────────────────
    // KR10 R1100-2: reach 1100 mm.  Fits whenever min(width, depth) > 1100 mm.
    cellgen::RobotSystem robot(
        *scene,
        {
            .urdf_path = assets / "robots/kuka_agilus/urdf/kr10_r1100_2.urdf",
            .reach_mm  = 1100,
            .name      = "KUKA KR10 R1100-2",
        });

    // Seed the fit check with the initial cell dimensions.
    robot.onCellResized(walls.widthMm(), walls.depthMm());

    int prev_width = walls.widthMm();
    int prev_depth = walls.depthMm();

    canvas.animate([&] {
        // Update robot placement whenever the cell is resized.
        const int w = walls.widthMm();
        const int d = walls.depthMm();
        if (w != prev_width || d != prev_depth) {
            conveyors.onCellResized(w, d);
            robot.onCellResized(w, d);
            prev_width = w;
            prev_depth = d;
        }

        renderer.render(*scene, *camera);
    });

    return 0;
}
