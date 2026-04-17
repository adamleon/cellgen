#include "visualization/cell_renderer.hpp"

using namespace threepp;

CellRenderer::CellRenderer(Scene& scene) : scene_(scene) {
    addFloorAndGrid();
    addLights();
}

void CellRenderer::addFloorAndGrid() {
    // Concrete floor plane — larger than any reasonable cell.
    auto floorGeo = PlaneGeometry::create(20.0f, 20.0f);
    auto floorMat = MeshPhongMaterial::create();
    floorMat->color = Color(0xd0cfc8);
    floorMat->shininess = 10;
    auto floor = Mesh::create(floorGeo, floorMat);
    floor->rotation.x = -math::PI / 2.0f;
    floor->position.y = -0.001f;
    scene_.add(floor);

    // 0.5 m grid overlay.
    auto grid = GridHelper::create(20, 40, Color(0x888888), Color(0x555555));
    scene_.add(grid);
}

void CellRenderer::addLights() {
    auto hemi = HemisphereLight::create(0xddeeff, 0x443322, 0.6f);
    scene_.add(hemi);

    auto key = DirectionalLight::create(0xffffff, 1.0f);
    key->position.set(6, 10, 6);
    scene_.add(key);

    auto fill = DirectionalLight::create(0xffffff, 0.3f);
    fill->position.set(-4, 5, -4);
    scene_.add(fill);
}
