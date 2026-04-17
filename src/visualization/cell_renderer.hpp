#pragma once

#include <threepp/threepp.hpp>

// Builds the static scene environment: floor, grid, and lights.
class CellRenderer {
public:
    explicit CellRenderer(threepp::Scene& scene);

private:
    void addFloorAndGrid();
    void addLights();

    threepp::Scene& scene_;
};
