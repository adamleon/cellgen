#pragma once

#include <filesystem>
#include <string>
#include <vector>
#include <climits>

namespace cellgen {

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

struct FencePanel {
    int         width_mm;
    int         height_mm;
    float       cost;       // EUR
    std::string mesh_file;  // OBJ filename within catalog directory; empty = none
};

struct WallLayout {
    int  actual_width_mm;       // snapped value (may differ from target)
    int  height_mm;
    std::vector<FencePanel> panels;  // left-to-right sequence
    float total_cost;

    int panel_count() const { return static_cast<int>(panels.size()); }

    // Half of the true visual length: panel widths plus the gaps between them.
    // Pass the inter-panel gap in metres (same value used by the renderer).
    // Used for wall group positioning so perpendicular walls meet at corners.
    float visualHalfLen(float panel_gap_m) const {
        int   n         = panel_count();
        float panel_sum = actual_width_mm * 0.001f;
        float gap_sum   = (n > 1 ? n - 1 : 0) * panel_gap_m;
        return (panel_sum + gap_sum) * 0.5f;
    }
};

// ---------------------------------------------------------------------------
// Catalog
// ---------------------------------------------------------------------------
// Standard fence panel sizes (Axelent-inspired).
// Widths : 600, 800, 1000, 1200 mm
// Heights: 1400 mm (low) | 2000 mm (standard robot-cell height)
//
// solve() snaps the target height to the nearest available height, then finds
// the cheapest combination of panel widths whose sum is closest to the target
// width.  When multiple widths are equidistant from the target the narrower
// one is preferred; among solutions of equal width the cheapest is returned.

class FenceCatalog {
public:
    // Default: hardcoded Axelent-inspired panels (for unit tests, no mesh files).
    FenceCatalog();

    // Load panel definitions from a catalog.json file.
    // catalog_json must be the full path to the JSON file.
    explicit FenceCatalog(const std::filesystem::path& catalog_json);

    WallLayout solve(int target_width_mm, int target_height_mm) const;

    const std::vector<FencePanel>& panels() const { return panels_; }

private:
    std::vector<FencePanel> panels_;

    int snapHeight(int target_mm) const;
};

} // namespace cellgen
