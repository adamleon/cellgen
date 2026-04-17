#include <gtest/gtest.h>
#include "core/fence_catalog.hpp"

using namespace cellgen;

// ---------------------------------------------------------------------------
// Height snapping
// ---------------------------------------------------------------------------

TEST(FenceCatalog, SnapToExactHeight) {
    FenceCatalog cat;
    EXPECT_EQ(cat.solve(1000, 1400).height_mm, 1400);
    EXPECT_EQ(cat.solve(1000, 2000).height_mm, 2000);
}

TEST(FenceCatalog, SnapHeightBelowMidpoint) {
    FenceCatalog cat;
    // 1699 is 299 from 1400, 301 from 2000 → snaps to 1400
    EXPECT_EQ(cat.solve(1000, 1699).height_mm, 1400);
}

TEST(FenceCatalog, SnapHeightAboveMidpoint) {
    FenceCatalog cat;
    // 1701 is 301 from 1400, 299 from 2000 → snaps to 2000
    EXPECT_EQ(cat.solve(1000, 1701).height_mm, 2000);
}

TEST(FenceCatalog, SnapHeightVeryLow) {
    FenceCatalog cat;
    EXPECT_EQ(cat.solve(1000, 0).height_mm, 1400);
}

TEST(FenceCatalog, SnapHeightVeryHigh) {
    FenceCatalog cat;
    EXPECT_EQ(cat.solve(1000, 9999).height_mm, 2000);
}

// ---------------------------------------------------------------------------
// Width snapping — closest achievable wins
// ---------------------------------------------------------------------------

TEST(FenceCatalog, ExactWidthSinglePanel) {
    FenceCatalog cat;
    for (int w : {600, 800, 1000, 1200}) {
        auto layout = cat.solve(w, 2000);
        EXPECT_EQ(layout.actual_width_mm, w)
            << "Expected exact match for " << w << " mm";
    }
}

TEST(FenceCatalog, SnapsToNarrowerOnExactTie) {
    FenceCatalog cat;
    // 700 mm: 600 is 100 away, 800 is 100 away — prefer narrower (600)
    EXPECT_EQ(cat.solve(700, 2000).actual_width_mm, 600);
}

TEST(FenceCatalog, SnapsBelowTarget) {
    FenceCatalog cat;
    // 650 mm: 600 is 50 away, 800 is 150 away → snap to 600
    EXPECT_EQ(cat.solve(650, 2000).actual_width_mm, 600);
}

TEST(FenceCatalog, SnapsAboveTarget) {
    FenceCatalog cat;
    // 850 mm: 800 is 50 away, 1000 is 150 away → snap to 800
    EXPECT_EQ(cat.solve(850, 2000).actual_width_mm, 800);
}

TEST(FenceCatalog, LargeWidthSnaps) {
    FenceCatalog cat;
    // 4000 mm is exactly achievable (4 × 1000 or 2 × 1200 + 2 × 800, etc.)
    EXPECT_EQ(cat.solve(4000, 2000).actual_width_mm, 4000);
}

// ---------------------------------------------------------------------------
// Cost optimisation — cheapest combination for a given snapped width
// ---------------------------------------------------------------------------

// Verified costs (h = 2000 mm):  600→155  800→190  1000→210  1200→240

TEST(FenceCatalog, SinglePanelCost) {
    FenceCatalog cat;
    EXPECT_FLOAT_EQ(cat.solve(600,  2000).total_cost, 155.0f);
    EXPECT_FLOAT_EQ(cat.solve(800,  2000).total_cost, 190.0f);
    EXPECT_FLOAT_EQ(cat.solve(1000, 2000).total_cost, 210.0f);
    EXPECT_FLOAT_EQ(cat.solve(1200, 2000).total_cost, 240.0f);
}

TEST(FenceCatalog, PrefersSingleWidePanel) {
    FenceCatalog cat;
    // 1200 mm:  1×1200 = 240  vs  2×600 = 310 → single panel wins
    auto layout = cat.solve(1200, 2000);
    EXPECT_EQ(layout.actual_width_mm, 1200);
    EXPECT_EQ(layout.panel_count(), 1);
    EXPECT_FLOAT_EQ(layout.total_cost, 240.0f);
}

TEST(FenceCatalog, CheapestFor3000mm) {
    FenceCatalog cat;
    // 3×1000 = 630
    // 1200+1000+800 = 640
    // 1200+1200+600 = 635
    // 3×1000 is cheapest
    auto layout = cat.solve(3000, 2000);
    EXPECT_EQ(layout.actual_width_mm, 3000);
    EXPECT_FLOAT_EQ(layout.total_cost, 630.0f);
}

TEST(FenceCatalog, CheapestFor4000mm) {
    FenceCatalog cat;
    // 4×1000 = 840
    // 2×1200+2×800 = 860
    // 4×1000 is cheapest
    auto layout = cat.solve(4000, 2000);
    EXPECT_EQ(layout.actual_width_mm, 4000);
    EXPECT_FLOAT_EQ(layout.total_cost, 840.0f);
}

TEST(FenceCatalog, CheapestFor2400mm) {
    FenceCatalog cat;
    // 2×1200 = 480
    // 3×800  = 570
    // 1200+1200 wins
    auto layout = cat.solve(2400, 2000);
    EXPECT_EQ(layout.actual_width_mm, 2400);
    EXPECT_FLOAT_EQ(layout.total_cost, 480.0f);
}

TEST(FenceCatalog, LowHeightCatalogUsed) {
    FenceCatalog cat;
    // At h=1400: 600→120  800→145  1000→165  1200→185
    // 2×1200+1×600 = 490  vs  3×1000 = 495  →  490 wins
    auto layout = cat.solve(3000, 1400);
    EXPECT_EQ(layout.height_mm, 1400);
    EXPECT_FLOAT_EQ(layout.total_cost, 490.0f);
}

// ---------------------------------------------------------------------------
// Panel sequence integrity
// ---------------------------------------------------------------------------

TEST(FenceCatalog, PanelWidthsSumToActualWidth) {
    FenceCatalog cat;
    for (int target : {600, 1200, 2400, 3000, 4000, 5600}) {
        auto layout = cat.solve(target, 2000);
        int sum = 0;
        for (const auto& p : layout.panels) sum += p.width_mm;
        EXPECT_EQ(sum, layout.actual_width_mm)
            << "Panel widths don't sum to actual_width_mm for target=" << target;
    }
}

TEST(FenceCatalog, AllPanelsMatchLayoutHeight) {
    FenceCatalog cat;
    auto layout = cat.solve(3600, 2000);
    for (const auto& p : layout.panels) {
        EXPECT_EQ(p.height_mm, layout.height_mm);
    }
}

TEST(FenceCatalog, NoPanelsForZeroWidth) {
    FenceCatalog cat;
    // Degenerate input: target 0 should return at least one panel (min = 600)
    auto layout = cat.solve(0, 2000);
    EXPECT_GE(layout.actual_width_mm, 600);
    EXPECT_GT(layout.panel_count(), 0);
}
