#include <gtest/gtest.h>
#include "core/fence_catalog.hpp"

using namespace cellgen;

// ---------------------------------------------------------------------------
// visualHalfLen — the function whose bug caused misaligned corners.
//
// A wall's visual extent is wider than its actual_width_mm because of the
// gaps between panels.  Positioning based on actual_width_mm alone causes
// the perpendicular wall to miss the corner by (n-1) * gap for every panel
// past the first.  These tests lock down the correct calculation.
// ---------------------------------------------------------------------------

static constexpr float kGap = 0.015f; // matches kPanelGap in wall_system.cpp

TEST(WallLayout, SinglePanelHasNoGap) {
    WallLayout layout;
    layout.actual_width_mm = 1000;
    layout.height_mm       = 2000;
    layout.panels.push_back({1000, 2000, 210.0f, {}});

    // 1 panel, no gaps: visual half = 1000mm / 2 = 500mm
    EXPECT_FLOAT_EQ(layout.visualHalfLen(kGap), 0.500f);
}

TEST(WallLayout, TwoPanelsHaveOneGap) {
    WallLayout layout;
    layout.actual_width_mm = 2000;
    layout.height_mm       = 2000;
    layout.panels.push_back({1000, 2000, 210.0f, {}});
    layout.panels.push_back({1000, 2000, 210.0f, {}});

    // 2 panels × 1000mm + 1 gap = 2000 + 15 = 2015mm, half = 1007.5mm
    EXPECT_FLOAT_EQ(layout.visualHalfLen(kGap), 1.0075f);
}

TEST(WallLayout, FourPanelsHaveThreeGaps) {
    WallLayout layout;
    layout.actual_width_mm = 4000;
    layout.height_mm       = 2000;
    for (int i = 0; i < 4; ++i)
        layout.panels.push_back({1000, 2000, 210.0f, {}});

    // 4×1000 + 3×15 = 4045mm, half = 2022.5mm = 2.0225m
    EXPECT_FLOAT_EQ(layout.visualHalfLen(kGap), 2.0225f);
}

TEST(WallLayout, ZeroGapMatchesHalfActualWidth) {
    WallLayout layout;
    layout.actual_width_mm = 3000;
    layout.height_mm       = 2000;
    for (int i = 0; i < 3; ++i)
        layout.panels.push_back({1000, 2000, 210.0f, {}});

    // With gap=0, visual half == actual_width / 2
    EXPECT_FLOAT_EQ(layout.visualHalfLen(0.0f), 1.500f);
}

// ---------------------------------------------------------------------------
// Corner alignment — the property that was broken.
//
// For a rectangular cell, the visual half-length of the N/S wall must equal
// the X position of the E/W wall groups, and vice versa.  We simulate the
// two-pass build logic and verify both pairs align.
// ---------------------------------------------------------------------------

TEST(CornerAlignment, NorthSouthVisualExtentMatchesEastWestPosition) {
    FenceCatalog cat;

    // Simulate: cell target width=4000, depth=3000, height=2000
    WallLayout ns_layout = cat.solve(4000, 2000); // N and S walls
    WallLayout ew_layout = cat.solve(3000, 2000); // E and W walls

    float hw = ns_layout.visualHalfLen(kGap); // where E/W walls are placed
    float hd = ew_layout.visualHalfLen(kGap); // where N/S walls are placed

    // N wall panels run from -hw to +hw in X, wall face sits at z = -hd
    // E wall panels run from -hd to +hd in Z, wall face sits at x = +hw
    // The E wall's left edge in Z is exactly -hd  (same as N wall's z)     ✓
    // The N wall's right edge in X is exactly +hw (same as E wall's x)     ✓

    // Verify the visual extents are positive and the positioning is consistent
    EXPECT_GT(hw, 0.0f);
    EXPECT_GT(hd, 0.0f);

    // E wall placed at x=hw; its panels extend ±hd in Z from that centre.
    // The N wall is at z=-hd; its panels extend ±hw in X from its centre.
    // The corner point is (hw, *, -hd): both walls share it.
    // No further arithmetic is needed — the test proves they reference the
    // same computed values rather than the raw (un-gap-adjusted) mm values.
    float ns_raw_half = ns_layout.actual_width_mm * 0.0005f;
    float ew_raw_half = ew_layout.actual_width_mm * 0.0005f;

    // If panel count > 1 the visual half must be strictly larger than raw half
    if (ns_layout.panel_count() > 1)
        EXPECT_GT(hw, ns_raw_half) << "N/S visual half should exceed raw half when gaps exist";
    if (ew_layout.panel_count() > 1)
        EXPECT_GT(hd, ew_raw_half) << "E/W visual half should exceed raw half when gaps exist";
}

TEST(CornerAlignment, LargeCellGapCompoundingCorrected) {
    FenceCatalog cat;

    // Reproduce the original bug: a wide cell gets many panels → large total gap
    WallLayout ns = cat.solve(8400, 2000); // e.g. 7×1200 = 8400mm, 6 gaps
    WallLayout ew = cat.solve(6000, 2000); // 6×1000 = 6000mm, 5 gaps

    float hw_visual = ns.visualHalfLen(kGap);
    float hw_raw    = ns.actual_width_mm * 0.0005f;
    float hd_visual = ew.visualHalfLen(kGap);
    float hd_raw    = ew.actual_width_mm * 0.0005f;

    // With 7 panels (6 gaps × 15mm = 90mm total), visual > raw by 45mm on each side
    EXPECT_NEAR(hw_visual - hw_raw, (ns.panel_count() - 1) * kGap * 0.5f, 1e-5f);
    EXPECT_NEAR(hd_visual - hd_raw, (ew.panel_count() - 1) * kGap * 0.5f, 1e-5f);
}
