#include "core/fence_catalog.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>

#include <nlohmann/json.hpp>

namespace cellgen {

FenceCatalog::FenceCatalog() {
    // clang-format off
    panels_ = {
        {  600, 1400, 120.0f, {} },
        {  600, 2000, 155.0f, {} },
        {  800, 1400, 145.0f, {} },
        {  800, 2000, 190.0f, {} },
        { 1000, 1400, 165.0f, {} },
        { 1000, 2000, 210.0f, {} },
        { 1200, 1400, 185.0f, {} },
        { 1200, 2000, 240.0f, {} },
    };
    // clang-format on
}

FenceCatalog::FenceCatalog(const std::filesystem::path& catalog_json) {
    std::ifstream f(catalog_json);
    if (!f.is_open())
        throw std::runtime_error("FenceCatalog: cannot open " + catalog_json.string());

    using json = nlohmann::json;
    const auto j = json::parse(f);

    for (const auto& p : j.at("panels")) {
        panels_.push_back({
            p.at("width_mm").get<int>(),
            p.at("height_mm").get<int>(),
            p.at("cost_eur").get<float>(),
            p.at("filename").get<std::string>(),
        });
    }
}

int FenceCatalog::snapHeight(int target_mm) const {
    int best      = panels_.front().height_mm;
    int best_diff = std::abs(target_mm - best);
    for (const auto& p : panels_) {
        int d = std::abs(target_mm - p.height_mm);
        if (d < best_diff) { best_diff = d; best = p.height_mm; }
    }
    return best;
}

WallLayout FenceCatalog::solve(int target_width_mm, int target_height_mm) const {
    const int height_mm = snapHeight(target_height_mm);

    // Collect panels matching this height.
    std::vector<FencePanel> avail;
    for (const auto& p : panels_)
        if (p.height_mm == height_mm) avail.push_back(p);

    // Minimum achievable width is the narrowest single panel.
    int min_panel = avail[0].width_mm;
    for (const auto& p : avail) min_panel = std::min(min_panel, p.width_mm);

    // DP up to target + 2 400 mm search headroom.
    const int search_max = target_width_mm + 2400;
    const float INF = std::numeric_limits<float>::infinity();

    std::vector<float> dp(search_max + 1, INF);
    std::vector<int>  from(search_max + 1, -1);  // panel width used to reach w
    dp[0] = 0.0f;

    for (int w = 1; w <= search_max; ++w) {
        for (const auto& p : avail) {
            int prev = w - p.width_mm;
            if (prev >= 0 && dp[prev] + p.cost < dp[w]) {
                dp[w] = dp[prev] + p.cost;
                from[w] = p.width_mm;
            }
        }
    }

    // Find achievable width closest to target (prefer narrower on tie).
    int best_w   = -1;
    int best_diff = INT_MAX;
    int search_min = min_panel;  // at least one panel
    for (int w = search_min; w <= search_max; ++w) {
        if (dp[w] < INF) {
            int diff = std::abs(w - target_width_mm);
            if (diff < best_diff || (diff == best_diff && w < best_w)) {
                best_diff = diff;
                best_w    = w;
            }
        }
    }

    if (best_w < 0) best_w = min_panel; // unreachable fallback

    // Reconstruct panel sequence.
    std::vector<FencePanel> seq;
    for (int rem = best_w; rem > 0; ) {
        int pw = from[rem];
        for (const auto& p : avail) {
            if (p.width_mm == pw) { seq.push_back(p); break; }
        }
        rem -= pw;
    }
    std::reverse(seq.begin(), seq.end());

    WallLayout layout;
    layout.actual_width_mm = best_w;
    layout.height_mm       = height_mm;
    layout.panels          = seq;
    layout.total_cost      = dp[best_w];
    return layout;
}

} // namespace cellgen
