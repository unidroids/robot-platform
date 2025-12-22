#pragma once

#include <cstddef>

#include "point_buffer.hpp"

// Placeholder pro budoucí vyhodnocení koridoru (obsazenost / sjízdnost).
class CorridorEvaluator {
public:
    struct Result {
        bool valid = false;
        // Přidej další metriky (např. clearance, left/right margin, drivable flag).
    };

    Result evaluate(const LidarPointBuffer &buffer) const {
        Result r{};
        r.valid = buffer.size() > 0;
        return r;
    }
};
