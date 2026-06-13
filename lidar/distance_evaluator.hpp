#pragma once

#include <cstddef>
#include <cmath>

#include "point_buffer.hpp"

// Vyhodnocení minimální vzdálenosti překážky z ring bufferu bodů.
class LidarDistanceEvaluator {
public:
    // Vrací sqrt(x^2 + y^2) [cm] v z-intervalu a y-intervalu; 5000 cm pokud nic nenalezeno; -1 pokud buffer nenaplněn.
    float distance(const LidarPointBuffer &buffer,
                   float z_min =  50.0f,
                   float z_max =  150.0f,
                   float y_min = -80.0f,
                   float y_max =  80.0f,
                   float min_intensity = 10.0f) const
    {
        if (buffer.size() < LidarPointBuffer::kCapacity) {
            return -1.0f;
        }

        float min_sq = 2000.0f * 2000.0f; // 20m * 20m
        bool found = false;

        for (const auto &p : buffer.snapshot()) {
            if (p.z < z_min || p.z > z_max || p.y < y_min || p.y > y_max || p.intensity < min_intensity) {
                continue;
            }
            const float d2 = p.x * p.x + p.y * p.y;
            if (d2 < min_sq) {
                min_sq = d2;
                found = true;
            }
        }

        if (!found) {
            return 2000.0f;
        }
        return std::sqrt(min_sq);
    }
};
