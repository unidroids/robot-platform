#pragma once

#include <cstddef>
#include <cmath>
#include <vector>
#include <algorithm>

#include "point_buffer.hpp"

// Vyhodnocení minimální vzdálenosti překážky z ring bufferu bodů.
class LidarDistanceEvaluator {
public:
    // Vrací sqrt(x^2 + y^2) [cm] v z-intervalu a y-intervalu; 2000 cm pokud nic nenalezeno; -1 pokud buffer nenaplněn.
    float distance(const LidarPointBuffer &buffer,
                   float z_min =  50.0f,
                   float z_max =  150.0f,
                   float y_min = -80.0f,
                   float y_max =  80.0f,
                   float min_intensity = 30.0f,
                   size_t min_points = 25) const
    {
        if (buffer.size() < LidarPointBuffer::kCapacity) {
            return -1.0f;
        }

        std::vector<float> valid_d2;
        valid_d2.reserve(buffer.size());

        for (const auto &p : buffer.snapshot()) {
            if (p.z < z_min || p.z > z_max || p.y < y_min || p.y > y_max || p.intensity < min_intensity) {
                continue;
            }
            valid_d2.push_back(p.x * p.x + p.y * p.y);
        }

        if (min_points == 0) {
            min_points = 1;
        }

        if (valid_d2.size() < min_points) {
            return 2000.0f;
        }

        // Aplikace rank filtru pro odstranění much, smetí, kapek
        std::nth_element(valid_d2.begin(), valid_d2.begin() + min_points - 1, valid_d2.end());
        
        return std::sqrt(valid_d2[min_points - 1]);
    }
};
