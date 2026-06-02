#pragma once

#include <vector>
#include <cstdint>
#include <string>
#include <fstream>
#include <algorithm>
#include <cmath>
#include "point_buffer.hpp"

// Evaluator pro generování černobílého (grayscale) obrázku odrazivosti (intensity).
// Bere prvních 20 000 bodů, ořízne je a promítne je do 2D BEV gridu s rozlišením 1cm = 1px.
class LidarReflectivityEvaluator {
public:
    struct ReflectivityImage {
        int width = 200;  // Y rozsah: -100 cm až 100 cm (2m) -> 200 px
        int height = 300; // X rozsah: 0 cm až 300 cm (3m) -> 300 px
        std::vector<uint8_t> data;

        ReflectivityImage() : data(width * height, 0) {}

        // Uloží obrázek do formátu PGM (Portable Graymap, P5 binary).
        bool savePGM(const std::string &filepath) const {
            std::ofstream ofs(filepath, std::ios::binary);
            if (!ofs) {
                return false;
            }
            ofs << "P5\n" << width << " " << height << "\n255\n";
            ofs.write(reinterpret_cast<const char*>(data.data()), data.size());
            return ofs.good();
        }
    };

    ReflectivityImage evaluate(const LidarPointBuffer &buffer) const {
        ReflectivityImage img;
        const auto &points = buffer.snapshot();
        const std::size_t n = std::min<std::size_t>(20000u, points.size());

        for (std::size_t i = 0; i < n; ++i) {
            const auto &pt = points[i];

            // Oříznutí podle X (0 až 3m = 0 až 300cm) a Y (-1 až 1m = -100 až 100cm)
            if (pt.x < 0.0f || pt.x > 300.0f || pt.y < -100.0f || pt.y > 100.0f || pt.z > 15.0f) {
                continue;
            }

            // Přepočet na souřadnice pixelů (1cm = 1px)
            // X osa určuje vzdálenost před robotem (výška obrázku, řádek)
            // Aby se vzdálenost zvětšovala směrem NAHORU (standardní BEV), otočíme osu X:
            int row = 299 - static_cast<int>(std::floor(pt.x));
            // Y osa určuje boční offset (šířka obrázku, sloupec)
            int col = static_cast<int>(std::floor(pt.y + 100.0f));
            // Z osa určuje korekci jasnosti (2~1  15~0)
            float correction = pt.z < 3.0f ? 1.0f : 1.0f - (pt.z / 15.0f);

            // Bezpečný zápis do obrazových dat
            if (row >= 0 && row < img.height && col >= 0 && col < img.width) {
                std::size_t idx = static_cast<std::size_t>(row * img.width + col);
                uint8_t val = static_cast<uint8_t>(std::clamp(pt.intensity, 0.0f, 255.0f)) * correction;
                if (val > img.data[idx]) {
                    img.data[idx] = val;
                }
            }
        }

        return img;
    }
};
