#pragma once

#include <cstddef>
#include <cstdint>

// Konfigurace geometrie, pater, výsečí a filtrace pro službu LIDAR-KOLIZE (port 9222)
struct CollisionConfig {
    // Geometrie a montáž
    float lidar_offset_x_cm      = 22.0f;   // LiDAR je 22 cm před středem otáčení robota (Center of Rotation - CoR)
    float robot_diameter_cm      = 75.0f;   // Válec těla robota (max šířka 60 cm + 7.5 cm rezerva na každé straně)
    float robot_radius_cm        = 37.5f;   // Poloměr pro inflaci při výpočtu TTC
    float min_valid_dist_cm      = 25.0f;   // Ignorovat body blíže než 25 cm (tělo robota)
    float max_range_cm           = 2000.0f; // Maximální sledovaná vzdálenost (20 m)

    // Vertikální patra
    float z_min_cm               = 10.0f;   // Odfiltrování země/nerovností pod 10 cm
    float z_max_cm               = 130.0f;  // Horní limit obrysu robota
    float floor_height_cm        = 10.0f;   // Výška jednoho patra (10 cm)
    size_t num_floors            = 12;      // Celkem 12 pater: [10-20), [20-30), ..., [120-130)

    // Horizontální výseče
    size_t num_sectors           = 36;      // 36 sektorů po 10° (360° / 36)
    float sector_angle_deg       = 10.0f;   // Úhlová šířka sektoru

    // Časování a dynamická filtrace
    uint32_t period_ms           = 100;     // Perioda vyhodnocení 100 ms (10 Hz)
    float dt_sec                 = 0.1f;    // Časový krok v sekundách
    float alpha                  = 0.40f;   // Koeficient polohy alfa-beta filtru
    float beta                   = 0.15f;   // Koeficient rychlosti alfa-beta filtru
    float max_track_jump_cm      = 40.0f;   // Max změna vzdálenosti za 100 ms pro zachování tracku
    size_t min_points_per_cell   = 2;       // Odfiltrování ojedinělých odrazů / šumu
    size_t min_points_per_cycle  = 10000;   // Minimální počet bodů za 100 ms pro platný cyklus
    uint32_t max_lag_ms          = 150;     // Max povolená prodleva mezi cykly
};
