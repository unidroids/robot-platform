#pragma once

#include <cstddef>
#include <cstdint>

// Konfigurace pro službu LIDAR-TRAVERSABILITY (port 9223)
// Generuje 80×80 potenciálovou mapu sjízdnosti (8×8 m po 10 cm, X ∈ [-2, +6] m, Y ∈ [-4, +4] m)
struct TraversabilityConfig {
    // Geometrie robota a montáž LiDARu
    float lidar_offset_x_cm      = 22.0f;   // LiDAR je 22 cm před středem otáčení robota (CoR)
    float robot_half_width_cm    = 35.0f;   // Poloviční šířka robota (±35 cm) pro zakázanou zónu středu
    float repulse_radius_cm      = 100.0f;  // Dosah odtlačovacího potenciálu od obrubníků/hran (100 cm)
    float min_valid_dist_cm      = 25.0f;   // Minimální platná vzdálenost od CoR
    float max_range_cm           = 800.0f;  // Maximální vzdálenost bodů pro mapu sjízdnosti (8 m)

    // Fyzikální limity sjízdnosti povrchu (vůči gravitaci)
    float max_step_cm            = 4.0f;    // Max povolený schod / obrubník (4 cm)
    float max_roughness_cm       = 4.0f;    // Max povolená drsnost v buňce (4 cm)
    float max_slope_deg          = 25.0f;   // Max povolený sklon svahu vůči gravitaci (25°)

    // Výškové pásmo pro detekci vysokých 3D překážek
    float obstacle_z_min_cm      = 12.0f;   // Výška nad lokálním terénem pro 3D překážku
    float obstacle_z_max_cm      = 130.0f;  // Horní obrys robota
    float ground_z_min_cm        = -120.0f; // Spodní mez pro body terénu (korekce svahu)
    float ground_z_max_cm        = 120.0f;  // Horní mez pro body terénu

    // Rozměry výstupní mřížky v těle robota (80 × 80 buněk po 10 cm = 8 × 8 m)
    float cell_size_cm           = 10.0f;   // Rozlišení buňky 10 × 10 cm
    size_t out_rows_x            = 80;      // X: [-200 cm, +600 cm) -> 80 buněk (CoR je na ix = 20)
    size_t out_cols_y            = 80;      // Y: [-400 cm, +400 cm) -> 80 buněk (CoR je na iy = 40)
    float out_x_min_cm           = -200.0f; // 2 m za středem otáčení robota
    float out_y_min_cm           = -400.0f; // 4 m vlevo (záporné Y = vlevo, kladné Y = vpravo CW)

    // Vnitřní severově stabilizovaná paměťová mřížka (140 × 140 buněk po 10 cm = 14 × 14 m)
    size_t mem_grid_size         = 140;     // Dostatečná rezerva pro diagonální rotaci 8×8 m okna

    // Časování a filtrace
    uint32_t period_ms           = 100;     // Perioda vyhodnocení 100 ms (10 Hz)
    uint32_t max_lag_ms          = 150;     // Max povolená prodleva mezi cykly (150 ms)
    size_t min_points_per_cycle  = 10000;   // Min počet bodů za 100 ms pro platný cyklus
    size_t min_ground_pts_cell   = 2;       // Min počet bodů v buňce pro vyhodnocení země
    size_t min_obstacle_pts_cell = 2;       // Min počet bodů pro vysokou překážku
    float memory_decay_sec       = 15.0f;   // Doba platnosti paměti pro místa mimo aktuální zorné pole (v křižovatce)
};

