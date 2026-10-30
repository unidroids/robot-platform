#pragma once

#include <cstddef>
#include <cstdint>

// Konfigurace geometrie, pater, výsečí a filtrace pro službu LIDAR-COLLISIONS
// (port 9222)
struct CollisionConfig {
  // Geometrie a montáž
  float lidar_offset_x_cm = 22.0f; // LiDAR je 22 cm před středem otáčení robota
                                   // (Center of Rotation - CoR)
  float robot_diameter_cm = 75.0f; // Válec těla robota (max šířka 60 cm + 7.5
                                   // cm rezerva na každé straně)
  float robot_radius_cm =
      37.5f; // Poloměr pro inflaci a CBDR kolizní kurz při výpočtu TTC
  float min_valid_dist_cm = 25.0f; // Ignorovat body blíže než 25 cm od CoR
  float max_range_cm = 2000.0f;    // Maximální sledovaná vzdálenost (20 m)

  // Pevný geometrický obrys šasi robota v kalibrovaném rámci LiDARu [m]
  float mask_front_x_m =
      0.23f; // Přední hrana (+18 cm čumák od LiDARu + 5 cm rezerva)
  float mask_x_min_m = -0.70f; // Zadní hrana (-70 cm za LiDARem)
  float mask_y_min_m = -0.35f; // Pravá boční hrana (-30 cm + 5 cm rezerva)
  float mask_y_max_m = 0.35f;  // Levá boční hrana (+30 cm + 5 cm rezerva)

  // Vertikální patra (spodní hranice 30 cm nad zemí, detekce pouze těla překážek)
  float z_min_cm = 30.0f;        // Odfiltrování země, nerovností a trávy pod 30 cm
  float z_max_cm = 130.0f;       // Horní limit obrysu robota
  float floor_height_cm = 10.0f; // Výška jednoho patra (10 cm)
  size_t num_floors = 10;        // Celkem 10 pater: [30-40), [40-50), ..., [120-130) cm

  // Horizontální výseče (72 sektorů po 5°)
  size_t num_sectors = 72;       // 72 sektorů po 5° (360° / 72)
  float sector_angle_deg = 5.0f; // Úhlová šířka sektoru

  // Časování a dynamická filtrace
  uint32_t period_ms = 100; // Perioda vyhodnocení 100 ms (10 Hz)
  float dt_sec = 0.1f;      // Časový krok v sekundách
  float alpha = 0.40f;      // Koeficient polohy alfa-beta filtru
  float beta = 0.15f;       // Koeficient rychlosti alfa-beta filtru
  float alpha_theta =
      0.50f; // Koeficient azimutu pro sledování úhlového posuvu (CBDR)
  float beta_theta = 0.20f; // Koeficient úhlové rychlosti pro CBDR
  float max_track_jump_cm =
      40.0f; // Max změna vzdálenosti za 100 ms pro zachování tracku
  uint32_t min_hits_for_ttc =
      3; // Min počet snímků v kolizním kurzu pro vyhlášení TTC
  size_t min_points_per_cell = 2; // Odfiltrování ojedinělých odrazů / šumu
  float min_ttc_speed_cms = 15.0f;     // Minimální rychlost přibližování pro TTC (15 cm/s, ignoruje statický šum)
  size_t min_points_per_cycle =
      500;                   // Minimální počet bodů za 100 ms pro platný cyklus (venku ~800-3500 bodů/100 ms)
  uint32_t max_lag_ms = 500; // Max povolená prodleva mezi cykly (odolnost vůči diskovému I/O)
};
