#pragma once

#include "traversability_config.hpp"
#include "../point_buffer.hpp"

#include <vector>
#include <array>
#include <cmath>
#include <algorithm>
#include <string>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <cstdint>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Jádro pro výpočet 80×80 potenciálové mapy sjízdnosti (8×8 m po 10 cm) se severově stabilizovanou pamětí
class TraversabilityEvaluator {
public:
    static constexpr size_t kOutRows = 80; // X: [-2.0 m, +6.0 m), CoR na ix = 20
    static constexpr size_t kOutCols = 80; // Y: [-4.0 m, +4.0 m) CW, CoR na iy = 40
    static constexpr size_t kMemSize = 140; // 14 × 14 m vnitřní severová paměť

    struct PoseState {
        double heading_deg = 0.0;
        double pitch_deg = 0.0;
        double roll_deg = 0.0;
        double lat = 0.0;
        double lon = 0.0;
        bool has_lat_lon = false;
    };

    struct EvaluationResult {
        double timestamp = 0.0;
        double heading_deg = 0.0;
        double pitch_deg = 0.0;
        double roll_deg = 0.0;
        size_t points_count = 0;
        double duration_ms = 0.0;

        uint32_t valid_cells = 0;
        uint32_t blocked_cells = 0;
        float min_corridor_clearance_cm = 600.0f;
        size_t traversable_cells_count = 0;
        uint8_t min_cost_front = 100;

        // Výstupní potenciálová mřížka 80 × 80 (hodnoty 0..100, kde 0 = ideální hladká rovina, 100 = neprůjezdné/neviděné)
        // Indexace: grid[ix * 80 + iy], kde ix ∈ [0..79] je X (-200..+600 cm), iy ∈ [0..79] je Y (-400..+400 cm CW)
        std::array<uint8_t, kOutRows * kOutCols> cost_grid{};
        std::string grid_b64;
        std::string json_payload;
    };

    explicit TraversabilityEvaluator(TraversabilityConfig config = TraversabilityConfig{})
        : config_(config)
    {
        reset();
    }

    void reset() {
        for (size_t i = 0; i < kMemSize; ++i) {
            for (size_t j = 0; j < kMemSize; ++j) {
                mem_grid_[i][j] = MemCell{};
            }
        }
        has_anchor_ = false;
        anchor_lat_ = 0.0;
        anchor_lon_ = 0.0;
        has_initial_plane_ = false;
    }

    bool evaluate(const std::vector<LidarPointBuffer::Sample> &samples,
                  const PoseState &pose,
                  double timestamp,
                  EvaluationResult &out_result)
    {
        double lat = pose.has_lat_lon ? pose.lat : 0.0;
        double lon = pose.has_lat_lon ? pose.lon : 0.0;
        return evaluate(samples, lat, lon, pose.heading_deg, pose.pitch_deg, pose.roll_deg, timestamp, out_result);
    }

    bool evaluate(const std::vector<LidarPointBuffer::Sample> &samples,
                  double lat_deg,
                  double lon_deg,
                  double heading_deg,
                  double pitch_deg,
                  double roll_deg,
                  double timestamp,
                  EvaluationResult &out_result)
    {
        auto start_time = std::chrono::steady_clock::now();

        if (samples.size() < config_.min_points_per_cycle) {
            return false;
        }

        // 1. Posun severově stabilizované paměťové mřížky podle WGS84 polohy z fusion
        double d_north_cm = 0.0;
        double d_east_cm  = 0.0;
        updateMemoryAnchor(lat_deg, lon_deg, d_north_cm, d_east_cm);

        // 2. Příprava rotačních matic:
        //    a) Narovnání bodů do horizontální roviny kolmé ke gravitaci (pitch, roll)
        //    b) Rotace do severu (heading_deg CW)
        const float deg2rad = static_cast<float>(M_PI) / 180.0f;
        const float pitch_rad = static_cast<float>(pitch_deg) * deg2rad;
        const float roll_rad  = static_cast<float>(roll_deg) * deg2rad;
        const float hdg_rad   = static_cast<float>(heading_deg) * deg2rad;

        const float cp = std::cos(pitch_rad), sp = std::sin(pitch_rad);
        const float cr = std::cos(roll_rad),  sr = std::sin(roll_rad);
        const float ch = std::cos(hdg_rad),   sh = std::sin(hdg_rad);

        // Pomocná akumulační mřížka pro aktuální 100ms snímek
        for (size_t i = 0; i < kMemSize; ++i) {
            for (size_t j = 0; j < kMemSize; ++j) {
                frame_accum_[i][j].reset();
            }
        }

        const float half_mem_cm = (static_cast<float>(kMemSize) * config_.cell_size_cm) * 0.5f; // 700 cm

        // 2b. Inicializace počáteční roviny při startu (předpoklad: obsluha spustila robota na sjízdném místě)
        if (config_.use_init_plane && !has_initial_plane_) {
            for (float xb = config_.init_plane_x_min_cm; xb <= config_.init_plane_x_max_cm; xb += config_.cell_size_cm * 0.5f) {
                for (float yb = config_.init_plane_y_min_cm; yb <= config_.init_plane_y_max_cm; yb += config_.cell_size_cm * 0.5f) {
                    float north_cm = static_cast<float>(d_north_cm) + (xb * ch - yb * sh);
                    float east_cm  = static_cast<float>(d_east_cm)  + (xb * sh + yb * ch);
                    int mi = static_cast<int>(std::floor((north_cm + half_mem_cm) / config_.cell_size_cm));
                    int mj = static_cast<int>(std::floor((east_cm  + half_mem_cm) / config_.cell_size_cm));
                    if (mi >= 0 && mi < static_cast<int>(kMemSize) &&
                        mj >= 0 && mj < static_cast<int>(kMemSize)) {
                        auto &mem = mem_grid_[mi][mj];
                        if (!mem.seen) {
                            mem.z_mean = 0.0f;
                            mem.z_min  = 0.0f;
                            mem.z_max  = 0.0f;
                            mem.roughness_cm = 0.0f;
                            mem.is_high_obstacle = false;
                            mem.seen = true;
                            mem.last_seen_ts = timestamp;
                        }
                    }
                }
            }
            has_initial_plane_ = true;
        }

        // 3. Průchod 3D body a projekce do severově stabilizované mřížky
        for (const auto &p : samples) {
            // Převod z LiDARu na střed otáčení robota (CoR)
            // V kalibrovaném rámci C: +X je vpřed, +Y je vlevo, +Z je nahoru
            float xr = p.x + config_.lidar_offset_x_cm;
            float yr = p.y;
            float zr = p.z;

            float d2 = xr * xr + yr * yr;
            if (d2 < config_.min_valid_dist_cm * config_.min_valid_dist_cm ||
                d2 > config_.max_range_cm * config_.max_range_cm) {
                continue;
            }

            // Korekce náklonu robota (pitch kolem Y, roll kolem X) -> soustava kolmá ke gravitaci
            float x_grav =  cp * xr + sp * sr * yr + sp * cr * zr;
            float y_grav =  cr * yr - sr * zr;
            float z_grav = -sp * xr + cp * sr * yr + cp * cr * zr;

            if (z_grav < config_.ground_z_min_cm || z_grav > config_.obstacle_z_max_cm) {
                continue;
            }

            // Převod příčné osy na clockwise konvenci (+Y_cw = doprava = -y_grav)
            float y_cw = -y_grav;

            // Rotace do severu (North-East v cm vůči kotvě paměťové mřížky)
            float north_cm = static_cast<float>(d_north_cm) + (x_grav * ch - y_cw * sh);
            float east_cm  = static_cast<float>(d_east_cm)  + (x_grav * sh + y_cw * ch);

            int mi = static_cast<int>(std::floor((north_cm + half_mem_cm) / config_.cell_size_cm));
            int mj = static_cast<int>(std::floor((east_cm  + half_mem_cm) / config_.cell_size_cm));

            if (mi < 0 || mi >= static_cast<int>(kMemSize) ||
                mj < 0 || mj >= static_cast<int>(kMemSize)) {
                continue;
            }

            auto &acc = frame_accum_[mi][mj];
            if (z_grav < acc.z_min) acc.z_min = z_grav;
            if (z_grav > acc.z_max) acc.z_max = z_grav;
            acc.z_sum += static_cast<double>(z_grav);
            acc.z_sq_sum += static_cast<double>(z_grav) * static_cast<double>(z_grav);
            acc.count++;
        }

        // 4. Aktualizace dlouhodobé paměti z aktuálního snímku a vypršení starých záznamů
        for (size_t i = 0; i < kMemSize; ++i) {
            for (size_t j = 0; j < kMemSize; ++j) {
                auto &mem = mem_grid_[i][j];
                const auto &acc = frame_accum_[i][j];

                if (mem.seen && timestamp > mem.last_seen_ts &&
                    (timestamp - mem.last_seen_ts) > config_.memory_decay_sec) {
                    mem = MemCell{};
                }

                if (acc.count >= config_.min_ground_pts_cell) {
                    float mean_z  = static_cast<float>(acc.z_sum / static_cast<double>(acc.count));
                    double var_z  = (acc.z_sq_sum / static_cast<double>(acc.count)) - (static_cast<double>(mean_z) * static_cast<double>(mean_z));
                    float std_z   = (var_z > 0.0) ? static_cast<float>(std::sqrt(var_z)) : 0.0f;
                    float span_z  = acc.z_max - acc.z_min;
                    // Drsnost počítáme ze směrodatné odchylky (odolnost vůči šumu dálkoměru)
                    // Pro méně než 6 bodů použijeme korigované vertikální rozpětí
                    float roughness = (acc.count >= 6) ? (2.5f * std_z) : (span_z * 0.7f);

                    // Pokud vertikální rozpětí v buňce přesahuje 15 cm nebo minimum leží vysoko nad zemí, jde o 3D překážku
                    bool is_high_obs = (span_z > 15.0f) || (acc.z_min > config_.obstacle_z_min_cm && acc.z_min < config_.obstacle_z_max_cm);

                    if (!mem.seen) {
                        mem.z_mean = mean_z;
                        mem.z_min  = acc.z_min;
                        mem.z_max  = acc.z_max;
                        mem.roughness_cm = roughness;
                        mem.is_high_obstacle = is_high_obs;
                        mem.seen = true;
                    } else {
                        // Exponenciální vyhlazení s pamětí
                        constexpr float alpha = 0.5f;
                        mem.z_mean = (1.0f - alpha) * mem.z_mean + alpha * mean_z;
                        mem.z_min  = (1.0f - alpha) * mem.z_min  + alpha * acc.z_min;
                        mem.z_max  = (1.0f - alpha) * mem.z_max  + alpha * acc.z_max;
                        mem.roughness_cm = (1.0f - alpha) * mem.roughness_cm + alpha * roughness;
                        mem.is_high_obstacle = is_high_obs || (mem.is_high_obstacle && roughness > config_.max_roughness_cm);
                    }
                    mem.last_seen_ts = timestamp;
                }
            }
        }

        // 5. Vzorkování 80×80 okna v těle robota (X ∈ [-200, +600) cm, Y ∈ [-400, +400) cm CW)
        //    a výpočet surové terénní ceny raw_cost (schod > 4 cm, drsnost > 4 cm, sklon > 25°, 3D překážka)
        uint8_t raw_cost[kOutRows][kOutCols];
        bool    seen_mask[kOutRows][kOutCols];
        bool    obs_mask[kOutRows][kOutCols];
        float   body_z[kOutRows][kOutCols];

        for (size_t ix = 0; ix < kOutRows; ++ix) {
            float x_body_cm = config_.out_x_min_cm + (static_cast<float>(ix) + 0.5f) * config_.cell_size_cm;
            for (size_t iy = 0; iy < kOutCols; ++iy) {
                float y_body_cw_cm = config_.out_y_min_cm + (static_cast<float>(iy) + 0.5f) * config_.cell_size_cm;

                // Pod půdorysem samotného robota (okolí CoR, kde šasi stíní paprsky) považujeme terén za již prověřený/hladký
                if (x_body_cm >= -55.0f && x_body_cm <= 45.0f && std::abs(y_body_cw_cm) <= 35.0f) {
                    seen_mask[ix][iy] = true;
                    obs_mask[ix][iy]  = false;
                    raw_cost[ix][iy]  = 0;
                    body_z[ix][iy]    = 0.0f;
                    continue;
                }

                float north_cm = static_cast<float>(d_north_cm) + (x_body_cm * ch - y_body_cw_cm * sh);
                float east_cm  = static_cast<float>(d_east_cm)  + (x_body_cm * sh + y_body_cw_cm * ch);

                int mi = static_cast<int>(std::floor((north_cm + half_mem_cm) / config_.cell_size_cm));
                int mj = static_cast<int>(std::floor((east_cm  + half_mem_cm) / config_.cell_size_cm));

                if (mi >= 0 && mi < static_cast<int>(kMemSize) &&
                    mj >= 0 && mj < static_cast<int>(kMemSize) &&
                    mem_grid_[mi][mj].seen) {
                    const auto &c = mem_grid_[mi][mj];
                    seen_mask[ix][iy] = true;
                    body_z[ix][iy]    = c.z_mean;

                    if (c.is_high_obstacle || c.roughness_cm > config_.max_roughness_cm) {
                        obs_mask[ix][iy] = true;
                        raw_cost[ix][iy] = 100;
                    } else {
                        obs_mask[ix][iy] = false;
                        float r_ratio = c.roughness_cm / config_.max_roughness_cm;
                        raw_cost[ix][iy] = static_cast<uint8_t>(std::clamp(std::lround(50.0f * r_ratio), 0L, 50L));
                    }
                } else {
                    seen_mask[ix][iy] = false;
                    obs_mask[ix][iy]  = false;
                    raw_cost[ix][iy]  = config_.unseen_cell_cost;
                    body_z[ix][iy]    = 0.0f;
                }
            }
        }

        // 5b. Vyhodnocení mezibuněčného schodu (> 4 cm), sklonu ke gravitaci (> 25°)
        //     a vnitřních děr (voda/propad uvnitř osvětlené oblasti)
        const float rad2deg = 180.0f / static_cast<float>(M_PI);

        for (size_t ix = 1; ix + 1 < kOutRows; ++ix) {
            for (size_t iy = 1; iy + 1 < kOutCols; ++iy) {
                if (!seen_mask[ix][iy]) {
                    // Pokud je neviděná buňka obklopena viděnými buňkami (např. louže vody nebo díra v zemi),
                    // označíme ji jako aktivní překážku generující odtlačovací pole
                    int seen_neighbors = 0;
                    for (int dx = -1; dx <= 1; ++dx) {
                        for (int dy = -1; dy <= 1; ++dy) {
                            if (seen_mask[ix + dx][iy + dy]) seen_neighbors++;
                        }
                    }
                    if (seen_neighbors >= 5) {
                        obs_mask[ix][iy] = true;
                    }
                    continue;
                }

                if (obs_mask[ix][iy]) continue;

                float z0 = body_z[ix][iy];
                float max_step = 0.0f;
                float dz_dx = 0.0f;
                float dz_dy = 0.0f;

                for (int dx = -1; dx <= 1; ++dx) {
                    for (int dy = -1; dy <= 1; ++dy) {
                        if (dx == 0 && dy == 0) continue;
                        if (!seen_mask[ix + dx][iy + dy]) continue;
                        float zn = body_z[ix + dx][iy + dy];
                        float diff = std::abs(z0 - zn);
                        if (diff > max_step) max_step = diff;
                    }
                }

                if (seen_mask[ix + 1][iy] && seen_mask[ix - 1][iy]) {
                    dz_dx = (body_z[ix + 1][iy] - body_z[ix - 1][iy]) / (2.0f * config_.cell_size_cm);
                }
                if (seen_mask[ix][iy + 1] && seen_mask[ix][iy - 1]) {
                    dz_dy = (body_z[ix][iy + 1] - body_z[ix][iy - 1]) / (2.0f * config_.cell_size_cm);
                }

                float grad_norm = std::sqrt(dz_dx * dz_dx + dz_dy * dz_dy);
                float slope_deg = std::atan(grad_norm) * rad2deg;

                if (max_step > config_.max_step_cm || slope_deg > config_.max_slope_deg) {
                    obs_mask[ix][iy] = true;
                    raw_cost[ix][iy] = 100;
                } else {
                    float step_ratio  = max_step / config_.max_step_cm;
                    float slope_ratio = slope_deg / config_.max_slope_deg;
                    uint8_t extra_cost = static_cast<uint8_t>(
                        std::clamp(std::lround(50.0f * std::max(step_ratio, slope_ratio)), 0L, 50L));
                    if (extra_cost > raw_cost[ix][iy]) {
                        raw_cost[ix][iy] = extra_cost;
                    }
                }
            }
        }

        // 6. Výpočet 2D euklidovské vzdálenostní transformace (Distance Transform) od překážek
        //    a sestavení výsledného potenciálového pole (šířka robota ±35 cm + odtlačovací polštář do 100 cm)
        computePotentialField(raw_cost, seen_mask, obs_mask, out_result.cost_grid);

        // Statistiky a Base64 zakódování
        size_t trav_count = 0;
        uint32_t valid_cnt = 0;
        uint32_t blocked_cnt = 0;
        uint8_t min_front = 100;
        for (size_t ix = 0; ix < kOutRows; ++ix) {
            for (size_t iy = 0; iy < kOutCols; ++iy) {
                if (seen_mask[ix][iy]) valid_cnt++;
                if (obs_mask[ix][iy]) blocked_cnt++;
                uint8_t c = out_result.cost_grid[ix * kOutCols + iy];
                if (c < 100) trav_count++;
                if (ix >= 25 && ix <= 45 && iy >= 30 && iy <= 50 && c < min_front) {
                    min_front = c;
                }
            }
        }

        // Výpočet volné délky v přímém koridoru před robotem (od ix = 25, tj. +50 cm před CoR, na ose iy = 40)
        // Pozn.: cost_grid již zahrnuje inflaci o poloviční šířku robota (±35 cm)
        float corridor_cm = 600.0f;
        for (size_t ix = 25; ix < kOutRows; ++ix) {
            if (out_result.cost_grid[ix * kOutCols + 40] >= 100) {
                corridor_cm = static_cast<float>(ix - 20) * config_.cell_size_cm;
                break;
            }
        }

        auto end_time = std::chrono::steady_clock::now();
        double duration_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

        out_result.timestamp                 = timestamp;
        out_result.heading_deg               = heading_deg;
        out_result.pitch_deg                 = pitch_deg;
        out_result.roll_deg                  = roll_deg;
        out_result.points_count              = samples.size();
        out_result.duration_ms               = duration_ms;
        out_result.valid_cells               = valid_cnt;
        out_result.blocked_cells             = blocked_cnt;
        out_result.min_corridor_clearance_cm = corridor_cm;
        out_result.traversable_cells_count   = trav_count;
        out_result.min_cost_front            = min_front;
        out_result.grid_b64                  = encodeBase64(out_result.cost_grid.data(), out_result.cost_grid.size());
        out_result.json_payload              = buildJsonPayload(out_result);

        return true;
    }

private:
    struct MemCell {
        float z_mean = 0.0f;
        float z_min = 0.0f;
        float z_max = 0.0f;
        float roughness_cm = 0.0f;
        double last_seen_ts = 0.0;
        bool is_high_obstacle = false;
        bool seen = false;
    };

    struct FrameAccum {
        float z_min = 1e9f;
        float z_max = -1e9f;
        double z_sum = 0.0;
        double z_sq_sum = 0.0;
        size_t count = 0;

        void reset() {
            z_min = 1e9f;
            z_max = -1e9f;
            z_sum = 0.0;
            z_sq_sum = 0.0;
            count = 0;
        }
    };

    void updateMemoryAnchor(double lat_deg, double lon_deg, double &rem_north_cm, double &rem_east_cm) {
        if (!std::isfinite(lat_deg) || !std::isfinite(lon_deg) || (std::abs(lat_deg) < 1e-5 && std::abs(lon_deg) < 1e-5)) {
            rem_north_cm = 0.0;
            rem_east_cm  = 0.0;
            return;
        }

        if (!has_anchor_) {
            anchor_lat_  = lat_deg;
            anchor_lon_  = lon_deg;
            has_anchor_  = true;
            rem_north_cm = 0.0;
            rem_east_cm  = 0.0;
            return;
        }

        constexpr double kMetersPerDegLat = 111132.95;
        double meters_per_deg_lon = 111412.84 * std::cos(anchor_lat_ * M_PI / 180.0);

        double total_north_cm = (lat_deg - anchor_lat_) * kMetersPerDegLat * 100.0;
        double total_east_cm  = (lon_deg - anchor_lon_) * meters_per_deg_lon * 100.0;

        // Pokud došlo ke skoku > 5 m (např. reinicializace GPS), resetujeme mřížku
        if (std::hypot(total_north_cm, total_east_cm) > 500.0) {
            reset();
            anchor_lat_  = lat_deg;
            anchor_lon_  = lon_deg;
            has_anchor_  = true;
            rem_north_cm = 0.0;
            rem_east_cm  = 0.0;
            return;
        }

        int shift_n = static_cast<int>(std::trunc(total_north_cm / config_.cell_size_cm));
        int shift_e = static_cast<int>(std::trunc(total_east_cm  / config_.cell_size_cm));

        if (shift_n != 0 || shift_e != 0) {
            MemCell new_grid[kMemSize][kMemSize];
            for (int i = 0; i < static_cast<int>(kMemSize); ++i) {
                for (int j = 0; j < static_cast<int>(kMemSize); ++j) {
                    int src_i = i + shift_n;
                    int src_j = j + shift_e;
                    if (src_i >= 0 && src_i < static_cast<int>(kMemSize) &&
                        src_j >= 0 && src_j < static_cast<int>(kMemSize)) {
                        new_grid[i][j] = mem_grid_[src_i][src_j];
                    }
                }
            }
            for (size_t i = 0; i < kMemSize; ++i) {
                for (size_t j = 0; j < kMemSize; ++j) {
                    mem_grid_[i][j] = new_grid[i][j];
                }
            }

            anchor_lat_ += (shift_n * config_.cell_size_cm / 100.0) / kMetersPerDegLat;
            anchor_lon_ += (shift_e * config_.cell_size_cm / 100.0) / meters_per_deg_lon;

            total_north_cm = (lat_deg - anchor_lat_) * kMetersPerDegLat * 100.0;
            total_east_cm  = (lon_deg - anchor_lon_) * meters_per_deg_lon * 100.0;
        }

        rem_north_cm = total_north_cm;
        rem_east_cm  = total_east_cm;
    }

    void computePotentialField(const uint8_t raw_cost[kOutRows][kOutCols],
                               const bool seen_mask[kOutRows][kOutCols],
                               const bool obs_mask[kOutRows][kOutCols],
                               std::array<uint8_t, kOutRows * kOutCols> &out_grid) const
    {
        constexpr int kInf = 1000000;
        int dist_sq[kOutRows][kOutCols];

        // 1. Průchod (po řádcích) pro přesnou 2D euklidovskou vzdálenost k nejbližší překážce
        for (size_t ix = 0; ix < kOutRows; ++ix) {
            int last_obs = -1000;
            for (int iy = 0; iy < static_cast<int>(kOutCols); ++iy) {
                if (obs_mask[ix][iy]) last_obs = iy;
                int d = iy - last_obs;
                dist_sq[ix][iy] = d * d;
            }
            last_obs = 1000;
            for (int iy = static_cast<int>(kOutCols) - 1; iy >= 0; --iy) {
                if (obs_mask[ix][iy]) last_obs = iy;
                int d = last_obs - iy;
                if (d * d < dist_sq[ix][iy]) dist_sq[ix][iy] = d * d;
            }
        }

        // 2. Průchod (po sloupcích) a vyhodnocení potenciálu
        const int max_rad_cells = static_cast<int>(std::ceil(config_.repulse_radius_cm / config_.cell_size_cm));
        for (size_t iy = 0; iy < kOutCols; ++iy) {
            int col_d2[kOutRows];
            for (size_t ix = 0; ix < kOutRows; ++ix) {
                col_d2[ix] = dist_sq[ix][iy];
            }

            for (int ix = 0; ix < static_cast<int>(kOutRows); ++ix) {
                // Pokud buňku LiDAR nikdy neviděl, dostane nastavenou penalizaci (neblokuje jako zeď)
                if (!seen_mask[ix][iy]) {
                    out_grid[ix * kOutCols + iy] = config_.unseen_cell_cost;
                    continue;
                }

                if (obs_mask[ix][iy]) {
                    out_grid[ix * kOutCols + iy] = 100;
                    continue;
                }

                int min_d2 = col_d2[ix];
                int r_start = std::max(0, ix - max_rad_cells);
                int r_end   = std::min(static_cast<int>(kOutRows) - 1, ix + max_rad_cells);
                for (int kx = r_start; kx <= r_end; ++kx) {
                    int dx = ix - kx;
                    int cand = col_d2[kx] + dx * dx;
                    if (cand < min_d2) min_d2 = cand;
                }

                float d_obs_cm = std::sqrt(static_cast<float>(min_d2)) * config_.cell_size_cm;
                uint8_t repulse = 0;

                if (d_obs_cm <= config_.robot_half_width_cm) {
                    repulse = 100; // Zakázaný pás pro střed robota (šířka ±35 cm)
                } else if (d_obs_cm < config_.repulse_radius_cm) {
                    float ratio = (config_.repulse_radius_cm - d_obs_cm) /
                                  (config_.repulse_radius_cm - config_.robot_half_width_cm);
                    repulse = static_cast<uint8_t>(std::clamp(std::lround(85.0f * ratio * ratio), 0L, 99L));
                }

                out_grid[ix * kOutCols + iy] = std::max(raw_cost[ix][iy], repulse);
            }
        }
    }

    static std::string encodeBase64(const uint8_t *data, size_t len) {
        static constexpr char kTable[] =
            "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        std::string out;
        out.reserve(((len + 2) / 3) * 4);

        for (size_t i = 0; i < len; i += 3) {
            uint32_t n = static_cast<uint32_t>(data[i]) << 16;
            if (i + 1 < len) n |= static_cast<uint32_t>(data[i + 1]) << 8;
            if (i + 2 < len) n |= static_cast<uint32_t>(data[i + 2]);

            out.push_back(kTable[(n >> 18) & 0x3F]);
            out.push_back(kTable[(n >> 12) & 0x3F]);
            out.push_back((i + 1 < len) ? kTable[(n >> 6) & 0x3F] : '=');
            out.push_back((i + 2 < len) ? kTable[n & 0x3F] : '=');
        }
        return out;
    }

    std::string buildJsonPayload(const EvaluationResult &res) const {
        std::ostringstream ss;
        ss << "{"
           << "\"timestamp\":" << std::fixed << std::setprecision(3) << res.timestamp << ","
           << "\"heading\":" << std::setprecision(1) << res.heading_deg << ","
           << "\"pitch\":" << std::setprecision(1) << res.pitch_deg << ","
           << "\"roll\":" << std::setprecision(1) << res.roll_deg << ","
           << "\"grid_nx\":80,"
           << "\"grid_ny\":80,"
           << "\"cell_size_cm\":10,"
           << "\"origin_ix\":20,"
           << "\"origin_iy\":40,"
           << "\"valid_cells\":" << res.valid_cells << ","
           << "\"blocked_cells\":" << res.blocked_cells << ","
           << "\"traversable_cells\":" << res.traversable_cells_count << ","
           << "\"min_corridor_clearance_cm\":" << std::setprecision(1) << res.min_corridor_clearance_cm << ","
           << "\"min_cost_front\":" << static_cast<int>(res.min_cost_front) << ","
           << "\"grid_b64\":\"" << res.grid_b64 << "\""
           << "}";
        return ss.str();
    }

    TraversabilityConfig config_;
    MemCell mem_grid_[kMemSize][kMemSize];
    FrameAccum frame_accum_[kMemSize][kMemSize];
    bool has_anchor_{false};
    bool has_initial_plane_{false};
    double anchor_lat_{0.0};
    double anchor_lon_{0.0};
};

