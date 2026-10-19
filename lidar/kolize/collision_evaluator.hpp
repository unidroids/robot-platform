#pragma once

#include "collision_config.hpp"
#include "../point_buffer.hpp"

#include <vector>
#include <cmath>
#include <algorithm>
#include <string>
#include <sstream>
#include <iomanip>
#include <chrono>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Matematické jádro pro vyhodnocení kolizních kurzů v mřížce 12 pater × 36 sektorů
class CollisionEvaluator {
public:
    struct SectorOutput {
        size_t sector_idx = 0;
        float  azimuth_deg = 0.0f;

        // Profil A: Průjezdnost (Clearance) - reálné minimum napříč patry (neinflační)
        float  clearance_dist_cm = 2000.0f;
        float  clearance_closing_speed_ms = 0.0f;
        size_t clearance_floor_idx = 0;

        // Profil B: Kolizní kurz (TTC) - vyhodnoceno s inflací válcem těla robota
        float  ttc_sec = 999.0f;
        float  threat_dist_cm = 2000.0f;
        float  threat_closing_speed_ms = 0.0f;
        size_t threat_floor_idx = 0;
    };

    struct EvaluationResult {
        double timestamp = 0.0;
        double heading_start = 0.0;
        double heading_end = 0.0;
        double heading_avg = 0.0;
        size_t points_count = 0;
        double duration_ms = 0.0;

        float min_clearance_cm = 2000.0f;
        float min_ttc_s = 999.0f;

        std::vector<SectorOutput> sectors;
        std::string json_payload;
    };

    explicit CollisionEvaluator(CollisionConfig config = CollisionConfig{})
        : config_(config)
    {
        reset();
    }

    void reset() {
        for (size_t f = 0; f < config_.num_floors; ++f) {
            for (size_t s = 0; s < config_.num_sectors; ++s) {
                grid_[f][s] = CellTrack{};
                grid_[f][s].d_est = config_.max_range_cm;
            }
        }
        consecutive_valid_frames_ = 0;
    }

    uint32_t getConsecutiveValidFrames() const {
        return consecutive_valid_frames_;
    }

    // Vyhodnotí dávku vzorků z posledních 100 ms
    bool evaluate(const std::vector<LidarPointBuffer::Sample> &samples,
                  double heading_start,
                  double heading_end,
                  double heading_avg,
                  double timestamp,
                  EvaluationResult &out_result)
    {
        auto start_time = std::chrono::steady_clock::now();

        // 1. Kontrola počtu bodů (min 10 000 bodů za 100 ms)
        if (samples.size() < config_.min_points_per_cycle) {
            consecutive_valid_frames_ = 0;
            return false;
        }

        // 2. Akumulace měření v mřížce 12 pater × 36 sektorů
        float meas_d[12][36];
        uint32_t meas_count[12][36];

        for (size_t f = 0; f < config_.num_floors; ++f) {
            for (size_t s = 0; s < config_.num_sectors; ++s) {
                meas_d[f][s] = config_.max_range_cm;
                meas_count[f][s] = 0;
            }
        }

        const float rad2deg = 180.0f / static_cast<float>(M_PI);

        for (const auto &p : samples) {
            // Kontrola výšky
            if (p.z < config_.z_min_cm || p.z >= config_.z_max_cm) {
                continue;
            }

            size_t f = static_cast<size_t>((p.z - config_.z_min_cm) / config_.floor_height_cm);
            if (f >= config_.num_floors) {
                continue;
            }

            // Převod na střed otáčení robota (Center of Rotation)
            float xr = p.x + config_.lidar_offset_x_cm;
            float yr = p.y;
            float d  = std::sqrt(xr * xr + yr * yr);

            if (d < config_.min_valid_dist_cm || d > config_.max_range_cm) {
                continue;
            }

            // Lokální azimut v rovině robota
            float local_deg = std::atan2(yr, xr) * rad2deg;

            // Globální azimut stabilizovaný na sever
            float global_deg = std::fmod(local_deg + static_cast<float>(heading_avg) + 360.0f, 360.0f);
            if (global_deg < 0.0f) global_deg += 360.0f;

            size_t s = static_cast<size_t>(global_deg / config_.sector_angle_deg) % config_.num_sectors;

            if (d < meas_d[f][s]) {
                meas_d[f][s] = d;
            }
            meas_count[f][s]++;
        }

        // 3. Dynamická filtrace Alfa-Beta a vertikální provázanost pater
        for (size_t s = 0; s < config_.num_sectors; ++s) {
            for (size_t f = 0; f < config_.num_floors; ++f) {
                auto &track = grid_[f][s];
                bool has_meas = (meas_count[f][s] >= config_.min_points_per_cell);
                float dm = meas_d[f][s];

                float d_pred = track.active ? (track.d_est - track.v_est * config_.dt_sec) : config_.max_range_cm;

                // Vertikální handover: pokud v tomto patře měření chybí, zkontroluj sousedy (míč / nerovnosti)
                if (!has_meas && track.active) {
                    size_t f_down = (f > 0) ? (f - 1) : f;
                    size_t f_up   = (f + 1 < config_.num_floors) ? (f + 1) : f;

                    if (f_down != f && meas_count[f_down][s] >= config_.min_points_per_cell) {
                        float cand_d = meas_d[f_down][s];
                        if (std::abs(cand_d - d_pred) < config_.max_track_jump_cm) {
                            dm = cand_d;
                            has_meas = true;
                        }
                    } else if (f_up != f && meas_count[f_up][s] >= config_.min_points_per_cell) {
                        float cand_d = meas_d[f_up][s];
                        if (std::abs(cand_d - d_pred) < config_.max_track_jump_cm) {
                            dm = cand_d;
                            has_meas = true;
                        }
                    }
                }

                if (has_meas) {
                    if (!track.active || std::abs(dm - d_pred) > config_.max_track_jump_cm) {
                        // Nový track
                        track.d_est = dm;
                        track.v_est = 0.0f;
                        track.hits  = 1;
                        track.missed_frames = 0;
                        track.active = true;
                    } else {
                        // Inkrementální aktualizace alfa-beta
                        float residual = d_pred - dm;
                        track.d_est = d_pred - config_.alpha * residual;
                        track.v_est = track.v_est + (config_.beta / config_.dt_sec) * residual;
                        track.hits++;
                        track.missed_frames = 0;
                        track.active = true;
                    }
                } else {
                    if (track.active) {
                        track.missed_frames++;
                        if (track.missed_frames > 2) {
                            track.active = false;
                            track.d_est = config_.max_range_cm;
                            track.v_est = 0.0f;
                            track.hits  = 0;
                        } else {
                            track.d_est = d_pred; // setrvačnost
                        }
                    }
                }
            }
        }

        consecutive_valid_frames_++;
        if (consecutive_valid_frames_ < 2) {
            // Potřebujeme alespoň 2 po sobě jdoucí platné snímky pro spolehlivou derivaci rychlostí
            return false;
        }

        // 4. Výpočet inflace a TTC pro každý sektor
        float ttc_min[36];
        float threat_d[36];
        float threat_v[36];
        size_t threat_floor[36];

        for (size_t s = 0; s < config_.num_sectors; ++s) {
            ttc_min[s]      = 999.0f;
            threat_d[s]     = config_.max_range_cm;
            threat_v[s]     = 0.0f;
            threat_floor[s] = 0;
        }

        for (size_t f = 0; f < config_.num_floors; ++f) {
            for (size_t s = 0; s < config_.num_sectors; ++s) {
                const auto &track = grid_[f][s];
                // Uvažujeme jen aktivní objekty, které se přibližují (v > 5 cm/s = 0.05 m/s)
                if (!track.active || track.d_est >= config_.max_range_cm || track.v_est <= 5.0f) {
                    continue;
                }

                float v_ms = track.v_est / 100.0f;
                float d_m  = track.d_est / 100.0f;
                float ttc  = d_m / v_ms;

                // Úhlové rozšíření kuželu (inflace o poloměr robota R = 37.5 cm)
                float ratio = std::min(1.0f, config_.robot_radius_cm / track.d_est);
                float delta_rad = std::asin(ratio);
                float delta_deg = delta_rad * rad2deg;
                int k = static_cast<int>(std::ceil(delta_deg / config_.sector_angle_deg));

                for (int off = -k; off <= k; ++off) {
                    int target_s = (static_cast<int>(s) + off + static_cast<int>(config_.num_sectors)) % static_cast<int>(config_.num_sectors);
                    if (ttc < ttc_min[target_s]) {
                        ttc_min[target_s]      = ttc;
                        threat_d[target_s]     = track.d_est;
                        threat_v[target_s]     = v_ms;
                        threat_floor[target_s] = f;
                    }
                }
            }
        }

        // 5. Syntéza Profilu A (Clearance) a Profilu B (Kolize) pro 36 sektorů
        out_result.sectors.resize(config_.num_sectors);
        float global_min_clearance = config_.max_range_cm;
        float global_min_ttc = 999.0f;

        for (size_t s = 0; s < config_.num_sectors; ++s) {
            auto &sec = out_result.sectors[s];
            sec.sector_idx  = s;
            sec.azimuth_deg = static_cast<float>(s) * config_.sector_angle_deg;

            // Profil A: Clearance (průjezdnost)
            float min_d = config_.max_range_cm;
            float min_v = 0.0f;
            size_t min_f = 0;

            for (size_t f = 0; f < config_.num_floors; ++f) {
                if (grid_[f][s].active && grid_[f][s].d_est < min_d) {
                    min_d = grid_[f][s].d_est;
                    min_v = grid_[f][s].v_est / 100.0f;
                    min_f = f;
                }
            }

            sec.clearance_dist_cm          = min_d;
            sec.clearance_closing_speed_ms = min_v;
            sec.clearance_floor_idx        = min_f;

            if (min_d < global_min_clearance) {
                global_min_clearance = min_d;
            }

            // Profil B: Kolize (TTC)
            sec.ttc_sec                 = ttc_min[s];
            sec.threat_dist_cm          = threat_d[s];
            sec.threat_closing_speed_ms = threat_v[s];
            sec.threat_floor_idx        = threat_floor[s];

            if (ttc_min[s] < global_min_ttc) {
                global_min_ttc = ttc_min[s];
            }
        }

        auto end_time = std::chrono::steady_clock::now();
        double duration_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

        out_result.timestamp        = timestamp;
        out_result.heading_start    = heading_start;
        out_result.heading_end      = heading_end;
        out_result.heading_avg      = heading_avg;
        out_result.points_count     = samples.size();
        out_result.duration_ms      = duration_ms;
        out_result.min_clearance_cm = global_min_clearance;
        out_result.min_ttc_s        = global_min_ttc;

        // 6. Generování výstupního JSONu
        out_result.json_payload = buildJsonPayload(out_result);

        return true;
    }

private:
    struct CellTrack {
        float d_est = 2000.0f;      // Vyhlazená vzdálenost [cm]
        float v_est = 0.0f;         // Rychlost přibližování [cm/s] (kladná = přibližování)
        uint32_t hits = 0;          // Počet navazujících detekcí
        uint32_t missed_frames = 0; // Počet po sobě chybějících měření
        bool active = false;
    };

    std::string buildJsonPayload(const EvaluationResult &res) const {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2);

        ss << "{"
           << "\"timestamp\":" << std::setprecision(3) << res.timestamp << ","
           << "\"heading_start\":" << std::setprecision(1) << res.heading_start << ","
           << "\"heading_end\":" << res.heading_end << ","
           << "\"heading_avg\":" << res.heading_avg << ","
           << "\"points_count\":" << res.points_count << ","
           << "\"eval_duration_ms\":" << std::setprecision(2) << res.duration_ms << ","
           << "\"sectors\":[";

        for (size_t i = 0; i < res.sectors.size(); ++i) {
            const auto &s = res.sectors[i];
            if (i > 0) ss << ",";
            ss << "{"
               << "\"sector_idx\":" << s.sector_idx << ","
               << "\"azimuth_deg\":" << s.azimuth_deg << ","
               << "\"clearance\":{"
               << "\"distance_cm\":" << s.clearance_dist_cm << ","
               << "\"closing_speed_ms\":" << s.clearance_closing_speed_ms << ","
               << "\"floor_idx\":" << s.clearance_floor_idx
               << "},"
               << "\"collision\":{"
               << "\"ttc_sec\":" << s.ttc_sec << ","
               << "\"threat_distance_cm\":" << s.threat_dist_cm << ","
               << "\"closing_speed_ms\":" << s.threat_closing_speed_ms << ","
               << "\"floor_idx\":" << s.threat_floor_idx
               << "}"
               << "}";
        }
        ss << "]}";
        return ss.str();
    }

    CollisionConfig config_;
    CellTrack grid_[12][36];
    uint32_t consecutive_valid_frames_{0};
};
