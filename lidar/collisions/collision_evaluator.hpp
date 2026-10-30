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

// Matematické jádro pro vyhodnocení kolizních kurzů v mřížce 12 pater × 72 sektorů (po 5°)
class CollisionEvaluator {
public:
    static constexpr size_t kMaxFloors  = 12;
    static constexpr size_t kMaxSectors = 72;

    struct EvaluationResult {
        double timestamp = 0.0;
        double heading_start = 0.0;
        double heading_end = 0.0;
        double heading_avg = 0.0;
        size_t points_count = 0;
        double duration_ms = 0.0;

        float min_clearance_cm = 2000.0f;
        float min_ttc_s = 999.0f;
        float min_ttc_angle_deg = -1.0f;

        // Výstupní pole v souřadnicích těla robota (72 sektorů po 5°, index 0 = 0° vpřed, rotace clockwise)
        // Přední polokoule: indexy 0..17 (0°..90°) a 54..71 (270°..360°)
        // Zadní slepá polokoule: indexy 18..53 (90°..270°) mají distances_cm = -1, delta_d_cms = 0
        std::vector<int> distances_cm;
        std::vector<int> delta_d_cms;

        std::string json_payload;
    };

    explicit CollisionEvaluator(CollisionConfig config = CollisionConfig{})
        : config_(config)
    {
        reset();
    }

    void reset() {
        for (size_t f = 0; f < kMaxFloors; ++f) {
            for (size_t s = 0; s < kMaxSectors; ++s) {
                grid_[f][s] = CellTrack{};
                grid_[f][s].d_est = config_.max_range_cm;
                grid_[f][s].theta_est = (static_cast<float>(s) + 0.5f) * config_.sector_angle_deg;
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

        // 1. Kontrola počtu bodů (min config_.min_points_per_cycle bodů za cyklus)
        if (samples.size() < config_.min_points_per_cycle) {
            consecutive_valid_frames_ = 0;
            return false;
        }

        // 2. Akumulace měření v globální (North-Up CW) mřížce 12 pater × 72 sektorů
        float meas_d[kMaxFloors][kMaxSectors];
        float meas_theta[kMaxFloors][kMaxSectors];
        uint32_t meas_count[kMaxFloors][kMaxSectors];

        for (size_t f = 0; f < config_.num_floors; ++f) {
            for (size_t s = 0; s < config_.num_sectors; ++s) {
                meas_d[f][s]     = config_.max_range_cm;
                meas_theta[f][s] = (static_cast<float>(s) + 0.5f) * config_.sector_angle_deg;
                meas_count[f][s] = 0;
            }
        }

        const float rad2deg = 180.0f / static_cast<float>(M_PI);
        const float deg2rad = static_cast<float>(M_PI) / 180.0f;

        for (const auto &p : samples) {
            // Kontrola geometrického obrysu šasi robota v rámci LiDARu (v metrech)
            float px_m = p.x * 0.01f;
            float py_m = p.y * 0.01f;
            if (px_m >= config_.mask_x_min_m && px_m <= config_.mask_front_x_m &&
                py_m >= config_.mask_y_min_m && py_m <= config_.mask_y_max_m) {
                continue;
            }

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

            // Lokální azimut v rovině robota (clockwise: 0° = +X vpřed, 90° = -Y vpravo, 270° = +Y vlevo)
            float local_cw_deg = std::atan2(-yr, xr) * rad2deg;
            if (local_cw_deg < 0.0f) local_cw_deg += 360.0f;

            // Globální azimut stabilizovaný na sever (clockwise: 0° = sever, 90° = východ)
            float global_cw_deg = wrap360(local_cw_deg + static_cast<float>(heading_avg));

            size_t s = static_cast<size_t>(global_cw_deg / config_.sector_angle_deg) % config_.num_sectors;

            if (d < meas_d[f][s]) {
                meas_d[f][s]     = d;
                meas_theta[f][s] = global_cw_deg;
            }
            meas_count[f][s]++;
        }

        // 3. Dynamická filtrace Alfa-Beta (vzdálenost + azimut) a vertikální provázanost pater
        for (size_t s = 0; s < config_.num_sectors; ++s) {
            for (size_t f = 0; f < config_.num_floors; ++f) {
                auto &track = grid_[f][s];
                bool has_meas = (meas_count[f][s] >= config_.min_points_per_cell);
                float dm      = meas_d[f][s];
                float thetam  = meas_theta[f][s];

                float d_pred = track.active ? (track.d_est - track.v_est * config_.dt_sec) : config_.max_range_cm;

                // Vertikální handover: pokud v tomto patře měření chybí, zkontroluj sousedy (míč / nerovnosti)
                if (!has_meas && track.active) {
                    size_t f_down = (f > 0) ? (f - 1) : f;
                    size_t f_up   = (f + 1 < config_.num_floors) ? (f + 1) : f;

                    if (f_down != f && meas_count[f_down][s] >= config_.min_points_per_cell) {
                        float cand_d = meas_d[f_down][s];
                        if (std::abs(cand_d - d_pred) < config_.max_track_jump_cm) {
                            dm       = cand_d;
                            thetam   = meas_theta[f_down][s];
                            has_meas = true;
                        }
                    } else if (f_up != f && meas_count[f_up][s] >= config_.min_points_per_cell) {
                        float cand_d = meas_d[f_up][s];
                        if (std::abs(cand_d - d_pred) < config_.max_track_jump_cm) {
                            dm       = cand_d;
                            thetam   = meas_theta[f_up][s];
                            has_meas = true;
                        }
                    }
                }

                if (has_meas) {
                    if (!track.active || std::abs(dm - d_pred) > config_.max_track_jump_cm) {
                        // Nový track
                        track.d_est         = dm;
                        track.v_est         = 0.0f;
                        track.theta_est     = thetam;
                        track.omega_est     = 0.0f;
                        track.hits          = 1;
                        track.missed_frames = 0;
                        track.active        = true;
                    } else {
                        // Inkrementální aktualizace alfa-beta pro vzdálenost
                        float residual = d_pred - dm;
                        track.d_est = std::clamp(d_pred - config_.alpha * residual,
                                                 config_.min_valid_dist_cm, config_.max_range_cm);
                        track.v_est = track.v_est + (config_.beta / config_.dt_sec) * residual;

                        // Inkrementální aktualizace alfa-beta pro globální úhel (sledování míjení / CBDR)
                        float theta_pred = wrap360(track.theta_est + track.omega_est * config_.dt_sec);
                        float theta_err  = wrap180(thetam - theta_pred);
                        track.theta_est  = wrap360(theta_pred + config_.alpha_theta * theta_err);
                        track.omega_est  = track.omega_est + (config_.beta_theta / config_.dt_sec) * theta_err;

                        track.hits++;
                        track.missed_frames = 0;
                        track.active = true;
                    }
                } else {
                    if (track.active) {
                        track.missed_frames++;
                        if (track.missed_frames > 2) {
                            track.active    = false;
                            track.d_est     = config_.max_range_cm;
                            track.v_est     = 0.0f;
                            track.omega_est = 0.0f;
                            track.hits      = 0;
                        } else {
                            track.d_est     = std::clamp(d_pred, config_.min_valid_dist_cm, config_.max_range_cm);
                            track.theta_est = wrap360(track.theta_est + track.omega_est * config_.dt_sec);
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

        // 4. Vyhodnocení kolizního kurzu (CBDR) v přední polokouli robota (-90° až +90°, tj. 270°..90° CW)
        float global_min_ttc       = 999.0f;
        float global_min_ttc_angle = -1.0f;

        for (size_t f = 0; f < config_.num_floors; ++f) {
            for (size_t s = 0; s < config_.num_sectors; ++s) {
                const auto &track = grid_[f][s];
                // Uvažujeme jen potvrzené aktivní objekty, které se přibližují (v >= min_ttc_speed_cms)
                if (!track.active ||
                    track.hits < config_.min_hits_for_ttc ||
                    track.d_est >= config_.max_range_cm ||
                    track.v_est < config_.min_ttc_speed_cms) {
                    continue;
                }

                // Pro nejnižší patro (Floor 0: 12-22 cm) vyžadujeme potvrzení z vyššího patra nebo dostatečný počet bodů (odfiltrování trávy)
                if (f == 0) {
                    bool has_upper = (config_.num_floors > 1 && grid_[1][s].active);
                    if (!has_upper && meas_count[0][s] < config_.min_points_ground_floor) {
                        continue;
                    }
                }

                // Směr hrozby v souřadnicích těla robota (0° = vpřed, CW)
                float body_deg = wrap360(track.theta_est - static_cast<float>(heading_avg));

                // Omezení na přední polokouli (-90° až +90°, tj. [0°..90°] a [270°..360°))
                if (body_deg > 90.0f && body_deg < 270.0f) {
                    continue;
                }

                // Princip CBDR (Constant Bearing, Decreasing Range):
                // Ze změny azimutu omega_est spočítáme tečnou rychlost v_t a nejbližší vzdálenost minutí (CPA).
                float v_r_cms = track.v_est; // > 0 (cm/s)
                float v_t_cms = track.d_est * std::abs(track.omega_est) * deg2rad;
                float v_rel   = std::sqrt(v_r_cms * v_r_cms + v_t_cms * v_t_cms);
                float d_cpa   = (v_rel > 1e-3f) ? (track.d_est * (v_t_cms / v_rel)) : 0.0f;

                // Pokud trajektorie míjí válec těla robota (R = 37.5 cm), nejde o kolizní kurz (např. strom vedle cesty)
                if (d_cpa > config_.robot_radius_cm) {
                    continue;
                }

                float ttc = track.d_est / v_r_cms; // [s] (cm / (cm/s))
                if (ttc < global_min_ttc) {
                    global_min_ttc       = ttc;
                    global_min_ttc_angle = body_deg;
                }
            }
        }

        // 5. Syntéza výstupních polí v těle robota (72 sektorů po 5°, 0 = vpřed, rotace clockwise)
        // Nejprve v globální mřížce najdeme aktuální d_min[72] a predikované d_pred_1s[72] (včetně bočního posuvu)
        float glob_d_now[kMaxSectors];
        float glob_d_pred1s[kMaxSectors];

        for (size_t s = 0; s < config_.num_sectors; ++s) {
            glob_d_now[s]    = config_.max_range_cm;
            glob_d_pred1s[s] = config_.max_range_cm;
        }

        for (size_t s = 0; s < config_.num_sectors; ++s) {
            for (size_t f = 0; f < config_.num_floors; ++f) {
                const auto &track = grid_[f][s];
                if (!track.active) continue;

                // Pro nejnižší patro (Floor 0: 12-22 cm) odfiltrujeme ojedinělá stébla trávy
                if (f == 0) {
                    bool has_upper = (config_.num_floors > 1 && grid_[1][s].active);
                    if (!has_upper && meas_count[0][s] < config_.min_points_ground_floor) {
                        continue;
                    }
                }

                if (track.d_est < glob_d_now[s]) {
                    glob_d_now[s] = track.d_est;
                }

                // Predikce polohy za 1.0 s (radiálně i úhlově)
                float d_1s = std::clamp(track.d_est - track.v_est * 1.0f,
                                        config_.min_valid_dist_cm, config_.max_range_cm);
                if (d_1s < glob_d_pred1s[s]) {
                    glob_d_pred1s[s] = d_1s;
                }

                if (track.hits >= 2 && std::abs(track.omega_est) > 1.0f) {
                    float theta_1s = wrap360(track.theta_est + track.omega_est * 1.0f);
                    size_t s_1s = static_cast<size_t>(theta_1s / config_.sector_angle_deg) % config_.num_sectors;
                    if (d_1s < glob_d_pred1s[s_1s]) {
                        glob_d_pred1s[s_1s] = d_1s;
                    }
                }
            }
        }

        out_result.distances_cm.assign(config_.num_sectors, -1);
        out_result.delta_d_cms.assign(config_.num_sectors, 0);
        float global_min_clearance = config_.max_range_cm;

        // Přemapování z globální soustavy (North-Up CW) do těla robota (Body-Up CW)
        // Sektor b = 0 odpovídá [0°, 5°) vpřed, b = 17 je [85°, 90°) vpravo,
        // b = 18..53 je zadní slepá polokoule [90°, 270°) -> hodnota -1,
        // b = 54..71 je levá přední čtvrtina [270°, 360°).
        for (size_t b = 0; b < config_.num_sectors; ++b) {
            if (b >= 18 && b < 54) {
                out_result.distances_cm[b] = -1;
                out_result.delta_d_cms[b]  = 0;
                continue;
            }

            float body_center_deg = (static_cast<float>(b) + 0.5f) * config_.sector_angle_deg;
            float glob_deg        = wrap360(body_center_deg + static_cast<float>(heading_avg));
            size_t glob_s         = static_cast<size_t>(glob_deg / config_.sector_angle_deg) % config_.num_sectors;

            float d_now  = glob_d_now[glob_s];
            float d_pred = glob_d_pred1s[glob_s];

            int d_now_int  = static_cast<int>(std::lround(d_now));
            int d_pred_int = static_cast<int>(std::lround(d_pred));

            out_result.distances_cm[b] = d_now_int;
            out_result.delta_d_cms[b]  = d_pred_int - d_now_int;

            if (d_now < global_min_clearance) {
                global_min_clearance = d_now;
            }
        }

        auto end_time = std::chrono::steady_clock::now();
        double duration_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

        out_result.timestamp         = timestamp;
        out_result.heading_start     = heading_start;
        out_result.heading_end       = heading_end;
        out_result.heading_avg       = heading_avg;
        out_result.points_count      = samples.size();
        out_result.duration_ms       = duration_ms;
        out_result.min_clearance_cm  = global_min_clearance;
        out_result.min_ttc_s         = global_min_ttc;
        out_result.min_ttc_angle_deg = global_min_ttc_angle;

        // 6. Generování výstupního JSONu
        out_result.json_payload = buildJsonPayload(out_result);

        return true;
    }

private:
    struct CellTrack {
        float d_est = 2000.0f;      // Vyhlazená vzdálenost [cm]
        float v_est = 0.0f;         // Rychlost přibližování [cm/s] (kladná = přibližování)
        float theta_est = 0.0f;     // Vyhlazený globální azimut [deg CW]
        float omega_est = 0.0f;     // Úhlová rychlost posuvu [deg/s CW]
        uint32_t hits = 0;          // Počet navazujících detekcí
        uint32_t missed_frames = 0; // Počet po sobě chybějících měření
        bool active = false;
    };

    static float wrap360(float deg) {
        float r = std::fmod(deg, 360.0f);
        if (r < 0.0f) r += 360.0f;
        return r;
    }

    static float wrap180(float deg) {
        float r = std::fmod(deg + 180.0f, 360.0f);
        if (r < 0.0f) r += 360.0f;
        return r - 180.0f;
    }

    std::string buildJsonPayload(const EvaluationResult &res) const {
        std::ostringstream ss;
        ss << "{"
           << "\"timestamp\":" << std::fixed << std::setprecision(3) << res.timestamp << ","
           << "\"heading\":" << std::setprecision(1) << res.heading_avg << ","
           << "\"min_ttc_s\":" << std::setprecision(2) << res.min_ttc_s << ","
           << "\"min_ttc_angle_deg\":" << std::setprecision(1) << res.min_ttc_angle_deg << ","
           << "\"distances_cm\":[";

        for (size_t i = 0; i < res.distances_cm.size(); ++i) {
            if (i > 0) ss << ",";
            ss << res.distances_cm[i];
        }

        ss << "],\"delta_d_cms\":[";
        for (size_t i = 0; i < res.delta_d_cms.size(); ++i) {
            if (i > 0) ss << ",";
            ss << res.delta_d_cms[i];
        }

        ss << "]}";
        return ss.str();
    }

    CollisionConfig config_;
    CellTrack grid_[kMaxFloors][kMaxSectors];
    uint32_t consecutive_valid_frames_{0};
};
