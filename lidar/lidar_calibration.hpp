#pragma once

#include <Eigen/Dense>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <limits>
#include <cmath>
#include <vector>
#include <chrono>
#include <thread>
#include <algorithm>

#include "unitree_lidar_sdk.h"
#include "lidar_reader.hpp"
#include "ply_logger.hpp"

struct LidarCalibration
{
    // Transformace z rámce LiDARu L do kalibrovaného rámce C (v metrech)
    Eigen::Matrix4f T_CL = Eigen::Matrix4f::Identity();

    // Maska robota v rámce C (v metrech)
    float mask_front_x = 0.0f;  // přední hrana (x_max + 5 cm)
    float mask_x_min   = 0.0f;  // zadní hrana (typicky -0.70)
    float mask_y_min   = 0.0f;
    float mask_y_max   = 0.0f;
};

inline bool saveCalibration(const std::string &path, const LidarCalibration &c)
{
    std::ofstream ofs(path);
    if (!ofs) {
        std::cerr << "[CALIBRATION] cannot open file for writing: " << path << std::endl;
        return false;
    }

    ofs << "CALIBRATION_V1\n";

    ofs << "T "
        << c.T_CL(0,0) << " " << c.T_CL(0,1) << " " << c.T_CL(0,2) << " " << c.T_CL(0,3) << "\n";
    ofs << "T "
        << c.T_CL(1,0) << " " << c.T_CL(1,1) << " " << c.T_CL(1,2) << " " << c.T_CL(1,3) << "\n";
    ofs << "T "
        << c.T_CL(2,0) << " " << c.T_CL(2,1) << " " << c.T_CL(2,2) << " " << c.T_CL(2,3) << "\n";
    ofs << "T "
        << c.T_CL(3,0) << " " << c.T_CL(3,1) << " " << c.T_CL(3,2) << " " << c.T_CL(3,3) << "\n";

    ofs << "MASK_FRONT_X " << c.mask_front_x << "\n";
    ofs << "MASK_X_MIN   " << c.mask_x_min   << "\n";
    ofs << "MASK_Y_MIN   " << c.mask_y_min   << "\n";
    ofs << "MASK_Y_MAX   " << c.mask_y_max   << "\n";

    if (!ofs.good()) {
        std::cerr << "[CALIBRATION] error while writing file: " << path << std::endl;
        return false;
    }

    std::cout << "[CALIBRATION] saved to " << path << std::endl;
    return true;
}

inline bool loadCalibration(const std::string &path, LidarCalibration &out)
{
    std::ifstream ifs(path);
    if (!ifs) {
        std::cerr << "[CALIBRATION] cannot open file for reading: " << path << std::endl;
        return false;
    }

    std::string line;
    if (!std::getline(ifs, line)) {
        std::cerr << "[CALIBRATION] empty file: " << path << std::endl;
        return false;
    }
    if (line.find("CALIBRATION_V1") == std::string::npos) {
        std::cerr << "[CALIBRATION] unsupported or missing header in " << path << std::endl;
        return false;
    }

    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    float mask_front_x = 0.0f;
    float mask_x_min   = 0.0f;
    float mask_y_min   = 0.0f;
    float mask_y_max   = 0.0f;

    bool have_row[4] = {false,false,false,false};
    bool have_front  = false;
    bool have_xmin   = false;
    bool have_ymin   = false;
    bool have_ymax   = false;

    int current_row = 0;

    while (std::getline(ifs, line)) {
        if (line.empty()) continue;
        if (line[0] == '#') continue;

        std::istringstream iss(line);
        std::string key;
        iss >> key;
        if (!iss) continue;

        if (key == "T") {
            if (current_row >= 4) {
                std::cerr << "[CALIBRATION] too many T rows in " << path << std::endl;
                return false;
            }
            float a,b,c,d;
            if (!(iss >> a >> b >> c >> d)) {
                std::cerr << "[CALIBRATION] malformed T row in " << path << std::endl;
                return false;
            }
            T(current_row,0) = a;
            T(current_row,1) = b;
            T(current_row,2) = c;
            T(current_row,3) = d;
            have_row[current_row] = true;
            ++current_row;
        } else if (key == "MASK_FRONT_X") {
            if (!(iss >> mask_front_x)) {
                std::cerr << "[CALIBRATION] malformed MASK_FRONT_X in " << path << std::endl;
                return false;
            }
            have_front = true;
        } else if (key == "MASK_X_MIN") {
            if (!(iss >> mask_x_min)) {
                std::cerr << "[CALIBRATION] malformed MASK_X_MIN in " << path << std::endl;
                return false;
            }
            have_xmin = true;
        } else if (key == "MASK_Y_MIN") {
            if (!(iss >> mask_y_min)) {
                std::cerr << "[CALIBRATION] malformed MASK_Y_MIN in " << path << std::endl;
                return false;
            }
            have_ymin = true;
        } else if (key == "MASK_Y_MAX") {
            if (!(iss >> mask_y_max)) {
                std::cerr << "[CALIBRATION] malformed MASK_Y_MAX in " << path << std::endl;
                return false;
            }
            have_ymax = true;
        }
    }

    for (int i=0; i<4; ++i) {
        if (!have_row[i]) {
            std::cerr << "[CALIBRATION] missing T row " << i << " in " << path << std::endl;
            return false;
        }
    }
    if (!have_front || !have_xmin || !have_ymin || !have_ymax) {
        std::cerr << "[CALIBRATION] missing mask parameters in " << path << std::endl;
        return false;
    }

    if (!std::isfinite(T(3,3)) || std::fabs(T(3,3) - 1.0f) > 1e-3f) {
        std::cerr << "[CALIBRATION] warning: suspicious T_CL[3,3] in " << path << std::endl;
    }

    out.T_CL         = T;
    out.mask_front_x = mask_front_x;
    out.mask_x_min   = mask_x_min;
    out.mask_y_min   = mask_y_min;
    out.mask_y_max   = mask_y_max;

    std::cout << "[CALIBRATION] loaded from " << path << std::endl;
    return true;
}

// Skeleton pro kalibrační pipeline (sběr IMU + cloudů, výpočet T_CL a masky).
class LidarCalibrator
{
public:
    LidarCalibrator()
        : ply_logger_("/data/robot/lidar/calibration")
    {
    }
    // duration: délka sběru dat; výsledek ukládá do 'out' a optionally uloží do file.
    bool run(LidarReader &reader,
             LidarCalibration &out,
             const std::string &file = "calibration.dat",
             std::chrono::seconds duration = std::chrono::seconds(10))
    {
        using namespace std::chrono;

        // 1) Spustit rotaci a sbírat data
        try {
            reader.startRotation();
        } catch (const std::exception &e) {
            std::cerr << "[CALIBRATE] exception in startRotation: " << e.what() << std::endl;
            return false;
        } catch (...) {
            std::cerr << "[CALIBRATE] unknown exception in startRotation" << std::endl;
            return false;
        }

        // Wait for data
        auto t_start = steady_clock::now();
        auto t_end   = t_start + std::chrono::seconds(120); 
        int has_imu = 0;
        int has_cloud = 0;
        while (steady_clock::now() < t_end && (has_imu < 100 || has_cloud < 100)) {
            int type = reader.runParse();
            if (type == LIDAR_POINT_DATA_PACKET_TYPE) {
                has_cloud++;
            } else if ( type == LIDAR_IMU_DATA_PACKET_TYPE) {
                has_imu++;
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }

        std::cout << "[CALIBRATE] data stream check: "
                  << (has_imu < 100 ? "IMU MISSING" : "IMU OK") << " (" << has_imu << ") "
                  << (has_cloud < 100 ? "CLOUD MISSING" : "CLOUD OK") << " (" << has_cloud << ")" << std::endl;
        
        if (has_imu < 100 || has_cloud < 100) return false;

        std::vector<Eigen::Vector3f> acc_samples;
        acc_samples.reserve(5000);
        std::vector<unilidar_sdk2::PointCloudUnitree> clouds;
        clouds.reserve(64);
        std::vector<Eigen::Quaternionf> q_samples;
        q_samples.reserve(5000);

        t_start = steady_clock::now();
        t_end   = t_start + duration;

        // Collect data
        while (steady_clock::now() < t_end) {
            int type = reader.runParse();

            // --- IMU packet: acc + quaternion ---
            if (type == LIDAR_IMU_DATA_PACKET_TYPE) {
                unilidar_sdk2::LidarImuData imu{};
                if (!reader.getImuData(imu)) continue;

                Eigen::Vector3f gyro(
                    imu.angular_velocity[0],
                    imu.angular_velocity[1],
                    imu.angular_velocity[2]
                );
                if (!gyro.allFinite()) continue;

                // ~stojím (filtruj velké gyro)
                if (gyro.norm() > 0.05f) continue;

                Eigen::Vector3f acc(
                    imu.linear_acceleration[0],
                    imu.linear_acceleration[1],
                    imu.linear_acceleration[2]
                );
                if (!acc.allFinite()) continue;
                acc_samples.push_back(acc);

                // SDK typicky posílá quaternion jako (x,y,z,w) -> do Eigen (w,x,y,z)
                const float qx = imu.quaternion[0];
                const float qy = imu.quaternion[1];
                const float qz = imu.quaternion[2];
                const float qw = imu.quaternion[3];

                Eigen::Quaternionf q(qw, qx, qy, qz);
                if (!std::isfinite(q.w()) || !std::isfinite(q.x()) ||
                    !std::isfinite(q.y()) || !std::isfinite(q.z())) {
                    continue;
                }
                if (q.squaredNorm() < 1e-12f) continue;
                q.normalize();

                q_samples.push_back(q);
            } else if (type == LIDAR_POINT_DATA_PACKET_TYPE) {
                unilidar_sdk2::PointCloudUnitree cloud;
                if (reader.getPointCloud(cloud)) {
                    clouds.push_back(std::move(cloud));
                }
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }

        // Zastavit rotaci
        try {
            reader.stopRotation();
        } catch (const std::exception &e) {
            std::cerr << "[CALIBRATE] exception in stopRotation: " << e.what() << std::endl;
        } catch (...) {
            std::cerr << "[CALIBRATE] unknown exception in stopRotation" << std::endl;
        }
        reader.clearBuffer();

        // 2) Kontroly dat
        if (acc_samples.empty()) {
            std::cerr << "[CALIBRATE] no IMU samples collected" << std::endl;
            return false;
        }
        if (clouds.empty()) {
            std::cerr << "[CALIBRATE] no point clouds collected" << std::endl;
            return false;
        }
        if (q_samples.empty()) {
            std::cerr << "[CALIBRATE] no IMU quaternion samples collected" << std::endl;
            return false;
        }

        std::cout << "[CALIBRATE] collected "
                  << acc_samples.size() << " IMU samples and "
                  << q_samples.size() << " quaternions, and "
                  << clouds.size() << " point clouds." << std::endl;


        // Ručně nastavené úhly z defaultTransformMatrix (bez scale):
        // Tx = Ms * Ry(th_y) * Rz(th_z)  => R_CL = Ry * Rz
        Eigen::Matrix3f R_CL;
        {
            const float deg  = static_cast<float>(M_PI) / 180.0f;
            const float th_z = -25.5f * deg;
            const float th_y = -47.5f * deg;

            Eigen::Matrix3f Rz;
            Rz <<  std::cos(th_z),  std::sin(th_z), 0.0f,
                -std::sin(th_z),  std::cos(th_z), 0.0f,
                                0.0f,           0.0f, 1.0f;

            Eigen::Matrix3f Ry;
            Ry <<  std::cos(th_y), 0.0f, -std::sin(th_y),
                                0.0f, 1.0f,           0.0f,
                std::sin(th_y), 0.0f,  std::cos(th_y);

            Eigen::Matrix3f R_manual = Ry * Rz;   // p_C = R_manual * p_L

            // rotace o 180° kolem osy X (flip Y a Z)
            Eigen::Matrix3f Rx_pi;
            Rx_pi << 1.0f,  0.0f,  0.0f,
                    0.0f, -1.0f,  0.0f,
                    0.0f,  0.0f, -1.0f;

            R_CL = Rx_pi * R_manual;  // p_C = R_CL * p_L
        }

/*        
        // ---- IMU debug statistiky (robust): acc_samples + q_samples (per-sample g, jitter, percentiles) ----
        // REQUIRE: R_CL must be in scope if you want the "expected gravity" part (can comment that section out).
        {
            auto clampf = [](float v, float lo, float hi) {
                return std::max(lo, std::min(hi, v));
            };

            auto angle_deg_between = [&](const Eigen::Vector3f& a, const Eigen::Vector3f& b) -> float {
                float na = a.norm(), nb = b.norm();
                if (!std::isfinite(na) || !std::isfinite(nb) || na < 1e-9f || nb < 1e-9f)
                    return std::numeric_limits<float>::quiet_NaN();
                float c = a.dot(b) / (na * nb);
                c = clampf(c, -1.f, 1.f);
                return std::acos(c) * 180.f / static_cast<float>(M_PI);
            };

            auto percentile_sorted = [&](const std::vector<float>& v_sorted, float p01) -> float {
                if (v_sorted.empty()) return std::numeric_limits<float>::quiet_NaN();
                p01 = clampf(p01, 0.f, 1.f);
                float idx = p01 * float(v_sorted.size() - 1);
                std::size_t i0 = (std::size_t)std::floor(idx);
                std::size_t i1 = std::min(i0 + 1, v_sorted.size() - 1);
                float t = idx - float(i0);
                return (1.f - t) * v_sorted[i0] + t * v_sorted[i1];
            };

            const std::size_t N = std::min(acc_samples.size(), q_samples.size());
            if (N < 10) {
                std::cout << "[IMU-STAT2] Not enough samples: N=" << N << "\n";
            } else {
                // --- ACC stats ---
                Eigen::Vector3f acc_sum = Eigen::Vector3f::Zero();
                Eigen::Vector3f acc_min( std::numeric_limits<float>::infinity(),
                                        std::numeric_limits<float>::infinity(),
                                        std::numeric_limits<float>::infinity());
                Eigen::Vector3f acc_max(-std::numeric_limits<float>::infinity(),
                                        -std::numeric_limits<float>::infinity(),
                                        -std::numeric_limits<float>::infinity());

                float acc_norm_sum = 0.f;
                float acc_norm_sq_sum = 0.f;

                // Direction mean (normalized acc)
                Eigen::Vector3f acc_dir_sum = Eigen::Vector3f::Zero();

                std::size_t n_acc_ok = 0;
                for (std::size_t i = 0; i < N; ++i) {
                    const auto& a = acc_samples[i];
                    if (!a.allFinite()) continue;
                    float an = a.norm();
                    if (!std::isfinite(an) || an < 1e-6f) continue;

                    acc_sum += a;
                    acc_min = acc_min.cwiseMin(a);
                    acc_max = acc_max.cwiseMax(a);

                    acc_norm_sum += an;
                    acc_norm_sq_sum += an * an;

                    acc_dir_sum += (a / an);
                    ++n_acc_ok;
                }

                Eigen::Vector3f acc_mean = Eigen::Vector3f::Zero();
                Eigen::Vector3f acc_std  = Eigen::Vector3f::Zero();
                float acc_norm_mean = std::numeric_limits<float>::quiet_NaN();
                float acc_norm_std  = std::numeric_limits<float>::quiet_NaN();
                Eigen::Vector3f acc_dir_mean = Eigen::Vector3f::Zero();

                if (n_acc_ok > 0) {
                    float invN = 1.f / float(n_acc_ok);
                    acc_mean = acc_sum * invN;

                    // component std
                    Eigen::Vector3f var = Eigen::Vector3f::Zero();
                    for (std::size_t i = 0; i < N; ++i) {
                        const auto& a = acc_samples[i];
                        if (!a.allFinite()) continue;
                        Eigen::Vector3f d = a - acc_mean;
                        var += d.cwiseProduct(d);
                    }
                    var *= invN;
                    acc_std = var.cwiseSqrt();

                    // norm std
                    acc_norm_mean = acc_norm_sum * invN;
                    float acc_norm_var = acc_norm_sq_sum * invN - acc_norm_mean * acc_norm_mean;
                    acc_norm_std = std::sqrt(std::max(0.f, acc_norm_var));

                    // direction mean
                    acc_dir_mean = acc_dir_sum * invN;
                    if (acc_dir_mean.norm() > 1e-6f) acc_dir_mean.normalize();
                }

                // direction spread (angles to mean direction)
                std::vector<float> acc_dir_angles;
                acc_dir_angles.reserve(n_acc_ok);
                for (std::size_t i = 0; i < N; ++i) {
                    const auto& a = acc_samples[i];
                    if (!a.allFinite()) continue;
                    float an = a.norm();
                    if (!std::isfinite(an) || an < 1e-6f) continue;
                    float ang = angle_deg_between(a, acc_dir_mean);
                    if (std::isfinite(ang)) acc_dir_angles.push_back(ang);
                }
                std::sort(acc_dir_angles.begin(), acc_dir_angles.end());

                // --- Quaternion-derived gravity stats (per-sample; no quaternion-mean artifacts) ---
                const Eigen::Vector3f g_W(0.f, 0.f, -9.81f);

                Eigen::Vector3f gA_sum = Eigen::Vector3f::Zero(), gA_sq = Eigen::Vector3f::Zero();
                Eigen::Vector3f gB_sum = Eigen::Vector3f::Zero(), gB_sq = Eigen::Vector3f::Zero();
                std::vector<float> ang_acc_A, ang_acc_B;
                ang_acc_A.reserve(N);
                ang_acc_B.reserve(N);

                // quaternion delta (stability / drift indicator)
                std::vector<float> dq_angles;
                dq_angles.reserve(N - 1);

                std::size_t n_q_ok = 0;

                Eigen::Quaternionf q_prev = q_samples[0];
                if (q_prev.squaredNorm() > 1e-12f && q_prev.coeffs().allFinite()) q_prev.normalize();

                bool have_first = false;
                Eigen::Vector3f gB_first = Eigen::Vector3f::Zero(), gB_last = Eigen::Vector3f::Zero();

                for (std::size_t i = 0; i < N; ++i) {
                    Eigen::Quaternionf q = q_samples[i];
                    if (!q.coeffs().allFinite()) continue;
                    if (q.squaredNorm() < 1e-12f) continue;
                    q.normalize();

                    // keep hemisphere consistent w.r.t prev for delta computation
                    if (i > 0 && q_prev.coeffs().dot(q.coeffs()) < 0.f) q.coeffs() *= -1.f;

                    Eigen::Matrix3f R = q.toRotationMatrix();

                    // A) R is body->world => g_B = R^T * g_W
                    Eigen::Vector3f g_B_A = R.transpose() * g_W;
                    // B) R is world->body => g_B = R * g_W
                    Eigen::Vector3f g_B_B = R * g_W;

                    gA_sum += g_B_A; gA_sq += g_B_A.cwiseProduct(g_B_A);
                    gB_sum += g_B_B; gB_sq += g_B_B.cwiseProduct(g_B_B);

                    // angles acc vs -g (directional consistency)
                    const auto& a = acc_samples[i];
                    if (a.allFinite() && a.norm() > 1e-6f) {
                        float angA = angle_deg_between(a, -g_B_A);
                        float angB = angle_deg_between(a, -g_B_B);
                        if (std::isfinite(angA)) ang_acc_A.push_back(angA);
                        if (std::isfinite(angB)) ang_acc_B.push_back(angB);
                    }

                    // first/last g_B_B direction (for drift)
                    if (!have_first) {
                        gB_first = g_B_B;
                        have_first = true;
                    }
                    gB_last = g_B_B;

                    // delta quaternion angle (between consecutive samples)
                    if (i > 0) {
                        Eigen::Quaternionf dq = q_prev.conjugate() * q; // rotation from prev to curr
                        if (dq.w() < 0.f) dq.coeffs() *= -1.f;
                        float w = clampf(dq.w(), -1.f, 1.f);
                        float ang = 2.f * std::acos(w) * 180.f / static_cast<float>(M_PI);
                        if (std::isfinite(ang)) dq_angles.push_back(ang);
                    }

                    q_prev = q;
                    ++n_q_ok;
                }

                Eigen::Vector3f gA_mean = Eigen::Vector3f::Zero(), gA_std = Eigen::Vector3f::Zero();
                Eigen::Vector3f gB_mean = Eigen::Vector3f::Zero(), gB_std = Eigen::Vector3f::Zero();
                Eigen::Vector3f gA_dir  = Eigen::Vector3f::Zero(), gB_dir  = Eigen::Vector3f::Zero();

                if (n_q_ok > 0) {
                    float invQ = 1.f / float(n_q_ok);
                    gA_mean = gA_sum * invQ;
                    gB_mean = gB_sum * invQ;

                    Eigen::Vector3f gA_var = gA_sq * invQ - gA_mean.cwiseProduct(gA_mean);
                    Eigen::Vector3f gB_var = gB_sq * invQ - gB_mean.cwiseProduct(gB_mean);
                    gA_std = gA_var.cwiseMax(0.f).cwiseSqrt();
                    gB_std = gB_var.cwiseMax(0.f).cwiseSqrt();

                    gA_dir = gA_mean; if (gA_dir.norm() > 1e-6f) gA_dir.normalize();
                    gB_dir = gB_mean; if (gB_dir.norm() > 1e-6f) gB_dir.normalize();
                }

                std::sort(ang_acc_A.begin(), ang_acc_A.end());
                std::sort(ang_acc_B.begin(), ang_acc_B.end());
                std::sort(dq_angles.begin(), dq_angles.end());

                auto mean_std = [&](const std::vector<float>& v) -> std::pair<float,float> {
                    if (v.empty()) return {std::numeric_limits<float>::quiet_NaN(),
                                        std::numeric_limits<float>::quiet_NaN()};
                    double s = 0.0, s2 = 0.0;
                    for (float x : v) { s += x; s2 += double(x)*double(x); }
                    double n = double(v.size());
                    double m = s / n;
                    double var = s2 / n - m*m;
                    if (var < 0) var = 0;
                    return {float(m), float(std::sqrt(var))};
                };

                auto [angA_mean, angA_std] = mean_std(ang_acc_A);
                auto [angB_mean, angB_std] = mean_std(ang_acc_B);
                auto [dq_mean,  dq_std ]   = mean_std(dq_angles);

                float ang_gB_drift = angle_deg_between(gB_first, gB_last); // should be small if stable
                float detR = q_prev.toRotationMatrix().determinant();      // just one sample check
                float ortho_err = (q_prev.toRotationMatrix() * q_prev.toRotationMatrix().transpose()
                                - Eigen::Matrix3f::Identity()).norm();

                std::cout << "[IMU-STAT2] N=" << N
                        << " acc_ok=" << n_acc_ok
                        << " q_ok=" << n_q_ok
                        << " angA_count=" << ang_acc_A.size()
                        << " angB_count=" << ang_acc_B.size()
                        << "\n";

                std::cout << "  acc_mean = [" << acc_mean.transpose() << "], |acc_mean|=" << acc_mean.norm() << "\n";
                std::cout << "  acc_std  = [" << acc_std.transpose()  << "]\n";
                std::cout << "  acc_min  = [" << acc_min.transpose()  << "]\n";
                std::cout << "  acc_max  = [" << acc_max.transpose()  << "]\n";
                std::cout << "  |acc| mean=" << acc_norm_mean << " std=" << acc_norm_std << "\n";
                std::cout << "  acc_dir_mean = [" << acc_dir_mean.transpose() << "]"
                        << " ang_to_dir: p50=" << percentile_sorted(acc_dir_angles, 0.50f)
                        << " p95=" << percentile_sorted(acc_dir_angles, 0.95f)
                        << " max=" << (acc_dir_angles.empty() ? NAN : acc_dir_angles.back()) << "\n";

                std::cout << "  g_B_A_mean (R^T*gW) = [" << gA_mean.transpose() << "], |mean|=" << gA_mean.norm()
                        << " std=[" << gA_std.transpose() << "]\n";
                std::cout << "    ang(acc,-gA): mean=" << angA_mean << " std=" << angA_std
                        << " p50=" << percentile_sorted(ang_acc_A, 0.50f)
                        << " p95=" << percentile_sorted(ang_acc_A, 0.95f)
                        << " max=" << (ang_acc_A.empty() ? NAN : ang_acc_A.back()) << "\n";

                std::cout << "  g_B_B_mean (R*gW)   = [" << gB_mean.transpose() << "], |mean|=" << gB_mean.norm()
                        << " std=[" << gB_std.transpose() << "]\n";
                std::cout << "    ang(acc,-gB): mean=" << angB_mean << " std=" << angB_std
                        << " p50=" << percentile_sorted(ang_acc_B, 0.50f)
                        << " p95=" << percentile_sorted(ang_acc_B, 0.95f)
                        << " max=" << (ang_acc_B.empty() ? NAN : ang_acc_B.back()) << "\n";

                std::cout << "  dq (consecutive quaternion) angle: mean=" << dq_mean << " std=" << dq_std
                        << " p95=" << percentile_sorted(dq_angles, 0.95f)
                        << " max=" << (dq_angles.empty() ? NAN : dq_angles.back()) << "\n";
                std::cout << "  g_B_B drift first->last: " << ang_gB_drift << " deg\n";
                std::cout << "  (sanity) last R det=" << detR << " ortho_err=" << ortho_err << "\n";

                // --- Expected gravity from mount (requires R_CL in scope) ---
                // robot/world "down" (z+ is up in robot frame)
                Eigen::Vector3f g_C(0.f, 0.f, -9.81f);
                Eigen::Vector3f g_L_expected = R_CL.transpose() * g_C; // since p_C = R_CL * p_L

                Eigen::Matrix3f Rx_pi;
                Rx_pi << 1.f, 0.f, 0.f,
                        0.f,-1.f, 0.f,
                        0.f, 0.f,-1.f;
                Eigen::Vector3f g_L_expected_flipYZ = Rx_pi * g_L_expected;

                std::cout << "  g_L_expected(from R_CL)       = [" << g_L_expected.transpose() << "]\n";
                std::cout << "  g_L_expected_flipYZ (Rx_pi*g) = [" << g_L_expected_flipYZ.transpose() << "]\n";

                std::cout << "  ang(g_L_expected,       gA_mean) = " << angle_deg_between(g_L_expected, gA_mean) << " deg\n";
                std::cout << "  ang(g_L_expected,       gB_mean) = " << angle_deg_between(g_L_expected, gB_mean) << " deg\n";
                std::cout << "  ang(g_L_expected_flipYZ,gA_mean) = " << angle_deg_between(g_L_expected_flipYZ, gA_mean) << " deg\n";
                std::cout << "  ang(g_L_expected_flipYZ,gB_mean) = " << angle_deg_between(g_L_expected_flipYZ, gB_mean) << " deg\n";

                std::cout << "========================================\n";
            }
        }
*/




/*
        // 3) Orientace z IMU → R_CL
        Eigen::Vector3f g_L = Eigen::Vector3f::Zero();
        for (const auto &a : acc_samples) {
            g_L += a;
        }
        g_L /= static_cast<float>(acc_samples.size());

        if (!std::isfinite(g_L.x()) || !std::isfinite(g_L.y()) || !std::isfinite(g_L.z())) {
            std::cerr << "[CALIBRATE] invalid IMU average" << std::endl;
            return false;
        }
        if (g_L.norm() < 1e-3f) {
            std::cerr << "[CALIBRATE] IMU gravity vector too small" << std::endl;
            return false;
        }



        Eigen::Vector3f z_C = -g_L.normalized(); // +z_C nahoru
        Eigen::Vector3f z_L(0.0f, 0.0f, 1.0f);

        Eigen::Vector3f x_C = z_L - (z_L.dot(z_C)) * z_C; // projekce osy z_L do roviny ⟂ g
        if (x_C.norm() < 1e-6f) {
            x_C = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
        } else {
            x_C.normalize();
        }
        Eigen::Vector3f y_C = z_C.cross(x_C).normalized();

        Eigen::Matrix3f R_CL;
        R_CL.row(0) = x_C.transpose();
        R_CL.row(1) = y_C.transpose();
        R_CL.row(2) = z_C.transpose();
*/


        // 3) Orientace → R_CL (test: ručně odladěná rotace místo IMU)







        // ***** 4) Hledání roviny země v ROI: x∈[0.30,0.70] a výpočet normály roviny země
        std::vector<float> z_vals;
        z_vals.reserve(100000);
        // tpoints obsahuje všechny body (i mimo ROI), filtrujeme až při průchodu
        std::vector<Eigen::Vector3f> tpoints;
        tpoints.reserve(100000);

        const float roi_x_min = 0.30f;
        const float roi_x_max = 0.70f;
        const float roi_y_abs = 0.20f;
        //const float roi_z_abs = 1.5f;

        for (const auto &cloud : clouds) {
            for (const auto &pt : cloud.points) {
                Eigen::Vector3f p_L(pt.x, pt.y, pt.z);
                Eigen::Vector3f p_C = R_CL * p_L;
                tpoints.push_back(p_C);

                const float x = p_C.x();
                const float y = p_C.y();
                const float z = p_C.z();

                if (x < roi_x_min || x > roi_x_max) continue;
                if (std::fabs(y) > roi_y_abs) continue;
                if (z > 0) continue;

                z_vals.push_back(z);
            }
        }

        ply_logger_.dumpDebugCloud(tpoints, "calibrate_rotated_points");

        const std::size_t N = z_vals.size();
        if (N < 1000) {
            std::cerr << "[CALIBRATE] not enough ground points in ROI, got " << N << std::endl;
            return false;
        }


        std::sort(z_vals.begin(), z_vals.end());
        float h = 0.0f;
        if (N % 2 == 1) {
            h = z_vals[N/2];
        } else {
            h = 0.5f * (z_vals[N/2 - 1] + z_vals[N/2]);
        }

        std::size_t count_ok = 0;
        const float max_dev = 0.05f; // ±5 cm
        float h_max = 0.0f;
        for (float z : z_vals) {
            auto dev = std::fabs(z - h);
            if (dev > h_max) {
                h_max = dev;
            }
            if (dev <= max_dev) {
                ++count_ok;
            }
        }

        std::cout << "[CALIBRATE] ground plane at h=" << h
                  << " m, max deviation: " << h_max
                  << " m, inliers: " << count_ok << " / " << N << std::endl;

        const double ratio = static_cast<double>(count_ok) / static_cast<double>(N);
        if (ratio < 0.9) {
            std::cerr << "[CALIBRATE] ground plane test failed: ratio=" << ratio
                      << " (N=" << N << ")" << std::endl;
            return false;
        }


        // 4a) Určení normály roviny země (PCA na inliers)
        Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
        std::vector<Eigen::Vector3f> inliers;
        inliers.reserve(count_ok);

        for (const auto &p : tpoints) {
            // Znovu filtrujeme ROI (tpoints obsahuje vše)
            if (p.x() < roi_x_min || p.x() > roi_x_max) continue;
            if (std::fabs(p.y()) > roi_y_abs) continue;
            // Z-inlier check vůči mediánu
            if (std::fabs(p.z() - h) > max_dev) continue;

            inliers.push_back(p);
            centroid += p;
        }

        if (inliers.empty()) {
            std::cerr << "[CALIBRATE] no inliers for normal computation" << std::endl;
            return false;
        }
        centroid /= static_cast<float>(inliers.size());

        Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
        for (const auto &p : inliers) {
            Eigen::Vector3f d = p - centroid;
            cov += d * d.transpose();
        }
        cov /= static_cast<float>(inliers.size());

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
        // Normála odpovídá vlastnímu vektoru s nejmenším vlastním číslem (sloupec 0)
        Eigen::Vector3f normal = solver.eigenvectors().col(0);

        // Orientace normály nahoru (ve směru Z)
        if (normal.z() < 0) {
            normal = -normal;
        }
        std::cout << "[CALIBRATE] ground normal: " << normal.transpose() << std::endl;

        // 4b) Oprava transformační matice (zarovnání normály s osou Z)
        Eigen::Quaternionf q_corr = Eigen::Quaternionf::FromTwoVectors(normal, Eigen::Vector3f::UnitZ());
        Eigen::Matrix3f R_corr = q_corr.toRotationMatrix();

        // Aktualizujeme R_CL o korekci
        R_CL = R_corr * R_CL;

        // Přepočítáme výšku h v novém rámci (střední hodnota Z inlierů po rotaci)
        double sum_z_new = 0.0;
        for (const auto &p : inliers) {
            // p je v původním R_CL rámci, aplikujeme R_corr
            Eigen::Vector3f p_new = R_corr * p;
            sum_z_new += p_new.z();
        }
        float h_new = static_cast<float>(sum_z_new / inliers.size());
        std::cout << "[CALIBRATE] corrected height h=" << h << " -> h_new=" << h_new << std::endl;

        Eigen::Matrix4f T_CL = Eigen::Matrix4f::Identity();
        T_CL.block<3,3>(0,0) = R_CL;
        T_CL(2,3) = -h_new; // země → z_C = 0



        // 5) Detekce obrysu robota (z≥5 cm, r≤1 m)
        const float r_max        = 0.7f;
        const float z_robot_min  = 0.07f;

        float x_robot_max = -std::numeric_limits<float>::infinity();
        float y_robot_min =  std::numeric_limits<float>::infinity();
        float y_robot_max = -std::numeric_limits<float>::infinity();

        for (const auto &cloud : clouds) {
            for (const auto &pt : cloud.points) {
                Eigen::Vector4f p_L_h(pt.x, pt.y, pt.z, 1.0f);
                Eigen::Vector4f p_C_h = T_CL * p_L_h;

                const float x = p_C_h.x();
                const float y = p_C_h.y();
                const float z = p_C_h.z();

                if (z < z_robot_min) continue;

                const float r2 = x*x + y*y;
                if (r2 > r_max * r_max) continue;

                if (x > x_robot_max) x_robot_max = x;
                if (y < y_robot_min) y_robot_min = y;
                if (y > y_robot_max) y_robot_max = y;
            }
        }

        if (!std::isfinite(x_robot_max) ||
            !std::isfinite(y_robot_min) ||
            !std::isfinite(y_robot_max)) {
            std::cerr << "[CALIBRATE] no robot points detected" << std::endl;
            return false;
        }

        const float safety_x = 0.05f;
        const float safety_y = 0.05f;

        out.T_CL         = T_CL;
        out.mask_front_x = x_robot_max + safety_x;
        out.mask_x_min   = -0.70f; // zadní hrana robota (konstanta)
        out.mask_y_min   = y_robot_min - safety_y;
        out.mask_y_max   = y_robot_max + safety_y;

        if (!file.empty()) {
            if (!saveCalibration(file, out)) {
                std::cerr << "[CALIBRATE] failed to save calibration to " << file << std::endl;
                return false;
            }
        }

        std::cout << "[CALIBRATE] done: h=" << h
                  << ", mask_front_x=" << out.mask_front_x
                  << ", mask_x_min="  << out.mask_x_min
                  << ", mask_y_min="  << out.mask_y_min
                  << ", mask_y_max="  << out.mask_y_max
                  << std::endl;

        return true;
    }
private:
    PLYLogger<std::vector<Eigen::Vector3f>> ply_logger_;

};
