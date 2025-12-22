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

        std::vector<Eigen::Vector3f> acc_samples;
        acc_samples.reserve(5000);
        std::vector<unilidar_sdk2::PointCloudUnitree> clouds;
        clouds.reserve(64);

        auto t_start = steady_clock::now();
        auto t_end   = t_start + duration;

        while (steady_clock::now() < t_end) {
            int type = reader.runParse();

            if (type == LIDAR_IMU_DATA_PACKET_TYPE) {
                unilidar_sdk2::LidarImuData imu{};
                if (!reader.getImuData(imu)) {
                    continue;
                }

                Eigen::Vector3f gyro(
                    imu.angular_velocity[0],
                    imu.angular_velocity[1],
                    imu.angular_velocity[2]);
                float gyro_norm = gyro.norm();

                // Přibližně "stojím" – ignoruj velké rotační rychlosti
                if (gyro_norm > 0.05f) {
                    continue;
                }

                Eigen::Vector3f acc(
                    imu.linear_acceleration[0],
                    imu.linear_acceleration[1],
                    imu.linear_acceleration[2]);
                acc_samples.push_back(acc);
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

        // 4) Hledání roviny země v ROI: x∈[0.30,0.70], |y|≤0.20
        std::vector<float> z_vals;
        z_vals.reserve(100000);

        const float roi_x_min = 0.30f;
        const float roi_x_max = 0.70f;
        const float roi_y_abs = 0.20f;
        const float roi_z_abs = 1.0f;

        for (const auto &cloud : clouds) {
            for (const auto &pt : cloud.points) {
                Eigen::Vector3f p_L(pt.x, pt.y, pt.z);
                Eigen::Vector3f p_C = R_CL * p_L;

                const float x = p_C.x();
                const float y = p_C.y();
                const float z = p_C.z();

                if (x < roi_x_min || x > roi_x_max) continue;
                if (std::fabs(y) > roi_y_abs) continue;
                if (std::fabs(z) > roi_z_abs) continue;

                z_vals.push_back(z);
            }
        }

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
        for (float z : z_vals) {
            if (std::fabs(z - h) <= max_dev) {
                ++count_ok;
            }
        }
        const double ratio = static_cast<double>(count_ok) / static_cast<double>(N);
        if (ratio < 0.9) {
            std::cerr << "[CALIBRATE] ground plane test failed: ratio=" << ratio
                      << " (N=" << N << ")" << std::endl;
            return false;
        }

        Eigen::Matrix4f T_CL = Eigen::Matrix4f::Identity();
        T_CL.block<3,3>(0,0) = R_CL;
        T_CL(2,3) = -h; // země → z_C = 0

        // 5) Detekce obrysu robota (z≥5 cm, r≤1 m)
        const float r_max        = 1.0f;
        const float z_robot_min  = 0.05f;

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
};
