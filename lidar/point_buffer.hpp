#pragma once

#include <array>
#include <cstdint>
#include <vector>
#include <cmath>
#include <string>
#include <utility>
#include <Eigen/Dense>

#include "unitree_lidar_utilities.h"  // PointCloudUnitree
#include "ply_logger.hpp"

// Ring buffer transformovaných LiDAR bodů (po aplikaci T_CL a ignore-boxu).
// Drží data v centimetrech (stejně jako původní LidarPointProcessing).
class LidarPointBuffer {
public:
    struct Sample {
        float x;
        float y;
        float z;
        float intensity;
        double ftime;
        double rtime;
        std::uint32_t ring;
    };

    struct Mask {
        float front_x = 0.0f;
        float x_min   = 0.0f;
        float y_min   = 0.0f;
        float y_max   = 0.0f;
        bool  enabled = false;
    };

    static constexpr std::size_t kCapacity = 1u << 16; // 65536 bodů

    explicit LidarPointBuffer(std::string base_dir = "/data/robot/lidar")
        : ply_logger_(std::move(base_dir))
    {}

    void setCalibrationAvailable(bool enabled) { has_calibration_ = enabled; }
    void setMask(const Mask &mask) { mask_ = mask; mask_.enabled = true; }
    void clear() { head_ = 0; size_ = 0; }

    std::size_t size() const noexcept { return size_; }

    // Transformuje cloud do rámce C, přeškáluje na cm, aplikuje masku a ukládá do ring bufferu.
    // T_CL musí být 4x4 homogenní matice v metrech.
    void pushCloud(const unilidar_sdk2::PointCloudUnitree &cloud_L,
                   const Eigen::Matrix4f &T_CL)
    {
        const Eigen::Matrix4f T_proc = processingTransform(T_CL);

        const double base_stamp = cloud_L.stamp;  // absolutní čas začátku scanu
        Eigen::Vector4f p(0.0f, 0.0f, 0.0f, 1.0f);

        for (const auto &pt : cloud_L.points) {
            p << pt.x, pt.y, pt.z, 1.0f;
            const Eigen::Vector4f q4 = T_proc * p;
            const Eigen::Vector3f q  = q4.head<3>();

            if (ignoreBox(q.x(), q.y())) {
                continue;
            }

            Sample s{};
            s.x = q.x();
            s.y = q.y();
            s.z = q.z();
            s.intensity = pt.intensity;
            s.ftime = base_stamp;
            s.rtime = static_cast<double>(pt.time);
            s.ring  = pt.ring;

            pushSample(s);
        }
    }

    std::vector<Sample> snapshot() const
    {
        std::vector<Sample> out;
        const std::size_t N = (size_ < kCapacity) ? size_ : kCapacity;
        out.reserve(N);
        for (std::size_t i = 0; i < N; ++i) {
            out.push_back(buffer_[i]);
        }
        return out;
    }

private:
    // Výchozí transformace, pokud není kalibrace k dispozici.
    static const Eigen::Matrix4f &defaultTransformMatrix()
    {
        static const Eigen::Matrix4f M = [] {
            const float deg  = static_cast<float>(M_PI) / 180.0f;
            const float th_z = -25.5f * deg;
            const float th_y = -47.5f * deg;

            Eigen::Matrix4f Rz;
            Rz <<  std::cos(th_z),  std::sin(th_z), 0, 0,
                  -std::sin(th_z),  std::cos(th_z), 0, 0,
                                 0,             0, 1, 0,
                                 0,             0, 0, 1;

            Eigen::Matrix4f Ry;
            Ry <<  std::cos(th_y), 0, -std::sin(th_y), 0,
                                 0, 1,              0, 0,
                   std::sin(th_y), 0,  std::cos(th_y), 0,
                                 0, 0,              0, 1;

            Eigen::Matrix4f Ms = Eigen::Matrix4f::Identity();
            Ms(0,0) = Ms(1,1) = Ms(2,2) = 100.0f;   // m → cm

            Eigen::Matrix4f Tx = Eigen::Matrix4f::Identity();
            Tx = Ms * Ry * Rz;
            return Tx;
        }();
        return M;
    }

    // Vrátí transformaci pro zpracování (Ms*T_CL pokud kalibrace, jinak default).
    Eigen::Matrix4f processingTransform(const Eigen::Matrix4f &T_CL) const
    {
        if (has_calibration_) {
            Eigen::Matrix4f Ms = Eigen::Matrix4f::Identity();
            Ms(0,0) = Ms(1,1) = Ms(2,2) = 100.0f;   // m → cm
            return Ms * T_CL;
        }
        return defaultTransformMatrix();
    }

    bool ignoreBox(float x_cm, float y_cm) const
    {
        if (!mask_.enabled) {
            // fallback původní fixní kvádr v cm
            return (y_cm > -20.0f && y_cm <  20.0f &&
                    x_cm <  20.0f && x_cm > -50.0f);
        }

        const float x = x_cm / 100.0f;  // zpět do metrů
        const float y = y_cm / 100.0f;

        return (x >= mask_.x_min && x <= mask_.front_x &&
                y >= mask_.y_min && y <= mask_.y_max);
    }

    void pushSample(const Sample &s)
    {
        buffer_[static_cast<std::size_t>(head_)] = s;

        ++head_;

        if (size_ < kCapacity) {
            ++size_;
        }

        if (size_ == kCapacity && head_ == 0) {
            ply_logger_.dump(buffer_.data(), kCapacity);
        }
    }

    Mask mask_{};
    bool has_calibration_{false};

    std::array<Sample, kCapacity> buffer_{};
    std::uint16_t head_{0};
    std::size_t   size_{0};
    PLYLogger<Sample> ply_logger_;
};
