#pragma once

#include <cstddef>
#include <limits>
#include <Eigen/Dense>

#include "imu_buffer.hpp"

struct ImuStatistics {
    double t_start = std::numeric_limits<double>::quiet_NaN();
    double t_end   = std::numeric_limits<double>::quiet_NaN();
    std::size_t count = 0;
    Eigen::Vector3d acc_mean  = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_std   = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_mean = Eigen::Vector3d::Zero();
};

// Lehká statistika nad IMU ring bufferem.
class ImuStats {
public:
    ImuStatistics evaluate(const ImuRingBuffer &buffer) const {
        ImuStatistics stats;
        const auto samples = buffer.snapshot();
        const std::size_t N = samples.size();
        stats.count = N;
        if (N == 0) {
            return stats;
        }

        stats.t_start = samples.front().timestamp;
        stats.t_end   = samples.back().timestamp;

        Eigen::Vector3d sum_acc = Eigen::Vector3d::Zero();
        Eigen::Vector3d sum_acc_sq = Eigen::Vector3d::Zero();
        Eigen::Vector3d sum_gyro = Eigen::Vector3d::Zero();

        for (const auto &s : samples) {
            Eigen::Vector3d acc(s.linear_acceleration[0],
                                s.linear_acceleration[1],
                                s.linear_acceleration[2]);
            Eigen::Vector3d gyro(s.angular_velocity[0],
                                 s.angular_velocity[1],
                                 s.angular_velocity[2]);

            sum_acc    += acc;
            sum_acc_sq += acc.cwiseProduct(acc);
            sum_gyro   += gyro;
        }

        const double invN = 1.0 / static_cast<double>(N);
        stats.acc_mean  = sum_acc * invN;
        stats.acc_std   = (sum_acc_sq * invN - stats.acc_mean.cwiseProduct(stats.acc_mean))
                            .cwiseMax(0.0).cwiseSqrt();
        stats.gyro_mean = sum_gyro * invN;

        return stats;
    }
};
