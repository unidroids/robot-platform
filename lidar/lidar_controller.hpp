#pragma once

// lidar_controller.hpp — řadič Unitree L2 LiDARu (SDK2)
// ---------------------------------------------------------------------------
// Orchestrace: UDP reader, kalibrace, logování a metriky nad ring bufferem.
// Používá obálku LidarReader, ring buffer pro body/IMU a samostatný kalibrátor.
// ---------------------------------------------------------------------------

#include <atomic>
#include <thread>
#include <memory>
#include <cmath>
#include <iostream>
#include <chrono>
#include <mutex>
#include <limits>
#include <cstdint>
#include <iomanip>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "unitree_lidar_sdk.h"
#include "unitree_lidar_protocol.h"

#include "lidar_reader.hpp"
#include "lidar_calibration.hpp"
#include "point_buffer.hpp"
#include "distance_evaluator.hpp"
#include "imu_buffer.hpp"
#include "imu_stats.hpp"
#include "raw_logger.hpp"

namespace unilidar = unilidar_sdk2;

class LidarController {
public:
    LidarController() = default;

    ~LidarController() {
        stop();
    }

    // Volitelný helper – jen zajistí initializeUDP.
    bool connect() {
        std::lock_guard<std::mutex> lg(mtx_);
        if (reader_.isInitialized()) {
            std::cout << "[CONNECT] already connected" << std::endl;
            return true;
        }
        return ensureReaderLocked();
    }

    bool start() {
        {
            std::lock_guard<std::mutex> lg(mtx_);
            if (running_) {
                std::cout << "[LIDAR] already running" << std::endl;
                return true;
            }

            if (!ensureReaderLocked()) {
                return false;
            }

            // Načti kalibraci, pokud ještě není.
            if (!calibration_loaded_) {
                LidarCalibration tmp;
                if (!loadCalibration("calibration.dat", tmp)) {
                    std::cout << "[LIDAR] start: calibration.dat not found or invalid, run CALIBRATE first" << std::endl;
                    return false;
                }
                calibration_        = tmp;
                calibration_loaded_ = true;
                applyCalibrationToBuffers();
                std::cout << "[LIDAR] start: calibration loaded" << std::endl;
            }

            point_buffer_.clear();
            imu_buffer_.clear();

            running_.store(true, std::memory_order_relaxed);
        } // lock uvolněn

        try {
            reader_.startRotation();
        } catch (const std::exception &e) {
            std::cerr << "[LIDAR] startRotation failed: " << e.what() << std::endl;
            running_.store(false, std::memory_order_relaxed);
            return false;
        } catch (...) {
            std::cerr << "[LIDAR] startRotation failed (unknown)" << std::endl;
            running_.store(false, std::memory_order_relaxed);
            return false;
        }

        worker_ = std::thread([this]{ loopRead(); });
        return true;
    }

    // Zastaví čtecí vlákno a rotaci,
    // UDP / reader_ nechá žít (re-use při dalším START).
    void stop() {
        {
            std::lock_guard<std::mutex> lg(mtx_);
            if (!running_.load(std::memory_order_relaxed)) return;
            running_.store(false, std::memory_order_relaxed);
        }

        if (worker_.joinable()) {
            worker_.join();
        }

        try {
            reader_.stopRotation();
        } catch (...) {
            std::cerr << "[LIDAR] stop: exception in stopRotation" << std::endl;
        }

        {
            std::lock_guard<std::mutex> lg(mtx_);
            point_buffer_.clear();
            imu_buffer_.clear();
        }

        std::cout << "[LIDAR] stopped" << std::endl;
    }

    // Nastaví pracovní mód LiDARu (bitová maska podle SDK).
    // Lze volat pouze, pokud LiDAR neběží.
    bool setMode(uint32_t mode) {
        std::cout << "[setMode] request " << mode << std::endl;

        std::lock_guard<std::mutex> lock(mtx_);
        if (running_.load(std::memory_order_relaxed)) {
            std::cout << "[setMode] cannot change mode while running" << std::endl;
            return false;
        }

        if (!ensureReaderLocked()) {
            std::cerr << "[setMode] ensureReaderLocked/initReader failed" << std::endl;
            return false;
        }

        try {
            reader_.setMode(mode);
            std::cout << "[setMode] mode sent: " << mode << std::endl;
        } catch (...) {
            std::cerr << "[setMode] exception while setting mode" << std::endl;
            return false;
        }

        return true;
    }

    bool getDistance(float &dist_out) {
        dist_out = distance_evaluator_.distance(point_buffer_);
        return dist_out >= 0.0f;
    }

    ImuStatistics getImuStats() const {
        return imu_stats_.evaluate(imu_buffer_);
    }

    // Spustí kalibraci (10 s sběr dat) a uloží výsledek do calibration.dat.
    bool calibrate(const std::string &file = "calibration.dat") {
        std::cout << "[CALIBRATE] starting calibration" << std::endl;

        {
            std::lock_guard<std::mutex> lg(mtx_);
            if (running_.load(std::memory_order_relaxed)) {
                std::cerr << "[CALIBRATE] cannot calibrate while LiDAR is running" << std::endl;
                return false;
            }
            if (!ensureReaderLocked()) {
                std::cerr << "[CALIBRATE] ensureReaderLocked() failed" << std::endl;
                return false;
            }
        }

        LidarCalibration calib;
        if (!calibrator_.run(reader_, calib, file)) {
            return false;
        }

        {
            std::lock_guard<std::mutex> lg(mtx_);
            calibration_        = calib;
            calibration_loaded_ = true;
            applyCalibrationToBuffers();
            point_buffer_.clear();
            imu_buffer_.clear();
        }

        return true;
    }

private:
    // Vytvoří reader_ a zavolá initializeUDP(), pokud ještě reader_ neexistuje.
    // PŘEDPOKLAD: volající drží mtx_.
    bool ensureReaderLocked() {
        if (reader_.isInitialized()) return true;

        std::string lidar_ip  = "192.168.10.62";
        std::string local_ip  = "192.168.10.2";
        uint16_t lidar_port   = 6101;
        uint16_t local_port   = 6201;
        uint16_t cloud_scan_num = 3;
        bool use_system_timestamp = true;

        if (!reader_.initializeUDP(lidar_port, lidar_ip, local_port, local_ip,
                                   cloud_scan_num, use_system_timestamp)) {
            std::cerr << "[LIDAR] initializeUDP failed" << std::endl;
            return false;
        }

        return true;
    }

    void applyCalibrationToBuffers() {
        point_buffer_.setCalibrationAvailable(true);
        LidarPointBuffer::Mask mask;
        mask.front_x = calibration_.mask_front_x;
        mask.x_min   = calibration_.mask_x_min;
        mask.y_min   = calibration_.mask_y_min;
        mask.y_max   = calibration_.mask_y_max;
        mask.enabled = true;
        point_buffer_.setMask(mask);
    }

    void processCloudData() {
        unilidar::PointCloudUnitree cloud;
        if (!reader_.getPointCloud(cloud)) {
            return;
        }

        point_buffer_.pushCloud(cloud, calibration_.T_CL);

        float cloud_min = distance_evaluator_.distance(point_buffer_);
        if (cloud_min >= 0.0f) {
            latest_.store(cloud_min, std::memory_order_relaxed);
            seq_.fetch_add(1u, std::memory_order_relaxed);
        }
    }

    void processIMUData() {
        unilidar::LidarImuData imu{};
        if (!reader_.getImuData(imu)) {
            return;
        }

        ImuRingBuffer::Sample s{};
        const auto &info = imu.info;
        s.timestamp =
            static_cast<double>(info.stamp.sec) +
            static_cast<double>(info.stamp.nsec) / 1.0e9;
        s.quaternion[0] = imu.quaternion[0];
        s.quaternion[1] = imu.quaternion[1];
        s.quaternion[2] = imu.quaternion[2];
        s.quaternion[3] = imu.quaternion[3];
        s.angular_velocity[0] = imu.angular_velocity[0];
        s.angular_velocity[1] = imu.angular_velocity[1];
        s.angular_velocity[2] = imu.angular_velocity[2];
        s.linear_acceleration[0] = imu.linear_acceleration[0];
        s.linear_acceleration[1] = imu.linear_acceleration[1];
        s.linear_acceleration[2] = imu.linear_acceleration[2];

        imu_buffer_.push(s);
    }

    inline uint64_t getMonotonicTimeNs() {
        using namespace std::chrono;
        return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
    }

    // Čtecí smyčka: parsuje pakety, deleguje na processCloudData/processIMUData.
    void loopRead() {
        LidarRawLogger raw_logger;

        while (running_.load(std::memory_order_relaxed)) {
            int type = reader_.runParse();
            uint64_t mono_ts_ns = getMonotonicTimeNs();

            if (type == LIDAR_POINT_DATA_PACKET_TYPE) {
                const auto& pkt = reader_.getPointPacket();
                raw_logger.writePointPacket(pkt, mono_ts_ns);
                processCloudData();
            } else if (type == LIDAR_IMU_DATA_PACKET_TYPE) {
                const auto& pkt = reader_.getImuPacket();
                raw_logger.writeImuPacket(pkt, mono_ts_ns);
                processIMUData();
            } else if (type == LIDAR_VERSION_PACKET_TYPE) {
                const auto& pkt = reader_.getVersionPacket();
                raw_logger.writeVersionPacket(pkt, mono_ts_ns);
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    }

    // ------------------------------------------------------------------------
    // Členské proměnné
    // ------------------------------------------------------------------------

    LidarCalibration calibration_;
    bool             calibration_loaded_{false};
    LidarCalibrator  calibrator_;
    LidarReader      reader_;
    std::thread      worker_;

    LidarPointBuffer      point_buffer_;
    LidarDistanceEvaluator distance_evaluator_;
    ImuRingBuffer         imu_buffer_;
    ImuStats              imu_stats_;

    std::atomic<bool>     running_{false};
    std::atomic<float>    latest_{-1.0f};
    std::atomic<uint64_t> seq_{0};

    mutable std::mutex mtx_;
};
