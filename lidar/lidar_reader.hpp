#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <iostream>

#include "unitree_lidar_sdk.h"
#include "unitree_lidar_protocol.h"

namespace unilidar = unilidar_sdk2;

// RAII wrapper kolem UnitreeLidarReader (SDK2).
class LidarReader {
public:
    LidarReader() = default;
    ~LidarReader() = default;

    bool initializeUDP(uint16_t lidar_port = 6101,
                       const std::string &lidar_ip = "192.168.10.62",
                       uint16_t local_port = 6201,
                       const std::string &local_ip = "192.168.10.2",
                       uint16_t cloud_scan_num = 3,
                       bool use_system_timestamp = true)
    {
        if (reader_) {
            return true;
        }

        reader_.reset(unilidar::createUnitreeLidarReader());
        if (!reader_) {
            std::cerr << "[LidarReader] createUnitreeLidarReader returned nullptr" << std::endl;
            return false;
        }

        int rc = reader_->initializeUDP(lidar_port, lidar_ip, local_port, local_ip,
                                        cloud_scan_num, use_system_timestamp);
        std::cout << "[LidarReader] initializeUDP rc = " << rc << std::endl;
        if (rc != 0) {
            reader_.reset();
            return false;
        }
        return true;
    }

    bool isInitialized() const noexcept { return static_cast<bool>(reader_); }

    void startRotation() {
        if (reader_) reader_->startLidarRotation();
    }

    void stopRotation() {
        if (reader_) reader_->stopLidarRotation();
    }

    void clearBuffer() {
        if (reader_) reader_->clearBuffer();
    }

    int runParse() {
        if (!reader_) return -1;
        return reader_->runParse();
    }

    bool getPointCloud(unilidar::PointCloudUnitree &cloud_out) {
        if (!reader_) return false;
        return reader_->getPointCloud(cloud_out);
    }

    bool getImuData(unilidar::LidarImuData &imu_out) {
        if (!reader_) return false;
        return reader_->getImuData(imu_out);
    }

    bool setMode(uint32_t mode) {
        if (!reader_) return false;
        reader_->setLidarWorkMode(mode);
        return true;
    }

    const unilidar::LidarPointDataPacket& getPointPacket() const {
        static unilidar::LidarPointDataPacket dummy{};
        if (!reader_) return dummy;
        return reader_->getLidarPointDataPacket();
    }

    const unilidar::LidarImuDataPacket& getImuPacket() const {
        static unilidar::LidarImuDataPacket dummy{};
        if (!reader_) return dummy;
        return reader_->getLidarImuDataPacket();
    }

    const unilidar::LidarVersionDataPacket& getVersionPacket() const {
        static unilidar::LidarVersionDataPacket dummy{};
        if (!reader_) return dummy;
        return reader_->getLidarVersionDataPacket();
    }

private:
    std::unique_ptr<unilidar::UnitreeLidarReader> reader_;
};
