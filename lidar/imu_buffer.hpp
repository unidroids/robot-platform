#pragma once

#include <array>
#include <cstdint>
#include <vector>

// Ring buffer IMU vzorků (~2 s, 512 záznamů).
class ImuRingBuffer {
public:
    struct Sample {
        double timestamp = 0.0;  // [s]
        double quaternion[4] = {0.0, 0.0, 0.0, 1.0};  // w, x, y, z
        double angular_velocity[3] = {0.0, 0.0, 0.0}; // rad/s
        double linear_acceleration[3] = {0.0, 0.0, 0.0}; // m/s^2
    };

    static constexpr std::size_t kCapacity = 512;

    void push(const Sample &s) {
        buffer_[head_] = s;
        head_ = (head_ + 1) % kCapacity;
        if (size_ < kCapacity) {
            ++size_;
        }
    }

    void clear() {
        head_ = 0;
        size_ = 0;
    }

    std::size_t size() const noexcept { return size_; }

    std::vector<Sample> snapshot() const {
        std::vector<Sample> out;
        out.reserve(size_);
        for (std::size_t i = 0; i < size_; ++i) {
            const std::size_t idx = (head_ + kCapacity - size_ + i) % kCapacity;
            out.push_back(buffer_[idx]);
        }
        return out;
    }

private:
    std::array<Sample, kCapacity> buffer_{};
    std::size_t head_{0};
    std::size_t size_{0};
};
