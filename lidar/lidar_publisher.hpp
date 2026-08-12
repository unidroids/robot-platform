#pragma once

#include <thread>
#include <atomic>
#include <chrono>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <zmq.hpp>
#include <cstring>

#include "lidar_controller.hpp"

class LidarPublisher {
public:
    LidarPublisher(LidarController* controller) 
        : controller_(controller), running_(false) {}

    ~LidarPublisher() {
        stop();
    }

    void start() {
        if (running_) return;
        running_ = true;
        pub_thread_ = std::thread(&LidarPublisher::publishLoop, this);
    }

    void stop() {
        if (!running_) return;
        running_ = false;
        if (pub_thread_.joinable()) {
            pub_thread_.join();
        }
    }

private:
    void publishLoop() {
        try {
            zmq::context_t context(1);
            // Zpětná kompatibilita pro starší verze cppzmq
            #ifdef ZMQ_CPP11
            zmq::socket_t publisher(context, zmq::socket_type::pub);
            #else
            zmq::socket_t publisher(context, ZMQ_PUB);
            #endif
            
            std::string ipc_path = "ipc:///tmp/robot-lidar";
            publisher.bind(ipc_path);
            chmod("/tmp/robot-lidar", 0777); // aby mohl cist kdokoli
            
            std::cout << "📡 [LidarPublisher] ZMQ PUB běží na " << ipc_path << " (20 Hz)" << std::endl;

            while (running_) {
                float dist = -1.0f;
                // getDistance vrátí true, pokud je buffer LiDARu naplněn
                if (controller_->getDistance(dist)) {
                    std::string topic = "DISTANCE";
                    std::string payload = "{\"distance\": " + std::to_string(dist) + "}";
                    
                    zmq::message_t topic_msg(topic.size());
                    std::memcpy(topic_msg.data(), topic.c_str(), topic.size());
                    
                    zmq::message_t payload_msg(payload.size());
                    std::memcpy(payload_msg.data(), payload.c_str(), payload.size());
                    
                    publisher.send(topic_msg, zmq::send_flags::sndmore);
                    publisher.send(payload_msg, zmq::send_flags::none);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 20 Hz
            }
        } catch (const std::exception& e) {
            std::cerr << "❌ [LidarPublisher] Chyba v ZMQ smyčce: " << e.what() << std::endl;
        }
    }

    LidarController* controller_;
    std::atomic<bool> running_;
    std::thread pub_thread_;
};
