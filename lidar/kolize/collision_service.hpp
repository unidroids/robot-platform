#pragma once

#include "collision_config.hpp"
#include "collision_evaluator.hpp"
#include "../lidar_controller.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <algorithm>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/stat.h>

#include <zmq.hpp>

// Služba LIDAR-KOLIZE (TCP port 9222)
class CollisionService {
public:
    enum class State {
        IDLE,
        WAITING_FOR_DATA,
        RUNNING
    };

    explicit CollisionService(LidarController *lidar,
                              uint16_t port = 9222,
                              std::string bind_addr = "127.0.0.1")
        : lidar_(lidar),
          port_(port),
          bind_addr_(std::move(bind_addr)),
          evaluator_(config_)
    {}

    ~CollisionService() {
        stop();
    }

    void setShutdownCallback(std::function<void()> cb) {
        shutdown_cb_ = std::move(cb);
    }

    bool isRunning() const {
        return running_server_.load();
    }

    bool isEvaluating() const {
        return evaluating_.load();
    }

    // Nastartuje TCP server na portu 9222 a vlákno odběru headingu
    bool startServer() {
        if (running_server_.load()) return true;

        server_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd_ < 0) {
            std::perror("[LIDAR-KOLIZE] socket");
            return false;
        }

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port   = htons(port_);
        if (inet_pton(AF_INET, bind_addr_.c_str(), &addr.sin_addr) <= 0) {
            std::perror("[LIDAR-KOLIZE] inet_pton");
            ::close(server_fd_);
            server_fd_ = -1;
            return false;
        }

        int opt = 1;
        setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        if (bind(server_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            std::perror("[LIDAR-KOLIZE] bind");
            ::close(server_fd_);
            server_fd_ = -1;
            return false;
        }

        if (listen(server_fd_, 8) < 0) {
            std::perror("[LIDAR-KOLIZE] listen");
            ::close(server_fd_);
            server_fd_ = -1;
            return false;
        }

        std::cout << "🛡️  [LIDAR-KOLIZE] TCP server naslouchá na " << bind_addr_ << ":" << port_ << std::endl;

        running_server_.store(true);
        tcp_thread_ = std::thread([this]{ tcpListenLoop(); });
        heading_thread_ = std::thread([this]{ headingSubLoop(); });

        return true;
    }

    // Spustí vyhodnocování (příkaz START na 9222)
    bool startEvaluating() {
        if (!running_server_.load()) {
            if (!startServer()) return false;
        }

        std::lock_guard<std::mutex> lg(eval_mtx_);
        if (evaluating_.load()) return true;

        evaluator_.reset();
        last_seq_ = lidar_ ? lidar_->getPointBuffer().totalWritten() : 0;
        last_eval_timestamp_ = 0.0;
        state_.store(State::WAITING_FOR_DATA);
        waiting_reason_ = "INITIALIZING";

        evaluating_.store(true);
        eval_thread_ = std::thread([this]{ evalLoop(); });

        std::cout << "🚀 [LIDAR-KOLIZE] Vyhodnocování kolizních kurzů spuštěno (10 Hz)" << std::endl;
        return true;
    }

    // Zastaví vyhodnocování (příkaz STOP na 9222)
    void stopEvaluating() {
        if (!evaluating_.load()) return;
        evaluating_.store(false);

        if (eval_thread_.joinable()) {
            eval_thread_.join();
        }

        state_.store(State::IDLE);
        std::cout << "🛑 [LIDAR-KOLIZE] Vyhodnocování kolizních kurzů zastaveno" << std::endl;
    }

    // Úplné zastavení služby a serveru
    void stop() {
        stopEvaluating();

        if (running_server_.exchange(false)) {
            int fd = server_fd_;
            server_fd_ = -1;
            if (fd >= 0) {
                ::shutdown(fd, SHUT_RDWR);
                ::close(fd);
            }

            closeAllClients();

            if (tcp_thread_.joinable()) {
                tcp_thread_.join();
            }
            if (heading_thread_.joinable()) {
                heading_thread_.join();
            }
            std::cout << "🛑 [LIDAR-KOLIZE] Služba ukončena" << std::endl;
        }
    }

    std::string getStatusJson() const {
        std::ostringstream ss;
        std::string state_str = "IDLE";
        if (state_.load() == State::RUNNING) state_str = "RUNNING";
        else if (state_.load() == State::WAITING_FOR_DATA) state_str = "WAITING_FOR_DATA";

        ss << state_str << " {"
           << "\"cycles_total\":" << cycles_total_.load() << ","
           << "\"cycles_published\":" << cycles_published_.load() << ","
           << "\"cycles_low_points\":" << cycles_low_points_.load() << ","
           << "\"cycles_lag_dropped\":" << cycles_lag_dropped_.load() << ","
           << "\"points_last_cycle\":" << points_last_cycle_.load() << ","
           << "\"min_clearance_cm\":" << std::fixed << std::setprecision(1) << min_clearance_cm_.load() << ","
           << "\"min_ttc_s\":" << std::setprecision(2) << min_ttc_s_.load() << ","
           << "\"heading_avg\":" << std::setprecision(1) << heading_avg_.load() << ","
           << "\"eval_duration_ms\":" << std::setprecision(2) << last_eval_duration_ms_.load() << ","
           << "\"waiting_reason\":\"" << waiting_reason_ << "\""
           << "}";
        return ss.str();
    }

    std::string getLatestJson() const {
        std::lock_guard<std::mutex> lg(data_mtx_);
        return latest_json_;
    }

private:
    void sendLine(int sock, const std::string &msg) {
        std::string out = msg + "\n";
        ::send(sock, out.data(), out.size(), MSG_NOSIGNAL);
    }

    void closeAllClients() {
        std::lock_guard<std::mutex> lg(clients_mtx_);
        for (int s : client_socks_) {
            ::shutdown(s, SHUT_RDWR);
            ::close(s);
        }
        client_socks_.clear();
    }

    // -------------------------------------------------------------
    // Vlákno TCP serveru pro port 9222
    // -------------------------------------------------------------
    void tcpListenLoop() {
        while (running_server_.load()) {
            sockaddr_in cli{};
            socklen_t len = sizeof(cli);
            int cs = ::accept(server_fd_, reinterpret_cast<sockaddr*>(&cli), &len);
            if (cs < 0) {
                if (!running_server_.load()) break;
                std::perror("[LIDAR-KOLIZE] accept");
                continue;
            }
            std::thread([this, cs]{ handleClient(cs); }).detach();
        }
    }

    void handleClient(int sock) {
        {
            std::lock_guard<std::mutex> lg(clients_mtx_);
            client_socks_.push_back(sock);
        }

        std::string buffer;
        char tmp[512];

        while (running_server_.load()) {
            ssize_t n = ::recv(sock, tmp, sizeof(tmp), 0);
            if (n <= 0) break;
            buffer.append(tmp, n);

            size_t pos;
            while ((pos = buffer.find('\n')) != std::string::npos) {
                std::string line = buffer.substr(0, pos);
                buffer.erase(0, pos + 1);
                if (!line.empty() && line.back() == '\r') line.pop_back();

                if (line == "PING") {
                    sendLine(sock, "PONG LIDAR-KOLIZE");
                } else if (line == "START") {
                    bool ok = startEvaluating();
                    sendLine(sock, ok ? "OK STARTED" : "ERR START");
                } else if (line == "STOP") {
                    stopEvaluating();
                    sendLine(sock, "OK STOPPED");
                } else if (line == "STATUS") {
                    sendLine(sock, getStatusJson());
                } else if (line == "COLLISIONS") {
                    std::string j = getLatestJson();
                    if (!j.empty()) {
                        sendLine(sock, j);
                    } else {
                        sendLine(sock, "ERR NOT_READY");
                    }
                } else if (line == "EXIT") {
                    sendLine(sock, "BYE");
                    ::shutdown(sock, SHUT_RDWR);
                    break;
                } else if (line == "SHUTDOWN") {
                    bool lidar_active = (lidar_ && lidar_->isRunning());
                    bool eval_active  = evaluating_.load();

                    if (lidar_active || eval_active) {
                        sendLine(sock, "ERR CANNOT_SHUTDOWN_WHILE_RUNNING (STOP services first)");
                    } else {
                        sendLine(sock, "SHUTTING DOWN");
                        if (shutdown_cb_) {
                            shutdown_cb_();
                        }
                        break;
                    }
                } else {
                    sendLine(sock, "ERR UNKNOWN COMMAND");
                }
            }
        }

        ::close(sock);
        {
            std::lock_guard<std::mutex> lg(clients_mtx_);
            client_socks_.erase(std::remove(client_socks_.begin(), client_socks_.end(), sock), client_socks_.end());
        }
    }

    // -------------------------------------------------------------
    // Vlákno odběru headingu ze služby fusion
    // -------------------------------------------------------------
    void headingSubLoop() {
        try {
            zmq::context_t ctx(1);
            zmq::socket_t sub(ctx, zmq::socket_type::sub);
            sub.set(zmq::sockopt::rcvtimeo, 200); // 200 ms timeout pro čisté ukončení
            sub.connect("ipc:///tmp/robot-fusion");
            sub.set(zmq::sockopt::subscribe, "SOLUTION");

            std::cout << "🧭 [LIDAR-KOLIZE] ZMQ SUB připojen na ipc:///tmp/robot-fusion (SOLUTION)" << std::endl;

            while (running_server_.load()) {
                zmq::message_t topic_msg;
                zmq::message_t payload_msg;

                auto res1 = sub.recv(topic_msg, zmq::recv_flags::none);
                if (!res1 || !running_server_.load()) continue;

                auto res2 = sub.recv(payload_msg, zmq::recv_flags::none);
                if (!res2) continue;

                std::string payload(static_cast<char*>(payload_msg.data()), payload_msg.size());
                parseHeadingJson(payload);
            }
        } catch (const std::exception &e) {
            std::cerr << "❌ [LIDAR-KOLIZE] Chyba v headingSubLoop: " << e.what() << std::endl;
        }
    }

    void parseHeadingJson(const std::string &json) {
        size_t hpos = json.find("\"heading\":");
        if (hpos == std::string::npos) return;

        size_t val_start = hpos + 10;
        while (val_start < json.size() && (json[val_start] == ' ' || json[val_start] == ':')) {
            val_start++;
        }
        size_t val_end = json.find_first_of(",}", val_start);
        if (val_end == std::string::npos) return;

        try {
            double h = std::stod(json.substr(val_start, val_end - val_start));
            auto now = std::chrono::steady_clock::now();

            std::lock_guard<std::mutex> lg(heading_mtx_);
            last_heading_start_ = latest_heading_;
            latest_heading_     = h;
            last_heading_time_  = now;
            has_heading_        = true;
        } catch (...) {}
    }

    // -------------------------------------------------------------
    // Hlavní vyhodnocovací smyčka kolizních kurzů (10 Hz / 100 ms)
    // -------------------------------------------------------------
    void evalLoop() {
        try {
            zmq::context_t ctx(1);
            zmq::socket_t pub(ctx, zmq::socket_type::pub);
            std::string ipc_path = "ipc:///tmp/robot-lidar-kolize";
            pub.bind(ipc_path);
            chmod("/tmp/robot-lidar-kolize", 0777);

            std::cout << "📡 [LIDAR-KOLIZE] ZMQ PUB běží na " << ipc_path << " (10 Hz)" << std::endl;

            std::vector<LidarPointBuffer::Sample> samples;
            samples.reserve(20000);

            auto next_tick = std::chrono::steady_clock::now();
            auto last_report_time = std::chrono::steady_clock::now();
            uint32_t report_cycles = 0;

            while (evaluating_.load()) {
                next_tick += std::chrono::milliseconds(config_.period_ms);
                std::this_thread::sleep_until(next_tick);

                cycles_total_++;
                report_cycles++;

                // 1. Kontrola běhu hardware LiDARu
                if (!lidar_ || !lidar_->isRunning()) {
                    state_.store(State::WAITING_FOR_DATA);
                    waiting_reason_ = "WAITING FOR LIDAR";
                    checkReport(last_report_time, report_cycles);
                    continue;
                }

                // 2. Kontrola dostupnosti headingu z fusion (nesmí být starší než 500 ms)
                double h_start, h_end, h_avg;
                {
                    std::lock_guard<std::mutex> lg(heading_mtx_);
                    auto now = std::chrono::steady_clock::now();
                    double age_s = std::chrono::duration<double>(now - last_heading_time_).count();

                    if (!has_heading_ || age_s > 0.5) {
                        state_.store(State::WAITING_FOR_DATA);
                        waiting_reason_ = "WAITING FOR HEADING";
                        checkReport(last_report_time, report_cycles);
                        continue;
                    }

                    h_start = last_heading_start_;
                    h_end   = latest_heading_;
                    h_avg   = (h_start + h_end) * 0.5;
                }

                // 3. Inkrementální načtení bodů z ring bufferu
                auto inc_res = lidar_->getPointBuffer().getNewSamples(last_seq_, samples);
                points_last_cycle_.store(inc_res.count);

                if (inc_res.count < config_.min_points_per_cycle) {
                    cycles_low_points_++;
                    state_.store(State::WAITING_FOR_DATA);
                    waiting_reason_ = "LOW LIDAR POINTS (" + std::to_string(inc_res.count) + ")";
                    checkReport(last_report_time, report_cycles);
                    continue;
                }

                // 4. Kontrola prodlevy (Lag > 150 ms)
                if (last_eval_timestamp_ > 0.0 && inc_res.min_time > 0.0) {
                    double dt = inc_res.min_time - last_eval_timestamp_;
                    if (dt > (config_.max_lag_ms / 1000.0) || inc_res.overflow) {
                        cycles_lag_dropped_++;
                        evaluator_.reset();
                    }
                }
                last_eval_timestamp_ = inc_res.max_time;

                // 5. Výpočet kolizních kurzů
                CollisionEvaluator::EvaluationResult res;
                double cur_ts = inc_res.max_time > 0.0 ? inc_res.max_time :
                    std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();

                bool ok = evaluator_.evaluate(samples, h_start, h_end, h_avg, cur_ts, res);
                if (!ok) {
                    state_.store(State::WAITING_FOR_DATA);
                    waiting_reason_ = "WAITING FOR 2ND FRAME";
                    checkReport(last_report_time, report_cycles);
                    continue;
                }

                // 6. Úspěšné vyhodnocení -> publikace ZMQ zprávy
                state_.store(State::RUNNING);
                waiting_reason_ = "OK";
                min_clearance_cm_.store(res.min_clearance_cm);
                min_ttc_s_.store(res.min_ttc_s);
                heading_avg_.store(res.heading_avg);
                last_eval_duration_ms_.store(res.duration_ms);

                {
                    std::lock_guard<std::mutex> lg(data_mtx_);
                    latest_json_ = res.json_payload;
                }

                std::string topic = "COLLISIONS";
                zmq::message_t topic_msg(topic.size());
                std::memcpy(topic_msg.data(), topic.data(), topic.size());

                zmq::message_t payload_msg(res.json_payload.size());
                std::memcpy(payload_msg.data(), res.json_payload.data(), res.json_payload.size());

                pub.send(topic_msg, zmq::send_flags::sndmore);
                pub.send(payload_msg, zmq::send_flags::none);

                cycles_published_++;

                checkReport(last_report_time, report_cycles);
            }
        } catch (const std::exception &e) {
            std::cerr << "❌ [LIDAR-KOLIZE] Chyba v evalLoop: " << e.what() << std::endl;
        }
    }

    void checkReport(std::chrono::steady_clock::time_point &last_report, uint32_t &cycles) {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - last_report).count();
        if (elapsed >= 1.0) {
            double rate = cycles / elapsed;
            std::string st = "IDLE";
            if (state_.load() == State::RUNNING) st = "RUNNING";
            else if (state_.load() == State::WAITING_FOR_DATA) st = "WAITING (" + waiting_reason_ + ")";

            std::cout << "🛡️  [LIDAR-KOLIZE] " << st
                      << " | " << std::fixed << std::setprecision(1) << rate << " Hz"
                      << " | Pts: " << points_last_cycle_.load()
                      << " | MinClearance: " << std::setprecision(1) << min_clearance_cm_.load() << " cm"
                      << " | MinTTC: " << std::setprecision(2) << min_ttc_s_.load() << " s"
                      << " | Pub: " << cycles_published_.load()
                      << " | LowPts: " << cycles_low_points_.load()
                      << " | Lag: " << cycles_lag_dropped_.load()
                      << std::endl;

            last_report = now;
            cycles = 0;
        }
    }

    // Reference a konfigurace
    LidarController *lidar_{nullptr};
    uint16_t port_{9222};
    std::string bind_addr_{"127.0.0.1"};
    CollisionConfig config_{};
    CollisionEvaluator evaluator_;

    // Řízení vláken a síť
    int server_fd_{-1};
    std::atomic<bool> running_server_{false};
    std::atomic<bool> evaluating_{false};
    std::thread tcp_thread_;
    std::thread heading_thread_;
    std::thread eval_thread_;
    std::function<void()> shutdown_cb_;

    std::mutex clients_mtx_;
    std::vector<int> client_socks_;

    // Heading stav
    mutable std::mutex heading_mtx_;
    double latest_heading_{0.0};
    double last_heading_start_{0.0};
    std::chrono::steady_clock::time_point last_heading_time_{};
    bool has_heading_{false};

    // Evaluátor stav a metriky
    std::mutex eval_mtx_;
    uint64_t last_seq_{0};
    double last_eval_timestamp_{0.0};

    std::atomic<State> state_{State::IDLE};
    std::string waiting_reason_{"IDLE"};

    std::atomic<uint64_t> cycles_total_{0};
    std::atomic<uint64_t> cycles_published_{0};
    std::atomic<uint64_t> cycles_low_points_{0};
    std::atomic<uint64_t> cycles_lag_dropped_{0};
    std::atomic<size_t>   points_last_cycle_{0};
    std::atomic<float>    min_clearance_cm_{2000.0f};
    std::atomic<float>    min_ttc_s_{999.0f};
    std::atomic<double>   heading_avg_{0.0};
    std::atomic<double>   last_eval_duration_ms_{0.0};

    mutable std::mutex data_mtx_;
    std::string latest_json_;
};
