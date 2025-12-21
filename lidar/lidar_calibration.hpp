#pragma once

#include <Eigen/Dense>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <limits>
#include <cmath>

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
