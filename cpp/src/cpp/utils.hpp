#ifndef FRANKA_UTILS_HPP
#define FRANKA_UTILS_HPP

#include <Eigen/Core>
#include <math.h>

const Eigen::Matrix3d MAIN_R = (Eigen::Matrix3d() << 0, 1, 0,
                                                    1, 0, 0,
                                                    0, 0, -1).finished();

std::vector<Eigen::Vector2d> create_grid(double box_step_x = 0.05, double box_step_y = 0.05) {
    std::vector<Eigen::Vector2d> box_positions;
    double box_x_min = 0.3;
    double box_x_max = 0.7;
    double box_y_min = -0.4;
    double box_y_max = 0.4;

    int box_nx_steps = static_cast<int>(floor((box_x_max - box_x_min) / box_step_x));
    int box_ny_steps = static_cast<int>(floor((box_y_max - box_y_min) / box_step_y));

    for (int x = 0; x <= box_nx_steps; x++) {
        for (int y = 0; y <= box_ny_steps; y++) {
            double box_x = box_x_min + x * box_step_x;
            double box_y = box_y_min + y * box_step_y;
            Eigen::Vector2d box_pt;
            box_pt << box_x, box_y;
            box_positions.push_back(box_pt);
        }
    }

    return box_positions;
}

std::vector<std::vector<std::string>> create_problems() {
    std::vector<std::vector<std::string>> problems;

    std::vector<std::string> cubes = {"red", "green", "blue"};

    for (const auto& cubeA : cubes) {
        for (const auto& cubeB : cubes) {
            if (cubeA == cubeB)
                continue;
            for (const auto& cubeC : cubes) {
                if (cubeA == cubeC || cubeB == cubeC)
                    continue;
                problems.push_back({cubeA, cubeB, cubeC});
            }
        }
    }

    return problems;
}

Eigen::Matrix<double, 9, 6> damped_pseudoinverse(const Eigen::Matrix<double, 6, 9>& jacobian, double l = 0.01) {
    return jacobian.transpose() * (jacobian * jacobian.transpose() + l * l * Eigen::Matrix<double, 6, 6>::Identity()).inverse();
}

#endif // FRANKA_UTILS_HPP
