#ifndef CONTROLLERS_H
#define CONTROLLERS_H

#include <Eigen/Core>
#include <pinocchio/fwd.hpp>
#include <robot_dart/robots/franka.hpp>

#include <control/PI_task.hpp>

namespace pin = pinocchio;

struct RobotState {
    int EEF_FRAME_ID;
    bool above;
    short move_state;
    pin::SE3 target;
    bool gripping;
    bool moving;
    double error_threshold;
};

bool moveToPosition(std::shared_ptr<robot_dart::robots::Franka>&, PITask&, RobotState&, pin::SE3&, Eigen::Matrix<double, 6, 9>&, double);
bool moveToGripPosition(std::shared_ptr<robot_dart::robots::Franka>&, PITask&, RobotState&, pin::SE3&, Eigen::Matrix<double, 6, 9>&, double);
bool moveToEndPosition(std::shared_ptr<robot_dart::robots::Franka>&, PITask&, RobotState&, pin::SE3&, Eigen::Matrix<double, 6, 9>&, double);
bool closeGripper(std::shared_ptr<robot_dart::robots::Franka>&, RobotState&, double);
bool openGripper(std::shared_ptr<robot_dart::robots::Franka>&, RobotState&, double);

#endif // CONTROLLERS_H