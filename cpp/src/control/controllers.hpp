#ifndef CONTROLLERS_H
#define CONTROLLERS_H

#include <Eigen/Core>
#include <pinocchio/fwd.hpp>
#include <robot_dart/robots/franka.hpp>

#include <control/ControllerBase.hpp>

bool moveToPosition(std::shared_ptr<robot_dart::robots::Franka>&, pin::Model&, pin::Data&, ControllerBase&, RobotState&);
bool moveToGripPosition(std::shared_ptr<robot_dart::robots::Franka>&, pin::Model&, pin::Data&, ControllerBase&, RobotState&);
bool moveToEndPosition(std::shared_ptr<robot_dart::robots::Franka>&, pin::Model&, pin::Data&, ControllerBase&, RobotState&);
bool closeGripper(std::shared_ptr<robot_dart::robots::Franka>&, RobotState&);
bool openGripper(std::shared_ptr<robot_dart::robots::Franka>&, RobotState&);

#endif // CONTROLLERS_H