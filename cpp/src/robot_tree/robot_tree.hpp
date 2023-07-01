#ifndef ROBOT_TREE_H
#define ROBOT_TREE_H

#include <Eigen/Core>
#include <string.h>
#include <robot_dart/robots/franka.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <cpp/utils.hpp>
#include <BehaviorTree/tree.cpp>
#include <control/controllers.cpp>

namespace pin = pinocchio;

std::shared_ptr<BehaviorTree::FallbackNode> createBoxFallback(const Eigen::Matrix<double, 6, 1>&, int, std::shared_ptr<robot_dart::robots::Franka>&, pin::Model&, pin::Data&, ControllerBase&, RobotState&);
std::shared_ptr<BehaviorTree::Root> createBehaviorTree(const std::vector<Eigen::Matrix<double, 6, 1>>&, std::shared_ptr<robot_dart::robots::Franka>&, pin::Model&, pin::Data&, ControllerBase&, RobotState&);

#endif // ROBOT_TREE_H
