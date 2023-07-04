#ifndef PD_TASK_TORQUE_H
#define PD_TASK_TORQUE_H

#ifndef CONTROLLER_BASE_H
#include <control/ControllerBase.cpp>
#endif

#include <pinocchio/algorithm/rnea.hpp>


class PDTaskTorque : public ControllerBase {
    public:
        PDTaskTorque(pin::SE3&, double, double = 25, double = 25, double = 0.005);
        void set_target(pin::SE3&) override;
        Eigen::Matrix<double, 9, 1> update(std::shared_ptr<robot_dart::robots::Franka>&, pin::Model&, pin::Data&, RobotState&) override;
        Eigen::Matrix<double, 9, 1> rest_commands(std::shared_ptr<robot_dart::robots::Franka>&, pin::Model&, pin::Data&, RobotState&) override;

    protected:
        double Kp;
        double Kd;
        double Ki;
        Eigen::Vector6d error_sum;
};


#endif // PD_TASK_TORQUE_H