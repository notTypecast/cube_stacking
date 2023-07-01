#ifndef PD_TASK_TORQUE_H
#define PD_TASK_TORQUE_H

#ifndef CONTROLLER_BASE_H
#include <control/ControllerBase.cpp>
#endif

#include <pinocchio/algorithm/rnea.hpp>


class PDTaskTorque : public ControllerBase {
    public:
        PDTaskTorque(pin::SE3&, double, double = 2, double = 0.01);
        Eigen::Matrix<double, 9, 1> update(std::shared_ptr<robot_dart::robots::Franka>&, pin::Model&, pin::Data&, RobotState&) override;

    protected:
        double Kp;
        double Kd;
};


#endif // PD_TASK_TORQUE_H