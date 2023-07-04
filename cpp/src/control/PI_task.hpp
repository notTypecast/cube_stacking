#ifndef PI_TASK_H
#define PI_TASK_H

#ifndef CONTROLLER_BASE_H
#include <control/ControllerBase.cpp>
#endif


class PITask : public ControllerBase {
    public:
        PITask(pin::SE3&, double, double = 2, double = 0.005);
        void set_target(pin::SE3&) override;
        Eigen::Matrix<double, 9, 1> update(std::shared_ptr<robot_dart::robots::Franka>&, pin::Model&, pin::Data&, RobotState&) override;
        Eigen::Matrix<double, 9, 1> rest_commands(std::shared_ptr<robot_dart::robots::Franka>&, pin::Model&, pin::Data&, RobotState&) override;

    protected:
        double Kp;
        double Ki;
        Eigen::Vector6d error_sum;
};

#endif // PI_TASK_H