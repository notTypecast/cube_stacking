#include <control/PD_task_torque.hpp>

PDTaskTorque::PDTaskTorque(pin::SE3 &target, double dt, double Kp, double Kd, double Ki) : 
    ControllerBase(target, dt, -20.0f, -2.0f, -2.0f), 
    Kp(Kp), 
    Kd(Kd), 
    Ki(Ki), 
    error_sum(Eigen::Vector6d::Zero()) 
    {}

void PDTaskTorque::set_target(pin::SE3 &target) {
    this->error_sum = Eigen::Vector6d::Zero();
    this->target = target;
    this->done = false;
}

Eigen::Matrix<double, 9, 1> PDTaskTorque::update(std::shared_ptr<robot_dart::robots::Franka> &robot, pin::Model &model, pin::Data &data, RobotState &state) {
    auto position = robot->positions();
    Eigen::Matrix<double, 6, 9> J = Eigen::Matrix<double, 6, 9>::Zero();
    pin::computeFrameJacobian(model, data, position, state.EEF_FRAME_ID, pin::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);
    Eigen::Matrix<double, 6, 9> J2;
    J2.block(0, 0, 3, 9) = J.block(3, 0, 3, 9);
    J2.block(3, 0, 3, 9) = J.block(0, 0, 3, 9);

    Eigen::Vector6d error_wf = this->get_error(data.oMf[state.EEF_FRAME_ID]);
    this->error_sum += error_wf * this->dt;

    pin::nonLinearEffects(model, data, position, robot->velocities());
    
    if (error_wf.norm() > this->error_threshold) {
        Eigen::Vector6d tau = this->Kp * error_wf + Kd * (-robot->body_velocity(state.EEF_NAME)) + this->Ki * this->error_sum;

        Eigen::Matrix<double, 9, 1> controls = J2.transpose() * tau;
        controls[7] = 0;
        controls = controls + data.nle;
        controls[8] = 0;
        return controls;
    }

    this->done = true;
    
    Eigen::Matrix<double, 9, 1> controls = data.nle;
    controls[8] = 0;

    return controls;   
}

Eigen::Matrix<double, 9, 1> PDTaskTorque::rest_commands(std::shared_ptr<robot_dart::robots::Franka> &robot, pin::Model &model, pin::Data &data, RobotState &state) {
    auto position = robot->positions();
    pin::nonLinearEffects(model, data, position, robot->velocities());
    Eigen::Matrix<double, 9, 1> controls = data.nle;
    controls[8] = 0;
    return controls;
}