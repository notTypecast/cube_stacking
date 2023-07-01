#include <control/PI_task.hpp>

PITask::PITask(pin::SE3 &target, double dt, double Kp, double Ki) : ControllerBase(target, dt), Kp(Kp), Ki(Ki), error_sum(Eigen::Vector6d::Zero()) {}

void PITask::set_target(pin::SE3 &target) {
    this->error_sum = Eigen::Vector6d::Zero();
    this->target = target;
    this->done = false;
}

Eigen::Matrix<double, 9, 1> PITask::update(std::shared_ptr<robot_dart::robots::Franka> &robot, pin::Model &model, pin::Data &data, RobotState &state) {
    auto position = robot->positions();
    Eigen::Matrix<double, 6, 9> J = Eigen::Matrix<double, 6, 9>::Zero();
    pin::computeFrameJacobian(model, data, position, state.EEF_FRAME_ID, pin::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);
    Eigen::Matrix<double, 6, 9> J2;
    J2.block(0, 0, 3, 9) = J.block(3, 0, 3, 9);
    J2.block(3, 0, 3, 9) = J.block(0, 0, 3, 9);

    Eigen::Vector6d error_wf = this->get_error(data.oMf[state.EEF_FRAME_ID]);
    this->error_sum += error_wf * this->dt;
    
    if (error_wf.norm() > this->error_threshold) {
        Eigen::Vector6d new_v = this->Kp * error_wf + this->Ki * this->error_sum;
        Eigen::Matrix<double, 9, 6> jac_pinv = damped_pseudoinverse(J2);
        return jac_pinv * new_v;
    }

    this->done = true;
    return Eigen::Matrix<double, 9, 1>::Zero();
}
