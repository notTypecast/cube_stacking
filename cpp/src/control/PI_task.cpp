#include <control/PI_task.hpp>

PITask::PITask(pin::SE3 &target, double dt, double Kp, double Ki) {
    this->target = target;
    this->dt = dt;
    this->Kp = Kp;
    this->Ki = Ki;
    this->error_sum = Eigen::Vector6d::Zero();
    this->error_threshold = ERROR_THRESHOLD_LOW;
}

void PITask::set_target(pin::SE3 &target) {
    this->error_sum = Eigen::Vector6d::Zero();
    this->target = target;
}

const pin::SE3& PITask::get_target() const {
    return this->target;
}

void PITask::set_threshold(double threshold) {
    this->error_threshold = threshold;
}

Eigen::Vector6d PITask::get_error(const pin::SE3 &tf) const {
    return calc_error(this->target, tf);
}

Eigen::Matrix<double, 9, 1> PITask::update(pin::SE3 &current, Eigen::Matrix<double, 6, 9> &jacobian) {
    Eigen::Vector6d error_wf = this->get_error(current);
    this->error_sum += error_wf * this->dt;

    Eigen::Vector6d new_v = this->Kp * error_wf + this->Ki * this->error_sum;
    
    if (error_wf.norm() > this->error_threshold) {
       Eigen::Matrix<double, 9, 6> jac_pinv = damped_pseudoinverse(jacobian);
       return jac_pinv * new_v;
    }

    return Eigen::Matrix<double, 9, 1>::Zero();
}
