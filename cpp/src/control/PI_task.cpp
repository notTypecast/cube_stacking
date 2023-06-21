#include <control/PI_task.hpp>

PITask::PITask(const pin::SE3 &target, double dt, double Kp, double Ki) {
    this->target = target;
    this->dt = dt;
    this->Kp = Kp;
    this->Ki = Ki;
    this->error_sum = Eigen::Vector6d::Zero();
    this->error_threshold = ERROR_THRESHOLD_LOW;
}

void PITask::set_target(const pin::SE3 &target) {
    this->error_sum = Eigen::Vector6d::Zero();
    this->target = target;
}

Eigen::Vector6d PITask::get_error(const pin::SE3 &tf) {
    Eigen::Vector3d error_rot = pin::log3(this->target.rotation() * tf.rotation().transpose());
    Eigen::Vector3d error_pos = this->target.translation() - tf.translation();

    Eigen::Vector6d error;
    error << error_rot, error_pos;

    return error;
}

Eigen::Matrix<double, 9, 1> PITask::update(const pin::SE3 &current, const Eigen::Matrix<double, 6, 9> &jacobian) {
    Eigen::Vector6d error_wf = this->get_error(current);
    this->error_sum += error_wf * this->dt;

    Eigen::Vector6d new_v = this->Kp * error_wf + this->Ki * this->error_sum;
    std::cout << "error_wf: " << error_wf.norm() << std::endl;
    if (error_wf.norm() > this->error_threshold) {
       Eigen::Matrix<double, 9, 6> jac_pinv = damped_pseudoinverse(jacobian);
       return jac_pinv * new_v;
    }

    return Eigen::Matrix<double, 9, 1>::Zero();
}
