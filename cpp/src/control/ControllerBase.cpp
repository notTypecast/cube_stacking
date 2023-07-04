#include <control/ControllerBase.hpp>

ControllerBase::ControllerBase(pin::SE3 &target, double dt, float close_gripper_const, float gripping_c1, float gripping_c2) : 
    target(target), 
    dt(dt), 
    error_threshold(ERROR_THRESHOLD_LOW), 
    done(false), 
    close_gripper_const(close_gripper_const), 
    gripping_c1(gripping_c1), 
    gripping_c2(gripping_c2) 
    {}

void ControllerBase::set_target(pin::SE3 &target) {
    this->target = target;
    this->done = false;
}

const pin::SE3& ControllerBase::get_target() const {
    return this->target;
}

void ControllerBase::set_threshold(double threshold) {
    this->error_threshold = threshold;
}

Eigen::Vector6d ControllerBase::get_error(const pin::SE3 &tf) const {
    return calc_error(this->target, tf);
}

bool ControllerBase::is_done() {
    return this->done;
}

float ControllerBase::get_close_gripper_const() {
    return this->close_gripper_const;
}

float ControllerBase::get_gripping_c1() {
    return this->gripping_c1;
}

float ControllerBase::get_gripping_c2() {
    return this->gripping_c2;
}
