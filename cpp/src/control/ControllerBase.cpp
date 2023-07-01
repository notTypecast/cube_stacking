#include <control/ControllerBase.hpp>

ControllerBase::ControllerBase(pin::SE3 &target, double dt) : target(target), dt(dt), error_threshold(ERROR_THRESHOLD_LOW), done(false) {}

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
