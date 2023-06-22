#ifndef PI_TASK_H
#define PI_TASK_H

#include <Eigen/Core>
#include <cpp/utils.hpp>

#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#define ERROR_THRESHOLD_LOW 0.005
#define ERROR_THRESHOLD_HIGH 0.1

namespace pin = pinocchio;

class PITask {
    public:
        PITask(pin::SE3&, double, double = 2, double = 0.005);
        void set_target(pin::SE3&);
        const pin::SE3& get_target() const;
        void set_threshold(double);
        Eigen::Vector6d get_error(const pin::SE3&) const;
        Eigen::Matrix<double, 9, 1> update(pin::SE3&, Eigen::Matrix<double, 6, 9>&);

    private:
        pin::SE3 target;
        double dt;
        double Kp;
        double Ki;
        Eigen::Vector6d error_sum;
        double error_threshold;
};

#endif // PI_TASK_H