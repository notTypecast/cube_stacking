#ifndef CONTROLLER_BASE_H
#define CONTROLLER_BASE_H

#include <Eigen/Core>
#include <cpp/utils.hpp>

#include <robot_dart/robots/franka.hpp>
#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

#define ERROR_THRESHOLD_LOW 0.005
#define ERROR_THRESHOLD_HIGH 0.1

namespace pin = pinocchio;

struct RobotState {
    std::string EEF_NAME;
    int EEF_FRAME_ID;
    bool above;
    short move_state;
    pin::SE3 target;
    bool gripping;
    bool moving;
    double error_threshold;
};

class ControllerBase {
    public:
        ControllerBase(pin::SE3&, double, float, float, float);
        virtual void set_target(pin::SE3&);
        const pin::SE3& get_target() const;
        void set_threshold(double);
        Eigen::Vector6d get_error(const pin::SE3&) const;
        virtual Eigen::Matrix<double, 9, 1> update(std::shared_ptr<robot_dart::robots::Franka>&, pin::Model&, pin::Data&, RobotState&) = 0;
        virtual Eigen::Matrix<double, 9, 1> rest_commands(std::shared_ptr<robot_dart::robots::Franka>&, pin::Model&, pin::Data&, RobotState&) = 0;
        bool is_done();
        float get_close_gripper_const();
        float get_gripping_c1();
        float get_gripping_c2();

    protected:
        pin::SE3 target;
        double dt;
        double error_threshold;
        bool done;
        float close_gripper_const;
        float gripping_c1;
        float gripping_c2;

};

#endif // CONTROLLER_BASE_H