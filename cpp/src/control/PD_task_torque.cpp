#include <control/PD_task_torque.hpp>

PDTaskTorque::PDTaskTorque(pin::SE3 &target, double dt, double Kp, double Kd) : ControllerBase(target, dt), Kp(Kp), Kd(Kd) {}

Eigen::Matrix<double, 9, 1> PDTaskTorque::update(std::shared_ptr<robot_dart::robots::Franka> &robot, pin::Model &model, pin::Data &data, RobotState &state) {
    auto position = robot->positions();
    Eigen::Matrix<double, 6, 9> J = Eigen::Matrix<double, 6, 9>::Zero();
    pin::computeFrameJacobian(model, data, position, state.EEF_FRAME_ID, pin::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);
    Eigen::Matrix<double, 6, 9> J2;
    J2.block(0, 0, 3, 9) = J.block(3, 0, 3, 9);
    J2.block(3, 0, 3, 9) = J.block(0, 0, 3, 9);

    Eigen::Vector6d error_wf = this->get_error(data.oMf[state.EEF_FRAME_ID]);

    pin::nonLinearEffects(model, data, position, robot->velocities());

    if (error_wf.norm() > this->error_threshold) {
        // add velocity error here
        Eigen::Vector6d new_v = this->Kp * error_wf;

        //Eigen::Matrix<double, 9, 6> jac_pinv = damped_pseudoinverse(J2);
        Eigen::Matrix<double, 9, 1> controls = J2.transpose() * new_v;
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