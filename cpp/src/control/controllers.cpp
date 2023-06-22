#include <control/controllers.hpp>

bool moveToPosition(std::shared_ptr<robot_dart::robots::Franka> &robot, PITask &task, RobotState &state, pin::SE3 &current, Eigen::Matrix<double, 6, 9> &jacobian, double gripper_pos) {
    auto commands = task.update(current, jacobian);

    if (commands.isZero(0)) {
        return true;
    }

    if (state.gripping) {
        commands[7] = gripper_pos < 0.02 ? 0.0 : -0.1;
    }
    else if (gripper_pos < 0.039) {
        commands[7] = 0.1;
    }

    robot->set_commands(commands);

    return false;
}

bool moveToGripPosition(std::shared_ptr<robot_dart::robots::Franka> &robot, PITask &task, RobotState &state, pin::SE3 &current, Eigen::Matrix<double, 6, 9> &jacobian, double gripper_pos) {
    bool res = moveToPosition(robot, task, state, current, jacobian, gripper_pos);

    if (res) {
        if (!state.above) {
            state.error_threshold = ERROR_THRESHOLD_LOW;
            task.set_threshold(state.error_threshold);
            pin::SE3 target = task.get_target();
            target.translation()[2] -= 0.3;
            state.above = true;
            task.set_target(target);
            return false;
        }
        else {
            state.above = false;
            return true;
        }
    }

    return false;
}

bool moveToEndPosition(std::shared_ptr<robot_dart::robots::Franka> &robot, PITask &task, RobotState &state, pin::SE3 &current, Eigen::Matrix<double, 6, 9> &jacobian, double gripper_pos) {
    if (state.move_state == -1) {
        state.move_state = 0;
        state.target = task.get_target();
        // TODO: make sure editing current is fine
        current.translation()[2] += 0.3;
        task.set_target(current);
    }

    bool res = moveToPosition(robot, task, state, current, jacobian, gripper_pos);

    if (res) {
        if (state.move_state == 0) {
            state.move_state = 1;
            pin::SE3 new_pos = state.target;
            new_pos.translation()[2] += 0.2;
            task.set_target(new_pos);
        }
        else if (state.move_state == 1) {
            state.move_state = 2;
            state.error_threshold = ERROR_THRESHOLD_LOW;
            task.set_threshold(state.error_threshold);
            task.set_target(state.target);
        }
        else {
            state.move_state = -1;
            return true;
        }
    }

    return false;
}

bool closeGripper(std::shared_ptr<robot_dart::robots::Franka> &robot, RobotState& state, double gripper_pos) {
    robot->set_commands((Eigen::Matrix<double, 9, 1>() << 0, 0, 0, 0, 0, 0, 0, -0.1, 0).finished());

    if (gripper_pos < 0.0201) {
        state.gripping = true;
        return true;
    }

    return false;
}

bool openGripper(std::shared_ptr<robot_dart::robots::Franka> &robot, RobotState& state, double gripper_pos) {
    robot->set_commands((Eigen::Matrix<double, 9, 1>() << 0, 0, 0, 0, 0, 0, 0, 0.05, 0).finished());

    if (gripper_pos > 0.039) {
        robot->set_commands((Eigen::Matrix<double, 9, 1>() << 0, 0, 0, 0, 0, 0, 0, 0, 0).finished());
        state.gripping = false;
        state.moving = false;
        state.above = false;
        
        return true;
    }

    return false;
}
