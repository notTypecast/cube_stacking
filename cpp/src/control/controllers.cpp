#include <control/controllers.hpp>

bool moveToPosition(std::shared_ptr<robot_dart::robots::Franka> &robot, pin::Model &model, pin::Data &data, ControllerBase &task, RobotState &state) {
    auto commands = task.update(robot, model, data, state);

    if (state.gripping) {
        commands[7] = robot->positions()[7] < 0.02 ? task.get_gripping_c1() : task.get_gripping_c2();
    }
    else if (robot->positions()[7] < 0.039) {
        commands[7] = 0.1;
    }

    robot->set_commands(commands);

    return task.is_done();
}

bool moveToGripPosition(std::shared_ptr<robot_dart::robots::Franka> &robot, pin::Model &model, pin::Data &data, ControllerBase &task, RobotState &state) {
    bool res = moveToPosition(robot, model, data, task, state);

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

bool moveToEndPosition(std::shared_ptr<robot_dart::robots::Franka> &robot, pin::Model &model, pin::Data &data, ControllerBase &task, RobotState &state) {
    if (state.move_state == -1) {
        state.move_state = 0;
        state.target = task.get_target();
        data.oMf[state.EEF_FRAME_ID].translation()[2] += 0.3;
        task.set_target(data.oMf[state.EEF_FRAME_ID]);
    }

    bool res = moveToPosition(robot, model, data, task, state);

    if (res) {
        if (state.move_state == 0) {
            state.move_state = 1;
            pin::SE3 new_pos = state.target;
            new_pos.translation()[2] += 0.2;
            task.set_target(new_pos);
        }
        else if (state.move_state == 1) {
            state.move_state = 2;
            // TODO: figure out issue with error on torque controller here
            state.error_threshold = 0.035; //ERROR_THRESHOLD_LOW;
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

bool closeGripper(std::shared_ptr<robot_dart::robots::Franka> &robot, pin::Model &model, pin::Data &data, ControllerBase &task, RobotState& state) {
    auto rest_commands = task.rest_commands(robot, model, data, state);
    rest_commands[7] = task.get_close_gripper_const();

    robot->set_commands(rest_commands);

    if (robot->positions()[7] < 0.0201) {
        state.gripping = true;
        return true;
    }

    return false;
}

bool openGripper(std::shared_ptr<robot_dart::robots::Franka> &robot, pin::Model &model, pin::Data &data, ControllerBase &task, RobotState& state) {
    auto rest_commands = task.rest_commands(robot, model, data, state);
    rest_commands[7] = -task.get_close_gripper_const()/2;

    robot->set_commands(rest_commands);

    if (robot->positions()[7] > 0.039) {
        robot->set_commands((Eigen::Matrix<double, 9, 1>() << 0, 0, 0, 0, 0, 0, 0, 0, 0).finished());
        state.gripping = false;
        state.moving = false;
        state.above = false;
        
        return true;
    }

    return false;
}
