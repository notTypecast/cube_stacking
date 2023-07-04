#include <robot_tree/robot_tree.hpp>

std::shared_ptr<BehaviorTree::FallbackNode> createBoxFallback(const Eigen::Matrix<double, 6, 1> &box_pos, int box_idx, std::shared_ptr<robot_dart::robots::Franka> &robot, pin::Model &model, pin::Data &data, ControllerBase &task, RobotState &state) {
    auto main_fb_0 = std::make_shared<BehaviorTree::FallbackNode>(nullptr);
    
    auto pos_correct_cond = std::make_shared<BehaviorTree::ConditionNode>([box_pos]() {
        if (abs(box_pos[3] - STACK_POS[0]) < 0.05 && abs(box_pos[4] - STACK_POS[1]) < 0.05) {
            return BehaviorTree::Status::SUCCESS;
        }
        else {
            return BehaviorTree::Status::FAILURE;
        }
    });

    auto move_box_to_stack_seq = std::make_shared<BehaviorTree::SequenceNode>(nullptr, false);

    auto move_to_grab_pos_fb = std::make_shared<BehaviorTree::FallbackNode>(nullptr);
    auto gripping_cond = std::make_shared<BehaviorTree::ConditionNode>([state]() {
        return (state.gripping ? BehaviorTree::Status::SUCCESS : BehaviorTree::Status::FAILURE);
    });
    auto move_grab_seq = std::make_shared<BehaviorTree::SequenceNode>(nullptr, false);

    move_to_grab_pos_fb->add_child(gripping_cond);
    move_to_grab_pos_fb->add_child(move_grab_seq);

    auto current_pos_fb = std::make_shared<BehaviorTree::FallbackNode>(nullptr);
    auto close_to_pos_cond = std::make_shared<BehaviorTree::ConditionNode>([box_pos, model, &data, &state]() mutable {
        pin::SE3 target = get_tf_above_box(box_pos);
        pin::updateFramePlacement(model, data, (pin::FrameIndex)state.EEF_FRAME_ID);
        pin::SE3 current = data.oMf[state.EEF_FRAME_ID];

        if (calc_error(target, current).norm() < state.error_threshold) {
            state.moving = false;
            state.above = false;
            return BehaviorTree::Status::SUCCESS;
        }
        else {
            return BehaviorTree::Status::FAILURE;
        }  
    });
    auto move_to_pos_action = std::make_shared<BehaviorTree::ActionNode>([box_pos, robot, model, &data, &task, &state]() mutable {
        if (!state.moving) {
            state.error_threshold = ERROR_THRESHOLD_HIGH;
            task.set_threshold(state.error_threshold);
            pin::SE3 target = get_tf_above_box(box_pos, true);
            task.set_target(target);

            state.moving = true;
            state.above = false;
        }

        bool res = moveToGripPosition(robot, model, data, task, state);

        if (res) {
            robot->set_commands((Eigen::VectorXd(9) << 0, 0, 0, 0, 0, 0, 0, 0, 0).finished());
            state.moving = false;
            state.above = false;
            return BehaviorTree::Status::SUCCESS;
        }

        return BehaviorTree::Status::RUNNING;

    });
    current_pos_fb->add_child(close_to_pos_cond);
    current_pos_fb->add_child(move_to_pos_action);

    auto grab_action = std::make_shared<BehaviorTree::ActionNode>([robot, model, &data, &task, &state]() mutable {
        bool res = closeGripper(robot, model, data, task, state);

        if (res) {
            return BehaviorTree::Status::SUCCESS;
        }

        return BehaviorTree::Status::RUNNING;
    });

    move_grab_seq->add_child(current_pos_fb);
    move_grab_seq->add_child(grab_action);

    auto stack_pos_fb = std::make_shared<BehaviorTree::FallbackNode>(nullptr);
    auto in_stack_pos_cond = std::make_shared<BehaviorTree::ConditionNode>([box_idx, model, &data, &state]() mutable {
        pin::updateFramePlacement(model, data, (pin::FrameIndex)state.EEF_FRAME_ID);
        pin::SE3 current = data.oMf[state.EEF_FRAME_ID];
        pin::SE3 target = pin::SE3(MAIN_R, Eigen::Vector3d(STACK_POS[0], STACK_POS[1], 0.15 + box_idx*0.04));

        if (calc_error(target, current).norm() < state.error_threshold) {
            return BehaviorTree::Status::SUCCESS;
        }
        else {
            return BehaviorTree::Status::FAILURE;
        }
        
    });
    auto move_to_stack_action = std::make_shared<BehaviorTree::ActionNode>([box_idx, robot, model, &data, &task, &state]() mutable {
        if (!state.moving) {
            state.error_threshold = ERROR_THRESHOLD_HIGH;
            task.set_threshold(state.error_threshold);
            pin::SE3 target = pin::SE3(MAIN_R, Eigen::Vector3d(STACK_POS[0], STACK_POS[1], 0.15 + box_idx*0.04));
            task.set_target(target);
            state.moving = true;
            state.above = false;
        }

        bool res = moveToEndPosition(robot, model, data, task, state);

        if (res) {
            state.moving = false;
            state.above = false;
            return BehaviorTree::Status::SUCCESS;
        }

        return BehaviorTree::Status::RUNNING;
    });
    stack_pos_fb->add_child(in_stack_pos_cond);
    stack_pos_fb->add_child(move_to_stack_action);

    auto release_gripper_action = std::make_shared<BehaviorTree::ActionNode>([robot, model, &data, &task, &state]() mutable {
        bool res = openGripper(robot, model, data, task, state);

        if (res) {
            return BehaviorTree::Status::SUCCESS;
        }

        return BehaviorTree::Status::RUNNING;
    });
    
    move_box_to_stack_seq->add_child(move_to_grab_pos_fb);
    move_box_to_stack_seq->add_child(stack_pos_fb);
    move_box_to_stack_seq->add_child(release_gripper_action);

    main_fb_0->add_child(pos_correct_cond);
    main_fb_0->add_child(move_box_to_stack_seq);

    return main_fb_0;
}



std::shared_ptr<BehaviorTree::Root> createBehaviorTree(const std::vector<Eigen::Matrix<double, 6, 1>> &box_positions, std::shared_ptr<robot_dart::robots::Franka> &robot, pin::Model &model, pin::Data &data, ControllerBase &task, RobotState &state) {
    auto main_seq = std::make_shared<BehaviorTree::SequenceNode>(nullptr, false);

    for (int i = 0; i < 3; ++i) {
        auto fb = createBoxFallback(box_positions.at(i), i, robot, model, data, task, state);
        main_seq->add_child(fb);
    }

    auto rest_action = std::make_shared<BehaviorTree::ActionNode>([robot, model, &data, &task, &state]() mutable {
        auto commands = task.rest_commands(robot, model, data, state);

        robot->set_commands(commands);

        return BehaviorTree::Status::RUNNING;
    });

    main_seq->add_child(rest_action);

    auto root = std::make_shared<BehaviorTree::Root>(main_seq);
    return root;
}