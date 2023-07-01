#include <iostream>
#include <random>
#include <map>

#include <robot_dart/robot_dart_simu.hpp>
#include <robot_dart/robots/franka.hpp>

#ifdef GRAPHIC
#include <robot_dart/gui/magnum/graphics.hpp>
#endif

#include <dart/dynamics/BodyNode.hpp>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/fwd.hpp>

#include <cpp/utils.hpp>
#include <control/PI_task.cpp>
#include <control/PD_task_torque.cpp>
#include <robot_tree/robot_tree.cpp>

enum CONTROL_TYPE {
    SERVO,
    TORQUE,
    NONE
};

namespace pin = pinocchio;

int main(int argc, char** argv) {
    enum CONTROL_TYPE control_type = argc == 1 ? SERVO : (strcmp(argv[1], "servo") == 0 ? SERVO : (strcmp(argv[1], "torque") == 0 ? TORQUE : NONE));
    
    if (control_type == NONE) {
        std::cout << "Usage: ./cube_stacking [servo/torque]" << std::endl;
        return 1;
    }

    std::srand(std::time(0));

    double dt = 0.001;
    double simulation_time = 100.;
    int total_steps = static_cast<int>(std::ceil(simulation_time / dt));

    // Create robot_dart robot object (for simulation)
    auto robot = std::make_shared<robot_dart::robots::Franka>(1. / dt);

    // set initial joint positions
    auto position = robot_dart::make_vector({0., M_PI / 4., 0., -M_PI / 4., 0., M_PI / 2., 0., 0.04, 0.04});
    robot->set_positions(position);

    // Create pinocchio robot objects (for kinematics)
    pin::Model model;
    pin::urdf::buildModel("./src/urdf/franka.urdf", model);

    pin::Data data(model);

    pin::forwardKinematics(model, data, position);
    pin::SE3 desired(MAIN_R, Eigen::Vector3d(0.3, 0.3, 0.4));

    double max_force = 5.;
    robot->set_force_lower_limits(robot_dart::make_vector({-max_force, -max_force}), {"panda_finger_joint1", "panda_finger_joint2"});
    robot->set_force_upper_limits(robot_dart::make_vector({max_force, max_force}), {"panda_finger_joint1", "panda_finger_joint2"});

    robot->set_actuator_types(control_type == SERVO ? "servo" : "torque");

    // Create boxes
    auto box_positions = create_grid();

    Eigen::Vector3d box_size;
    box_size << 0.04, 0.04, 0.04;

    std::random_device rd; // only used once to initialise (seed) engine
    std::mt19937 rng(rd()); // random-number engine used (Mersenne-Twister in this case)
    std::uniform_int_distribution<int> box_dist(0, box_positions.size() - 1);

    Eigen::Vector6d box_pose;

    // Red Box
    // Random cube position
    int red_box_pt = box_dist(rng);
    box_pose << 0., 0., 0., box_positions[red_box_pt][0], box_positions[red_box_pt][1], box_size[2] / 2.0;
    auto red_box = robot_dart::Robot::create_box(box_size, box_pose, "free", 0.1, dart::Color::Red(1.), "red_box");

    // Green Box
    // Random cube position
    int green_box_pt = box_dist(rng);
    while (green_box_pt == red_box_pt)
        green_box_pt = box_dist(rng);
    box_pose << 0., 0., 0., box_positions[green_box_pt][0], box_positions[green_box_pt][1], box_size[2] / 2.0;
    auto green_box = robot_dart::Robot::create_box(box_size, box_pose, "free", 0.1, dart::Color::Green(1.), "green_box");

    // Blue Box
    // Random cube position
    int box_pt = box_dist(rng);
    while (box_pt == green_box_pt || box_pt == red_box_pt)
        box_pt = box_dist(rng);
    box_pose << 0., 0., 0., box_positions[box_pt][0], box_positions[box_pt][1], box_size[2] / 2.0;
    auto blue_box = robot_dart::Robot::create_box(box_size, box_pose, "free", 0.1, dart::Color::Blue(1.), "blue_box");

    // Choose problem
    auto problems = create_problems();

    std::uniform_int_distribution<int> prob_dist(0, problems.size() - 1);
    int problem_id = prob_dist(rng);
    auto problem = problems[problem_id];
    std::cout << "We want to put the " << problem[2] << " cube on top of the " << problem[1] << " and the " << problem[1] << " cube on top of the " << problem[0] << " cube." << std::endl;

    // Create graphics
#ifdef GRAPHIC
    robot_dart::gui::magnum::GraphicsConfiguration configuration;
    configuration.width = 1280; // you can change the resolution
    configuration.height = 960;
    auto graphics = std::make_shared<robot_dart::gui::magnum::Graphics>(configuration);
#endif

    // Create simulator object
    robot_dart::RobotDARTSimu simu(dt);
    simu.set_collision_detector("fcl"); // you can use bullet here
    simu.set_control_freq(100);
#ifdef GRAPHIC
    simu.set_graphics(graphics);
    graphics->look_at({0., 2.5, 1.5}, {0., 0., 0.25});
#endif
    simu.add_checkerboard_floor();
    simu.add_robot(robot);
    simu.add_robot(red_box);
    simu.add_robot(green_box);
    simu.add_robot(blue_box);

    std::vector<Eigen::Matrix<double, 6, 1>> total_pos;

    std::map<std::string, std::shared_ptr<robot_dart::Robot>> box_map;
    box_map["red"] = red_box;
    box_map["green"] = green_box;
    box_map["blue"] = blue_box;

    for (auto box: problem) {
        total_pos.push_back(box_map[box]->positions());
    }

    struct RobotState state;
    state.EEF_FRAME_ID = model.getFrameId("panda_hand");
    state.above = false;
    state.move_state = -1;
    state.gripping = false;
    state.moving = false;

    std::shared_ptr<BehaviorTree::Root> root;

    if (control_type == SERVO) {
        PITask controller(desired, dt);
        root = createBehaviorTree(total_pos, robot, model, data, controller, state);
    }
    else {
        PDTaskTorque controller(desired, dt);
        root = createBehaviorTree(total_pos, robot, model, data, controller, state);
    }

    for (int i = 0; i < total_steps; i++) {
        if (simu.schedule(simu.control_freq())) {
            root->tick();
        }

        if (simu.step_world()) {
            break;
        }
    }

    robot.reset();
    return 0;
}
