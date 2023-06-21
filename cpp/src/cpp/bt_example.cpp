#include <BehaviorTree/tree.cpp>

int main() {
    BehaviorTree::SequenceNode main_sequence(nullptr);

    BehaviorTree::SequenceNode sequence1(nullptr);

    BehaviorTree::ActionNode action1([]() {
        std::cout << "Action 1" << std::endl;
        return BehaviorTree::Status::SUCCESS;
    });

    BehaviorTree::ActionNode action2([]() {
        std::cout << "Action 2" << std::endl;
        return BehaviorTree::Status::SUCCESS;
    });

    sequence1.add_child(std::make_shared<BehaviorTree::ActionNode>(action1));
    sequence1.add_child(std::make_shared<BehaviorTree::ActionNode>(action2));

    main_sequence.add_child(std::make_shared<BehaviorTree::SequenceNode>(sequence1));

    BehaviorTree::FallbackNode fallback1(nullptr);

    BehaviorTree::ConditionNode condition1([]() {
        if (3 < 1) {
            return BehaviorTree::Status::SUCCESS;
        }
        else {
            return BehaviorTree::Status::FAILURE;
        }
    });

    BehaviorTree::ActionNode action3([]() {
        std::cout << "Action 3" << std::endl;
        return BehaviorTree::Status::FAILURE;
    });

    fallback1.add_child(std::make_shared<BehaviorTree::ConditionNode>(condition1));
    fallback1.add_child(std::make_shared<BehaviorTree::ActionNode>(action3));

    main_sequence.add_child(std::make_shared<BehaviorTree::FallbackNode>(fallback1));

    BehaviorTree::ActionNode action4([]() {
        std::cout << "Action 4" << std::endl;
        return BehaviorTree::Status::SUCCESS;
    });

    main_sequence.add_child(std::make_shared<BehaviorTree::ActionNode>(action4));

    BehaviorTree::Root root(std::make_shared<BehaviorTree::SequenceNode>(main_sequence));

    BehaviorTree::Status status;
    do {
        std::cout << "Running here..." << std::endl;
        status = root.tick();
    } while (status == BehaviorTree::Status::RUNNING);
}