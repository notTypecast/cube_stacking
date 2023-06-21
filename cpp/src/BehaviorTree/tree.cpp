#include <BehaviorTree/tree.hpp>

BehaviorTree::Root::Root(const std::shared_ptr<Node> &child) {
    this->child = child;
}

BehaviorTree::Status BehaviorTree::Root::tick() {
    return this->child->tick();
}

void BehaviorTree::Node::reset() {}

BehaviorTree::ControlNode::ControlNode(const std::vector<std::shared_ptr<Node>> *children, bool memory) {
    if (children == NULL) {
        this->children = std::vector<std::shared_ptr<Node>>();
    }
    else {
        this->children = std::vector<std::shared_ptr<Node>>(*children);
    }
    this->memory = memory;
    this->status = BehaviorTree::Status::RUNNING;
    this->child_index = 0;
}

void BehaviorTree::ControlNode::add_child(const std::shared_ptr<Node> &child) {
    this->children.push_back(child);
}

void BehaviorTree::ControlNode::add_children(const std::vector<std::shared_ptr<Node>> &children) {
    this->children.insert(this->children.end(), children.begin(), children.end());
}

void BehaviorTree::ControlNode::reset_subtree() {
    for (auto &child: this->children) {
        child->reset();
    }
    this->reset();
}

void BehaviorTree::ControlNode::reset() {
    this->child_index = 0;
    this->status = BehaviorTree::Status::RUNNING;
}

BehaviorTree::SequenceNode::SequenceNode(const std::vector<std::shared_ptr<Node>> *children, bool memory): ControlNode(children, memory) {}

BehaviorTree::Status BehaviorTree::SequenceNode::tick() {
    if (this->children.size() == 0) {
        throw BehaviorTree::InvalidTreeException();
    }

    if (this->status != BehaviorTree::Status::RUNNING) {
        return this->status;
    }

    int start = this->memory ? this->child_index : 0;
    for (size_t i = start; i < this->children.size(); ++i) {
        BehaviorTree::Status status = this->children[i]->tick();

        if (this->memory && status == BehaviorTree::Status::RUNNING) {
            this->child_index = i;
        }

        if (status != BehaviorTree::Status::SUCCESS) {
            this->status = status;
            return status;
        }
    }

    this->status = BehaviorTree::Status::SUCCESS;
    return this->status;
}

BehaviorTree::FallbackNode::FallbackNode(const std::vector<std::shared_ptr<Node>> *children, bool memory): ControlNode(children, memory) {}

BehaviorTree::Status BehaviorTree::FallbackNode::tick() {
    if (this->children.size() == 0) {
        throw BehaviorTree::InvalidTreeException();
    }

    if (this->status != BehaviorTree::Status::RUNNING) {
        return this->status;
    }

    int start = this->memory ? this->child_index : 0;
    for (size_t i = start; i < this->children.size(); ++i) {
        BehaviorTree::Status status = this->children[i]->tick();

        if (this->memory && status == BehaviorTree::Status::RUNNING) {
            this->child_index = i;
        }

        if (status != BehaviorTree::Status::FAILURE) {
            this->status = status;
            return status;
        }
    }

    this->status = BehaviorTree::Status::FAILURE;
    return this->status;
}

BehaviorTree::LeafNode::LeafNode(const std::function<BehaviorTree::Status()> function): function(function) {}

BehaviorTree::ActionNode::ActionNode(const std::function<BehaviorTree::Status()> function): LeafNode(function), status(BehaviorTree::Status::RUNNING) {}

void BehaviorTree::ActionNode::reset() {
    this->status = BehaviorTree::Status::RUNNING;
}

BehaviorTree::Status BehaviorTree::ActionNode::tick() {
    if (this->status != BehaviorTree::Status::RUNNING) {
        return this->status;
    }

    this->status = this->function();
    return this->status;
}

BehaviorTree::ConditionNode::ConditionNode(const std::function<BehaviorTree::Status()> function): LeafNode(function) {}

BehaviorTree::Status BehaviorTree::ConditionNode::tick() {
    return this->function();
}

char* BehaviorTree::InvalidTreeException::what() {
    return (char*)"Invalid tree";
}
