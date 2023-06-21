#ifndef TREE_H
#define TREE_H

#include <memory>
#include <vector>
#include <functional>

namespace BehaviorTree {
    /**
     * Status of a node
     * SUCCESS: Node has completed successfully
     * FAILURE: Node has failed
     * RUNNING: Node is still running
     */
    enum Status {
        SUCCESS,
        FAILURE,
        RUNNING
    };

    /**
     * Base class for all nodes
     * Subclasses are abstract types of nodes
     * tick() must be implemented on any subclasses that are not abstract
     * reset() does not need to be implemented and does nothing by default
     */
    class Node {
        public:
            virtual Status tick() = 0;
            virtual void reset();
    };

    /**
     * Base class for root node
     * Can only have one child node
     * tick() ticks the child node
     */
    class Root {
        public:
            Root(const std::shared_ptr<Node>&);
            Status tick();
        private:
            std::shared_ptr<Node> child;
    };

    /**
     * Base class for control nodes
     * Can have multiple children
     * tick() ticks each child node
     * Can have memory
     */
    class ControlNode: public Node {
        public:
            ControlNode(const std::vector<std::shared_ptr<Node>>*, bool = true);
            void add_child(const std::shared_ptr<Node>&);
            void add_children(const std::vector<std::shared_ptr<Node>>&);
            virtual Status tick() = 0;
            void reset() override;
        protected:
            void reset_subtree();
            std::vector<std::shared_ptr<Node>> children;
            Status status;
            bool memory;
            int child_index;
    };

    /**
     * Base class for leaf nodes
     * Cannot have children
     * tick() returns SUCCESS, FAILURE, or RUNNING
     */
    class LeafNode: public Node {
        public:
            LeafNode(const std::function<Status()>);

        protected:
            std::function<Status()> function;
    };

    /**
     * Base class for sequence nodes
     * Implements the tick method
     * The tick method returns RUNNING or FAILURE if any child does the same, otherwise SUCCESS
     */
    class SequenceNode: public ControlNode {
        public:
            SequenceNode(const std::vector<std::shared_ptr<Node>>*, bool = true);
            Status tick() override;
    };

    /**
     * Base class for fallback nodes
     * Implements the tick method
     * The tick method returns RUNNING or SUCCESS if any child does the same, otherwise FAILURE
     */
    class FallbackNode: public ControlNode {
        public:
            FallbackNode(const std::vector<std::shared_ptr<Node>>*, bool = true);
            Status tick() override;
    };

    /**
     * Base class for action nodes
     * Implements the tick method and keeps a status
     * Will not run the function if the status is not RUNNING or null
     * The tick method returns the status returned by the function passed to the constructor
     */
    class ActionNode: public LeafNode {
        public:
            ActionNode(const std::function<Status()>);
            Status tick() override;
            void reset() override;

        protected:
            Status status;
    };

    /**
     * Base class for condition nodes
     * Implements the tick method and keeps a status
     * Will not run the function if the status is not RUNNING or null
     * The tick method returns the status returned by the function passed to the constructor
     */
    class ConditionNode: public LeafNode {
        public:
            ConditionNode(const std::function<Status()>);
            Status tick() override;
    };

    class InvalidTreeException: public std::exception {
        public:
            char* what();
    };

}
#endif // TREE_H
