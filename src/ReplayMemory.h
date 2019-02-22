#pragma once

#include <Eigen/Core>


struct sample {

    // Samples.
    Eigen::MatrixXf state;
    Eigen::MatrixXf action;
    Eigen::MatrixXf reward;
    Eigen::MatrixXf next_state;
    Eigen::MatrixXf done;

    // Operators to check for initialization.
	bool operator !() const {

		return (this->state.size()      == 0 || 
                this->action.size()     == 0 ||
                this->reward.size()     == 0 ||
                this->next_state.size() == 0 ||
                this->done.size()       == 0);
	};

    explicit operator bool() const {

        return (this->state.size()      != 0 && 
                this->action.size()     != 0 &&
                this->reward.size()     != 0 &&
                this->next_state.size() != 0 &&
                this->done.size()       != 0);
    }
};


// Deque of samples.
using memory = std::vector<sample>;


class ReplayMemory {

public:

    ReplayMemory() : rd_(), re_(rd_()) {    };

    auto Add(sample& sample) -> void {

        if (Length() <= memory_.max_size()) {
            memory_.push_back(sample);
        }
        else {
            printf("ReplayMemory -- deque reached maximum size.\n");
        }
    };

    auto Sample() const -> sample {

        // Shuffle memory.
        std::shuffle(memory_.begin(), memory_.end(), re_);

        // Return single states as one matrix.
        uint n = Length();

        Eigen::MatrixXf states(memory_[0].state.cols(), n);
        Eigen::MatrixXf actions(memory_[0].action.cols(), n);
        Eigen::MatrixXf rewards(memory_[0].reward.cols(), n);
        Eigen::MatrixXf next_states(memory_[0].next_state.cols(), n);
        Eigen::MatrixXf dones(memory_[0].done.cols(), n);

        for (uint i = 0; i < n; i++) {

            states.col(i) = memory_[i].state;
            actions.col(i) = memory_[i].action;
            rewards.col(i) = memory_[i].reward;
            next_states.col(i) = memory_[i].next_state;
            dones.col(i) = memory_[i].done;
        }

        return {states, actions, rewards, next_states, dones};
    };

    auto Flush() -> void {

        memory_.clear();
    };

    auto Length() const -> uint {

        return memory_.size();
    };

private:

    // Random engine.
    std::random_device rd_;
    std::mt19937 re_;

    // Memory.
    memory memory_;
};
