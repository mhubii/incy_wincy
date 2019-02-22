#pragma once

#include <deque>
#include <random>

#include <torch/torch.h>
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

        if (Length() <= deque_.max_size()) {
            deque_.push_back(sample);
        }
        else {
            printf("ReplayMemory -- deque reached maximum size.\n");
        }
    };

    auto Sample() const -> sample {

        // Shuffle memory.
        std::shuffle(deque_.begin(), deque_.end(), re_);

        // Return single states as one matrix.
        uint n = Length();

        Eigen::MatrixXf states(deque_[0].state.cols(), n);
        Eigen::MatrixXf actions(deque_[0].action.cols(), n);
        Eigen::MatrixXf rewards(deque_[0].reward.cols(), n);
        Eigen::MatrixXf next_states(deque_[0].next_state.cols(), n);
        Eigen::MatrixXf dones(deque_[0].done.cols(), n);

        for (uint i = 0; i < n; i++) {

            states.col(i) = deque_[i].state;
            actions.col(i) = deque_[i].action;
            rewards.col(i) = deque_[i].reward;
            next_states.col(i) = deque_[i].next_state;
            dones.col(i) = deque_[i].done;
        }

        return {states, actions, rewards, next_states, dones};
    };

    auto Flush() -> void {

        deque_.clear();
    };

    auto Length() const -> uint {

        return deque_.size();
    };

private:

    // Random engine.
    std::random_device rd_;
    std::mt19937 re_;

    // Memory.
    memory deque_;
};
