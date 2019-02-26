#pragma once

#include <torch/torch.h>
#include <vector>

// Markov decision.
using MD = std::tuple<torch::Tensor /*state*/,
                      torch::Tensor /*action*/,
                      torch::Tensor /*reward*/,
                      torch::Tensor /*next_state*/,
                      torch::Tensor /*dones*/>;

// Markov decision process.
using MDP = std::vector<MD>;

class ReplayMemory
{
private:
    uint steps_;
    uint step_;

    MDP mdp_;

    // Random engine for shuffling memory.
    std::random_device rd_;
    std::mt19937 re_;

public:
    ReplayMemory(uint steps, torch::ArrayRef<int64_t> state_shape, torch::ArrayRef<int64_t> action_shape);

    auto insert(MD md) -> void;
    auto gather() -> MDP;
    auto to(torch::Device dev) -> void;
};

ReplayMemory::ReplayMemory(uint steps, torch::ArrayRef<int64_t> state_shape, torch::ArrayRef<int64_t> action_shape)
    : steps_(steps),
      step_(0),

      mdp_(steps, std::make_tuple(torch::zeros(state_shape),
                                  torch::zeros(action_shape),
                                  torch::zeros({1}),
                                  torch::zeros(state_shape),
                                  torch::zeros({1}))),

      rd_(),
      re_(rd_())
{
}

auto ReplayMemory::insert(MD md) -> void
{
    std::get<0>(mdp_[step_]).copy_(std::get<0>(md));
    std::get<1>(mdp_[step_]).copy_(std::get<1>(md));
    std::get<2>(mdp_[step_]).copy_(std::get<2>(md));
    std::get<3>(mdp_[step_]).copy_(std::get<3>(md));
    std::get<4>(mdp_[step_]).copy_(std::get<4>(md));

    step_++;

    if (step_ % steps_ == 0)
        step_ = 0;
}

auto ReplayMemory::gather() -> MDP
{
    // Randomly shuffle memory and return Markov decision process.
    std::shuffle(mdp_.begin(), mdp_.end(), re_);

    return mdp_;
}

auto ReplayMemory::to(torch::Device dev) -> void 
{
    // Load memory onto CPU/GPU.
    for (auto& md : mdp_) 
    {
        std::get<0>(md).to(dev);
        std::get<1>(md).to(dev);
        std::get<2>(md).to(dev);
        std::get<3>(md).to(dev);
        std::get<4>(md).to(dev);
    }
}
