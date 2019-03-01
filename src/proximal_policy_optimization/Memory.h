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

class Memory
{
private:
    uint steps_;
    uint step_;

    MDP mdp_;

public:
    Memory(uint steps, torch::ArrayRef<int64_t> state_shape, torch::ArrayRef<int64_t> action_shape);

    auto insert(MD md) -> void;
    auto to(torch::Device dev) -> void;

    // Getters.
    inline auto size() const -> const uint& { return steps_; };
    inline auto mdp() const -> const MDP& { return mdp_; };
};

Memory::Memory(uint steps, torch::ArrayRef<int64_t> state_shape, torch::ArrayRef<int64_t> action_shape)
    : steps_(steps),
      step_(0)
{
    for (uint i=0;i<steps;i++) 
    {
        mdp_.push_back(std::make_tuple(torch::zeros(state_shape),
                                       torch::zeros(action_shape),
                                       torch::zeros({1}),
                                       torch::zeros(state_shape),
                                       torch::zeros({1})));
    }
}

auto Memory::insert(MD md) -> void
{
    std::get<0>(mdp_[step_]).copy_(std::get<0>(md));
    std::get<1>(mdp_[step_]).copy_(std::get<1>(md));
    std::get<2>(mdp_[step_]).copy_(std::get<2>(md));
    std::get<3>(mdp_[step_]).copy_(std::get<3>(md));
    std::get<4>(mdp_[step_]).copy_(std::get<4>(md));

    step_++;

    if (step_ % steps_ == 0)
    {
        step_ = 0;
    }
}

auto Memory::to(torch::Device dev) -> void 
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
