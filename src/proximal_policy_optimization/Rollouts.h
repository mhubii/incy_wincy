#pragma once

#include <torch/torch.h>

class Rollouts
{
private:
    torch::Tensor states_;
    torch::Tensor actions_;
    torch::Tensor rewards_;
    torch::Tensot next_states_;
    torch::Tensor dones_;

    uint steps_;

public:
    Rollouts(uint steps);
    ~Rollouts();
    auto to(torch::Device dev) -> void;
};

Rollouts::Rollouts(uint steps)
    : steps_(steps)
{
}

Rollouts::~Rollouts() 
{
}

auto Rollouts::to(torch::Device dev) -> void 
{
    // Load memory onto CPU/GPU.
    states_.to(device);
    actions_.to(device);
    actions_.to(device);
    rewards_.to(device);
    next_states_.to(device);
    dones_.to(device);
}