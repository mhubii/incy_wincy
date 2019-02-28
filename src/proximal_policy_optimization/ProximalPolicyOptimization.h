#pragma once

#include <torch/torch.h>

#include "Memory.h"
#include "Models.h"

// Generalized advantage estimate.
using GAE = std::vector<std::tuple<torch::Tensor /*state*/,
                                   torch::Tensor /*action*/,
                                   torch::Tensor /*policy*/,
                                   torch::Tensor /*return*/,
                                   torch::Tensor /*advantage*/>>;

// Vector of tensors.
using VT = std::vector<torch::Tensor>;

// Random engine for shuffling memory.
std::random_device rd;
std::mt19937 re(rd());

// Proximal policy optimization, https://arxiv.org/abs/1707.06347
class PPO
{
public:
    static auto returns(const MDP& mdp, VT& val, float gamma, float lambda) -> VT; // Generalized advantage estimate, https://arxiv.org/abs/1506.02438
    static auto update(GAE& gae) -> void; // Update the policy after T time steps for K epochs
};

auto PPO::returns(const MDP& mdp, VT& val, float gamma, float lambda) -> VT
{
    // Compute the returns.
    torch::Tensor gae = torch::zeros({1});
    VT returns(mdp.size(), torch::zeros({1}));

    for (uint i=mdp.size()-1;i>=0;i--)
    {
        // Advantage.
        auto reward = std::get<2>(mdp[i]);
        auto done = std::get<4>(mdp[i]);

        auto delta = reward + gamma*val[i+1]*done - val[i];
        gae = delta + gamma*lambda*done*gae;

        returns[i] = gae + val[i];
    }

    return returns;
}

auto PPO::update(GAE& gae) -> void
{
    // Iterate over shuffled GAE and update mod.
    std::shuffle(gae.begin(), gae.end(), re);
}
