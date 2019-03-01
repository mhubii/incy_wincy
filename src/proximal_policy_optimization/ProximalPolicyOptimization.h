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
    static auto returns(const MDP& mdp, VT& vals, float gamma, float lambda) -> VT; // Generalized advantage estimate, https://arxiv.org/abs/1506.02438
    static auto update(ActorCritic& mod, GAE& gaes, float clip_param=.2) -> void; // Update the policy after T time steps for K epochs
};

auto PPO::returns(const MDP& mdp, VT& vals, float gamma, float lambda) -> VT
{
    // Compute the returns.
    torch::Tensor gae = torch::zeros({1});
    VT returns(mdp.size(), torch::zeros({1}));

    for (uint i=mdp.size();i-- >0;) // inverse for loops over unsigned: https://stackoverflow.com/questions/665745/whats-the-best-way-to-do-a-reverse-for-loop-with-an-unsigned-index/665773
    {
        // Advantage.
        auto reward = std::get<2>(mdp[i]);
        auto done = (1-std::get<4>(mdp[i]));

        auto delta = reward + gamma*vals[i+1]*done - vals[i];
        gae = delta + gamma*lambda*done*gae;

        returns[i] = gae + vals[i];
    }

    return returns;
}

auto PPO::update(ActorCritic& ac, GAE& gaes, float clip_param) -> void
{
    std::shuffle(gaes.begin(), gaes.end(), re);

    for (auto& gae: gaes)
    {
        auto state = std::get<0>(gae);
        auto av = ac.forward(state); // action value pairs
        auto action = std::get<0>(av);
        auto entropy = ac.entropy();
        auto new_log_prob = ac.log_prob(action);

        auto old_log_prob = std::get<2>(gae);
        auto ratio = (new_log_prob - old_log_prob).exp();
        auto advantage = std::get<4>(gae);
        auto surr1 = ratio*advantage;
        auto surr2 = torch::clamp(ratio, 1. - clip_param, 1. + clip_param)*advantage;

        auto ret = std::get<4>(gae);
        auto val = std::get<1>(av);
        auto actor_loss = -torch::min(surr1, surr2);
        auto critic_loss = (ret-val).pow(2);

        auto loss = 0.5*critic_loss+actor_loss-0.001*entropy;

        // next optimize.. where to get parameters for optimizer from?? where to get optimizer from
        // without having everything being coupled too much??? how do I remove the device from the 
        // forward function?? how do I set the device for tensors within the actor critic class, that
        // are not registered???
    }
}
