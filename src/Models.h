#pragma once

#include <torch/torch.h>

// Network model for Proximal Policy Optimization on Incy Wincy.
struct ActorCritic : public torch::nn::Module {

    ActorCritic(int64_t n_in, int64_t n_out, float std)
        : // Actor.
          a_lin1_(torch::nn::Linear(n_in, 16)),
          a_lin2_(torch::nn::Linear(16, 32)),
          a_lin3_(torch::nn::Linear(32, n_out)),
          std_(torch::full(n_out, std)),
          
          // Critic
          c_lin1_(torch::nn::Linear(n_in, 16)),
          c_lin2_(torch::nn::Linear(16, 32)),
          c_lin3_(torch::nn::Linear(32, n_out)),
          c_val_(torch::nn::Linear(n_out, 1)) {

        // Register the modules.
        register_module("a_lin1", a_lin1_);
        register_module("a_lin2", a_lin2_);
        register_module("a_lin3", a_lin3_);
        register_parameter("std", std_);

        register_module("c_lin1", c_lin1_);
        register_module("c_lin2", c_lin2_);
        register_module("c_lin3", c_lin3_);
        register_module("c_val", c_val_);
    }

    // Forward pass.
    auto forward(torch::Tensor x, torch::Device device) -> std::tuple<torch::Tensor, torch::Tensor> {

        // Actor.
        torch::Tensor mu = torch::tanh(a_lin1_->forward(x));
        mu = torch::tanh(a_lin2_->forward(mu));
        mu = torch::tanh(a_lin3_->forward(mu));

        // Critic.
        torch::Tensor val = torch::tanh(c_lin1_(x));
        val = torch::tanh(c_lin2_(val));
        val = torch::tanh(c_lin3_(val));
        val = c_val_(val);

        // Reparametrization trick.
        if (this->is_training()) {
            torch::Tensor sample = torch::normal(torch::zeros(mu.sizes(), device), torch::ones(mu.sizes(), device)).detach();
            sample = sample.mul(std_).add(mu).expand_as(mu);
            return std::make_tuple(sample, val);  
        }
        else {
            return std::make_tuple(mu, val);  
        }
    }

    // Initialize network.
    void init() {

        torch::NoGradGuard no_grad;

        for (auto& p: this->parameters()) {
            p.normal_(0.,0.1);
        }         
    }

    // Actor.
    torch::nn::Linear a_lin1_, a_lin2_, a_lin3_;

    torch::Tensor std_;

    // Critic.
    torch::nn::Linear c_lin1_, c_lin2_, c_lin3_, c_val_;
    
};