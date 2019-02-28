#pragma once

#include <torch/torch.h>
#include <math.h>

// Network model for Proximal Policy Optimization on Incy Wincy.
struct ActorCritic : public torch::nn::Module 
{
    // Actor.
    torch::nn::Linear a_lin1_, a_lin2_, a_lin3_;
    torch::Tensor mu_;
    torch::Tensor std_;

    // Critic.
    torch::nn::Linear c_lin1_, c_lin2_, c_lin3_, c_val_;

    ActorCritic(int64_t n_in, int64_t n_out, float std)
        : // Actor.
          a_lin1_(torch::nn::Linear(n_in, 16)),
          a_lin2_(torch::nn::Linear(16, 32)),
          a_lin3_(torch::nn::Linear(32, n_out)),
          mu_(torch::full(n_out, 0.)),
          std_(torch::full(n_out, std)),
          
          // Critic
          c_lin1_(torch::nn::Linear(n_in, 16)),
          c_lin2_(torch::nn::Linear(16, 32)),
          c_lin3_(torch::nn::Linear(32, n_out)),
          c_val_(torch::nn::Linear(n_out, 1)) 
    {
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
    auto forward(torch::Tensor x, torch::Device device) -> std::tuple<torch::Tensor, torch::Tensor> 
    {
 
        // Actor.
        mu_ = torch::relu(a_lin1_->forward(x));
        mu_ = torch::relu(a_lin2_->forward(mu_));
        mu_ = a_lin3_->forward(mu_);

        // Critic.
        torch::Tensor val = torch::relu(c_lin1_(x));
        val = torch::relu(c_lin2_(val));
        val = c_lin3_(val);
        val = c_val_(val);

        // Reparametrization trick.
        if (this->is_training()) 
        {
            torch::NoGradGuard no_grad;

            torch::Tensor action = torch::normal(mu_, std_.expand_as(mu_)).to(device);
            return std::make_tuple(action, val);  
        }
        else 
        {
            return std::make_tuple(mu_, val);  
        }
    }

    // Initialize network.
    void init() 
    {
        torch::NoGradGuard no_grad;

        for (auto& p: this->parameters()) 
        {
            p.normal_(0.,0.1);
        }         
    }

    auto entropy() -> torch::Tensor
    {
        // Differential entropy of normal distribution. For reference https://pytorch.org/docs/stable/_modules/torch/distributions/normal.html#Normal
        if (*(std_.data<float>()) < 0)
            printf("std smaller zero -------------------------------------------------\n");
        return 0.5 + 0.5*log(2*M_PI) + std_.log();
    }

    auto log_prob(torch::Tensor action) -> torch::Tensor
    {
        // Logarithmic probability of taken action, given the current distribution.
        torch::Tensor var = std_*std_;
        torch::Tensor log_scale = std_.log();

        return -((action - mu_)*(action - mu_))/(2*var) - log_scale - log(sqrt(2*M_PI));
    }

    // Forward pass using reparametrization.
    auto rforward(torch::Tensor x, torch::Device device) -> std::tuple<torch::Tensor, torch::Tensor> 
    {
 
        // Actor.
        mu_ = torch::relu(a_lin1_->forward(x));
        mu_ = torch::relu(a_lin2_->forward(mu_));
        mu_ = a_lin3_->forward(mu_);

        // Critic.
        torch::Tensor val = torch::relu(c_lin1_(x));
        val = torch::relu(c_lin2_(val));
        val = c_lin3_(val);
        val = c_val_(val);

        // Reparametrization trick.
        if (this->is_training()) 
        {
            torch::Tensor action = torch::normal(torch::zeros(mu_.sizes(), device), torch::ones(mu_.sizes(), device)).detach();
            action = action.mul(std_).add(mu_).expand_as(mu_);
            return std::make_tuple(action, val);  
        }
        else 
        {
            return std::make_tuple(mu_, val);  
        }
    }
};