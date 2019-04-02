#pragma once

#include <torch/torch.h>
#include <math.h>

// Network model for Proximal Policy Optimization on Incy Wincy.
struct ActorCriticImpl : public torch::nn::Module 
{
    // Actor.
    torch::nn::Linear a_lin1_{nullptr}, a_lin2_{nullptr}, a_lin3_{nullptr}, a_lin4_{nullptr},
                      a_lin5_{nullptr}, a_lin6_{nullptr};
    torch::Tensor mu_;
    torch::Tensor log_std_;
    double mu_max_;
    double std_max_;

    // Critic.
    torch::nn::Linear c_lin1_{nullptr}, c_lin2_{nullptr}, c_lin3_{nullptr}, c_lin4_{nullptr},
                      c_lin5_{nullptr}, c_lin6_{nullptr}, c_val_{nullptr};

    ActorCriticImpl(int64_t n_in, int64_t n_out, double mu_max, double std_max)
        : // Actor.
          a_lin1_(torch::nn::Linear(n_in, 16)),
          a_lin2_(torch::nn::Linear(16, 32)),
          a_lin3_(torch::nn::Linear(32, 64)),
          a_lin4_(torch::nn::Linear(64, 64)),
          a_lin5_(torch::nn::Linear(64, 32)),
          a_lin6_(torch::nn::Linear(32, n_out)),
          mu_(torch::full(n_out, 0.)),
          log_std_(torch::full(n_out, std_max, torch::kFloat64)),
          mu_max_(mu_max),
          std_max_(std_max),
          
          // Critic
          c_lin1_(torch::nn::Linear(n_in, 16)),
          c_lin2_(torch::nn::Linear(16, 32)),
          c_lin3_(torch::nn::Linear(32, 64)),
          c_lin4_(torch::nn::Linear(64, 64)),
          c_lin5_(torch::nn::Linear(64, 32)),
          c_lin6_(torch::nn::Linear(32, n_out)),
          c_val_(torch::nn::Linear(n_out, 1)) 
    {
        // Register the modules.
        register_module("a_lin1", a_lin1_);
        register_module("a_lin2", a_lin2_);
        register_module("a_lin3", a_lin3_);
        register_module("a_lin4", a_lin4_);
        register_module("a_lin5", a_lin5_);
        register_module("a_lin6", a_lin6_);
        register_parameter("log_std", log_std_);

        register_module("c_lin1", c_lin1_);
        register_module("c_lin2", c_lin2_);
        register_module("c_lin3", c_lin3_);
        register_module("c_lin4", c_lin4_);
        register_module("c_lin5", c_lin5_);
        register_module("c_lin6", c_lin6_);
        register_module("c_val", c_val_);
    }

    ActorCriticImpl() = default;

    // Forward pass.
    auto forward(torch::Tensor x) -> std::tuple<torch::Tensor, torch::Tensor> 
    {

        // Actor.
        mu_ = torch::relu(a_lin1_->forward(x));
        mu_ = torch::relu(a_lin2_->forward(mu_));
        mu_ = torch::relu(a_lin3_->forward(mu_));
        mu_ = torch::relu(a_lin4_->forward(mu_));
        mu_ = torch::relu(a_lin5_->forward(mu_));
        mu_ = torch::tanh(a_lin6_->forward(mu_)).mul(mu_max_);

        // Critic.
        torch::Tensor val = torch::relu(c_lin1_->forward(x));
        val = torch::relu(c_lin2_->forward(val));
        val = torch::relu(c_lin3_->forward(val));
        val = torch::relu(c_lin4_->forward(val));
        val = torch::relu(c_lin5_->forward(val));
        val = torch::tanh(c_lin6_->forward(val)).mul(mu_max_);
        val = c_val_->forward(val);

        // Reparametrization trick.
        if (this->is_training()) 
        {
            torch::NoGradGuard no_grad;

            torch::Tensor action = torch::normal(mu_, torch::sigmoid(log_std_.exp().expand_as(mu_).mul(std_max_)));
            return std::make_tuple(action, val);  
        }
        else 
        {
            return std::make_tuple(mu_, val);  
        }
    }

    // Initialize network.
    void normal(double mu, double std) 
    {
        torch::NoGradGuard no_grad;

        for (auto& p: this->parameters()) 
        {
            p.normal_(mu,std);
        }         
    }

    auto entropy() -> torch::Tensor
    {
        // Differential entropy of normal distribution. For reference https://pytorch.org/docs/stable/_modules/torch/distributions/normal.html#Normal
        return 0.5 + 0.5*log(2*M_PI) + log_std_;
    }

    auto log_prob(torch::Tensor action) -> torch::Tensor
    {
        // Logarithmic probability of taken action, given the current distribution.
        torch::Tensor var = (log_std_+log_std_).exp();

        return -((action - mu_)*(action - mu_))/(2*var) - log_std_ - log(sqrt(2*M_PI));
    }
};

TORCH_MODULE(ActorCritic);
