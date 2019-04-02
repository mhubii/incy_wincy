#pragma once

#include <torch/torch.h>
#include <math.h>

// Network model for Proximal Policy Optimization on Incy Wincy.
struct ActorCriticImpl : public torch::nn::Module 
{
    // Actor.
    torch::nn::Linear amod_lin1_{nullptr}, amod_lin2_{nullptr}, amod_lin3_{nullptr}, // model
                      amod_lin4_{nullptr}, amod_lin5_{nullptr}, amod_lin6_{nullptr}; 

    torch::nn::Linear aenv_lin1_{nullptr}, aenv_lin2_{nullptr}, aenv_lin3_{nullptr}, // environment
                      aenv_lin4_{nullptr}, aenv_lin5_{nullptr}, aenv_lin6_{nullptr}; 
          
    torch::nn::Linear a_lin1_{nullptr}, a_lin2_{nullptr};

    torch::Tensor mu_;
    torch::Tensor log_std_;
    double mu_max_;
    double std_max_;

    // Critic.
    torch::nn::Linear cmod_lin1_{nullptr}, cmod_lin2_{nullptr}, cmod_lin3_{nullptr}, // model
                      cmod_lin4_{nullptr}, cmod_lin5_{nullptr}, cmod_lin6_{nullptr}; 

    torch::nn::Linear cenv_lin1_{nullptr}, cenv_lin2_{nullptr}, cenv_lin3_{nullptr}, // environment
                      cenv_lin4_{nullptr}, cenv_lin5_{nullptr}, cenv_lin6_{nullptr}; 
          
    torch::nn::Linear c_lin1_{nullptr}, c_lin2_{nullptr}, c_val_{nullptr};

    ActorCriticImpl(int64_t n_in_mod, int64_t n_in_env, int64_t n_out, double mu_max, double std)
        : // Actor.
          amod_lin1_(torch::nn::Linear(n_in_mod, 16)), // model
          amod_lin2_(torch::nn::Linear(16, 32)),
          amod_lin3_(torch::nn::Linear(32, 32)),
          amod_lin4_(torch::nn::Linear(32, 32)),
          amod_lin5_(torch::nn::Linear(32, 32)),
          amod_lin6_(torch::nn::Linear(32, 16)),

          aenv_lin1_(torch::nn::Linear(n_in_env, 16)), // environment
          aenv_lin2_(torch::nn::Linear(16, 32)),
          aenv_lin3_(torch::nn::Linear(32, 32)),
          aenv_lin4_(torch::nn::Linear(32, 32)),
          aenv_lin5_(torch::nn::Linear(32, 32)),
          aenv_lin6_(torch::nn::Linear(32, 16)),

          a_lin1_(torch::nn::Linear(32, 16)),
          a_lin2_(torch::nn::Linear(16, n_out)),

          mu_(torch::full(n_out, 0.)),
          log_std_(torch::full(n_out, log(std), torch::kFloat64)),
          mu_max_(mu_max),
          std_max_(std_max),
          
          // Critic
          cmod_lin1_(torch::nn::Linear(n_in_mod, 16)), // model
          cmod_lin2_(torch::nn::Linear(16, 32)),
          cmod_lin3_(torch::nn::Linear(32, 32)),
          cmod_lin4_(torch::nn::Linear(32, 32)),
          cmod_lin5_(torch::nn::Linear(32, 32)),
          cmod_lin6_(torch::nn::Linear(32, 16)),

          cenv_lin1_(torch::nn::Linear(n_in_env, 16)), // environment
          cenv_lin2_(torch::nn::Linear(16, 32)),
          cenv_lin3_(torch::nn::Linear(32, 32)),
          cenv_lin4_(torch::nn::Linear(32, 32)),
          cenv_lin5_(torch::nn::Linear(32, 32)),
          cenv_lin6_(torch::nn::Linear(32, 16)),

          c_lin1_(torch::nn::Linear(32, 16)),
          c_lin2_(torch::nn::Linear(16, n_out)),
          c_val_(torch::nn::Linear(n_out, 1)) 
    {
        // Register the modules.
        // Actor.
        register_module("amod_lin1", amod_lin1_);
        register_module("amod_lin2", amod_lin2_);
        register_module("amod_lin3", amod_lin3_);
        register_module("amod_lin4", amod_lin4_);
        register_module("amod_lin5", amod_lin5_);
        register_module("amod_lin6", amod_lin6_);

        register_module("aenv_lin1", aenv_lin1_);
        register_module("aenv_lin2", aenv_lin2_);
        register_module("aenv_lin3", aenv_lin3_);
        register_module("aenv_lin4", aenv_lin4_);
        register_module("aenv_lin5", aenv_lin5_);
        register_module("aenv_lin6", aenv_lin6_);

        register_module("a_lin1", a_lin1_);
        register_module("a_lin2", a_lin2_);
        register_parameter("log_std", log_std_);

        // Critic.
        register_module("cmod_lin1", cmod_lin1_);
        register_module("cmod_lin2", cmod_lin2_);
        register_module("cmod_lin3", cmod_lin3_);
        register_module("cmod_lin4", cmod_lin4_);
        register_module("cmod_lin5", cmod_lin5_);
        register_module("cmod_lin6", cmod_lin6_);

        register_module("cenv_lin1", cenv_lin1_);
        register_module("cenv_lin2", cenv_lin2_);
        register_module("cenv_lin3", cenv_lin3_);
        register_module("cenv_lin4", cenv_lin4_);
        register_module("cenv_lin5", cenv_lin5_);
        register_module("cenv_lin6", cenv_lin6_);

        register_module("c_lin1", c_lin1_);
        register_module("c_lin2", c_lin2_);
        register_module("c_val", c_val_);
    }

    ActorCriticImpl() = default;

    // Forward pass.
    auto forward(torch::Tensor mod /*model*/, torch::Tensor env /*environment*/) -> std::tuple<torch::Tensor, torch::Tensor> 
    {

        // Actor.
        torch::Tensor amod = torch::relu(amod_lin1_->forward(mod));
        amod = torch::relu(amod_lin2_->forward(amod));
        amod = torch::relu(amod_lin3_->forward(amod));
        amod = torch::relu(amod_lin4_->forward(amod));
        amod = torch::relu(amod_lin5_->forward(amod));
        amod = torch::relu(amod_lin6_->forward(amod));

        torch::Tensor aenv = torch::relu(aenv_lin1_->forward(env));
        aenv = torch::relu(aenv_lin2_->forward(aenv));
        aenv = torch::relu(aenv_lin3_->forward(aenv));
        aenv = torch::relu(aenv_lin4_->forward(aenv));
        aenv = torch::relu(aenv_lin5_->forward(aenv));
        aenv = torch::relu(aenv_lin6_->forward(aenv));

        mu_ = torch::cat({amod, aenv}, 1);

        mu_ = torch::relu(a_lin1_->forward(mu_));
        mu_ = torch::tanh(a_lin2_->forward(mu_)).mul(mu_max_);

        // Critic.
        torch::Tensor cmod = torch::relu(cmod_lin1_->forward(mod));
        cmod = torch::relu(cmod_lin2_->forward(cmod));
        cmod = torch::relu(cmod_lin3_->forward(cmod));
        cmod = torch::relu(cmod_lin4_->forward(cmod));
        cmod = torch::relu(cmod_lin5_->forward(cmod));
        cmod = torch::relu(cmod_lin6_->forward(cmod));

        torch::Tensor cenv = torch::relu(cenv_lin1_->forward(env));
        cenv = torch::relu(cenv_lin2_->forward(cenv));
        cenv = torch::relu(cenv_lin3_->forward(cenv));
        cenv = torch::relu(cenv_lin4_->forward(cenv));
        cenv = torch::relu(cenv_lin5_->forward(cenv));
        cenv = torch::relu(cenv_lin6_->forward(cenv));

        torch::Tensor val = torch::cat({cmod, cenv}, 1);

        val = torch::relu(c_lin1_->forward(val));
        val = torch::tanh(c_lin2_->forward(val)).mul(mu_max_);
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
