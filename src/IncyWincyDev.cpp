#include "IncyWincyDev.h"

IncyWincy::IncyWincy(std::string filename, 
                     std::string outLoc,
                     RigidBodyDynamics::Model* model,
                     RigidBodyDynamics::Math::VectorNd& q_init,
                     std::vector<std::string>& ballNames,
                     std::vector<uint>& ballIds,
                     std::vector<double>& ballRadii,
                     uint steps,
                     uint epochs,
                     uint mini_batch_size,
                     uint ppo_epochs,
                     int64_t n_in,
                     int64_t n_out,
                     double std)
    : model_(*model),

      q_(q_init),
      qd_ (RigidBodyDynamics::Math::VectorNd::Zero(model_.dof_count)),
      qdd_(RigidBodyDynamics::Math::VectorNd::Zero(model_.dof_count)),
      tau_(RigidBodyDynamics::Math::VectorNd::Zero(model_.dof_count)),
    //   vector_type x = zero_vector<double>(model_.dof_count*2);

      q_init_(q_init),
      qd_init_(RigidBodyDynamics::Math::VectorNd::Zero(model_.dof_count)),
      qdd_init_(RigidBodyDynamics::Math::VectorNd::Zero(model_.dof_count)),  

      x_new_(RigidBodyDynamics::CalcBodyToBaseCoordinates(*model, q_init, model->GetBodyId("Body"), RigidBodyDynamics::Math::Vector3d::Zero())[0]),
      x_old_(x_new_),

      z_new_(RigidBodyDynamics::CalcBodyToBaseCoordinates(*model, q_init, model->GetBodyId("Body"), RigidBodyDynamics::Math::Vector3d::Zero())[2]),
      z_old_(z_new_),

      reset_(false),

      exponent_(2.0),
      stiffness_(100),
      damping_(1.0),

      staticFrictionSpeed_(0.008),//0.001;
      staticFrictionCoefficient_(6.4),//0.8;
      dynamicFrictionSpeed_(0.08),//0.01;
      dynamicFrictionCoefficient_(4.8),//0.6;
      viscousFrictionSlope_(0.8),//0.1;
      veps_(staticFrictionSpeed_/100.0),

      angleAtZeroTorque_(0.), // offset for the joint torques is set via calcValue(...)
      dangle_(M_PI/2.),
      stiffnessAtLowTorque_(0.),      
      stiffnessAtOneNormTorque_(10.),//1.1/std::abs(angleAtZeroTorque-dangle); // minimum possible stiffness
      curviness_(1.),

      z_(0.),
      dz_(0.),
      fext_(model->mBodies.size(), RigidBodyDynamics::Math::SpatialVector::Zero()),

      ballNames_(ballNames),
      ballIds_(ballIds),
      ballRadii_(ballRadii),

      fK0n_(RigidBodyDynamics::Math::Vector3d::Zero()),
      tK0n_(RigidBodyDynamics::Math::Vector3d::Zero()),
      fK0t_(RigidBodyDynamics::Math::Vector3d::Zero()),
      tK0t_(RigidBodyDynamics::Math::Vector3d::Zero()),

      mu_(0.),

      EB0_(RigidBodyDynamics::Math::Matrix3d::Zero()),
      r0B0_(RigidBodyDynamics::Math::Vector3d::Zero()),
      rBKB_(RigidBodyDynamics::Math::Vector3d::Zero()),
      r0K0_(RigidBodyDynamics::Math::Vector3d::Zero()),
      v0K0_(RigidBodyDynamics::Math::Vector3d::Zero()),
      v0K0t_(RigidBodyDynamics::Math::Vector3d::Zero()),
      r0P0_(RigidBodyDynamics::Math::Vector3d::Zero()),
      eN0_(RigidBodyDynamics::Math::Vector3d::Zero()),
      eT0_(RigidBodyDynamics::Math::Vector3d::Zero()),

      steps_(steps),
      epochs_(epochs),
      mini_batch_size_(mini_batch_size),
      ppo_epochs_(ppo_epochs),
  
      n_in_(n_in),
      n_out_(n_out),
      std_(std),

      ac_(n_in, n_out, std),
    //   opt_(ac_->parameters(), 1e-3)
{
    //If veps is too small, we really might have problems
    assert(veps_ > std::sqrt(std::numeric_limits<double>::epsilon()));

    ContactToolkit::createRegularizedFrictionCoefficientCurve(staticFrictionSpeed_, staticFrictionCoefficient_,
                                                              dynamicFrictionSpeed_,dynamicFrictionCoefficient_,
                                                              viscousFrictionSlope_,"mu",frictionCoefficientCurve_);

    RigidBodyDynamics::Addons::Muscle::TorqueMuscleFunctionFactory::createPassiveTorqueAngleCurve(angleAtZeroTorque_,
                                                                                                  angleAtZeroTorque_+dangle_,
                                                                                                  stiffnessAtLowTorque_,      
                                                                                                  stiffnessAtOneNormTorque_, 
                                                                                                  curviness_, "tp", jointTorqueCurve_);
}

auto IncyWincy::operator() (const vector_type& x, vector_type& dxdt, double t) -> vector_type
{
    //q
    int j = 0;
    for(unsigned int i=0; i<model_.q_size; i++){                
        q_[i] = x(j);
        j++;
    }

    //qd
    for(unsigned int i=0; i<model_.qdot_size; i++){
        qd_[i] = x(j);
        j++;
    }

    //tau = 0, this is the trigger for the agent
    double q_max = 0.25;

    auto act_val = ac_->forward(states_[c_]);
    actions_.push_back(std::get<0>(act_val));
    values_.push_back(std::get<1>(act_val));

    for(unsigned int i=3; i<model_.qdot_size; i++){ // floating base shouldnt be able to actuate the spider, hence i=3              
        tau_[i] = *(actions_[c_].data<double>() + (i-3));
        tau_[i] *= q_max;
    }

    int no_float = model_.q_size-3; // no floating base
    tau_.bottomRows(no_float) += Torque(q_.bottomRows(no_float), 
                                        q_init_.bottomRows(no_float), 
                                        qd_.bottomRows(no_float), q_max, 0.01);

    for (unsigned int i=0; i<ballIds_.size(); i++){

        //Calculate the contact forces and populate fext
        r0B0_ = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_,q_,ballIds_[i],RigidBodyDynamics::Math::Vector3dZero,true);
        ContactToolkit::calcSpherePlaneContactPointPosition(r0B0_,ballRadii_[i],eN0_,r0K0_);

        //if the contact point is in the sphere compute contact data
        z_ = (r0K0_-r0P0_).dot(eN0_);
        dz_=0.; //this is zero until contact is made

        if( z_ < 0. ){
            if (!strcmp(model_.GetBodyName(ballIds_[i]).c_str(), "Body"))
            {
                reset_ = true;
            }

            //Get the point of contact resolved in the coordinates of the ball          
            EB0_   = RigidBodyDynamics::CalcBodyWorldOrientation(model_,q_,ballIds_[i],true);
            rBKB_  = EB0_*(r0K0_-r0B0_);

            //Get the velocity of the point of contact
            v0K0_ = RigidBodyDynamics::CalcPointVelocity(model_,q_,qd_,ballIds_[i],rBKB_,true);

            //Evaluate Hunt-Crossley Contact forces
            dz_ = (v0K0_).dot(eN0_); //assuming the plane is fixed.
            ContactToolkit::calcHuntCrossleyContactForce(z_,dz_,stiffness_,exponent_,damping_,hcInfo_);

            //Resolve the scalar force into the root frame as a wrench
            fK0n_ = hcInfo_.force*eN0_;
            tK0n_ = RigidBodyDynamics::Math::VectorCrossMatrix(r0K0_)*fK0n_;

            //Now go and compute the applied friction forces
            v0K0t_ = v0K0_ - dz_*eN0_;
            ContactToolkit::calcTangentialVelocityDirection(v0K0t_,veps_,eT0_);
            mu_ = frictionCoefficientCurve_.calcValue(v0K0t_.norm());
            fK0t_ = -mu_*hcInfo_.force*eT0_;
            tK0t_ = RigidBodyDynamics::Math::VectorCrossMatrix(r0K0_)*fK0t_;
            //Apply it to fext
            if (model_.IsFixedBodyId(ballIds_[i])) { // rbdl performs some reparametrization for fixed bodies, we need to address for that
                fext_[model_.GetParentBodyId(ballIds_[i])][0] += tK0n_[0] + tK0t_[0];
                fext_[model_.GetParentBodyId(ballIds_[i])][1] += tK0n_[1] + tK0t_[1];
                fext_[model_.GetParentBodyId(ballIds_[i])][2] += tK0n_[2] + tK0t_[2];

                fext_[model_.GetParentBodyId(ballIds_[i])][3] += fK0n_[0] + fK0t_[0];
                fext_[model_.GetParentBodyId(ballIds_[i])][4] += fK0n_[1] + fK0t_[1];
                fext_[model_.GetParentBodyId(ballIds_[i])][5] += fK0n_[2] + fK0t_[2];
            }
            else {
                fext_[ballIds_[i]][0] = tK0n_[0] + tK0t_[0];
                fext_[ballIds_[i]][1] = tK0n_[1] + tK0t_[1];
                fext_[ballIds_[i]][2] = tK0n_[2] + tK0t_[2];

                fext_[ballIds_[i]][3] = fK0n_[0] + fK0t_[0];
                fext_[ballIds_[i]][4] = fK0n_[1] + fK0t_[1];
                fext_[ballIds_[i]][5] = fK0n_[2] + fK0t_[2];
            }

        }else{
            //zero the entry of fext associated with the ball.     
            if (!model_.IsFixedBodyId(ballIds_[i])) {  
                fext_[ballIds_[i]]=RigidBodyDynamics::Math::SpatialVector::Zero();
            }
            hcInfo_.force        = 0.;
            hcInfo_.springForce  = 0.;
            hcInfo_.dampingForce = 0.;          
            fK0n_.setZero();
            tK0n_.setZero();
            fK0t_.setZero();
            tK0t_.setZero();
        }
    }

    RigidBodyDynamics::ForwardDynamics(model_,q_,qd_,tau_,qdd_,&fext_);
    for (auto& f: fext_) { // set forces to zero after each iteration
        f.setZero();
    }

    //populate dxdt
    //If you are using a quaternion joint, you must map wx,wy,wz to the 
    //derivatives of the 4 quaternion components using the omegaToQDot 
    //function
    j = 0;
    for(unsigned int i = 0; i < model_.q_size; i++){
        dxdt[j] = double(qd_[i]);
        j++;
    }
    for(unsigned int i = 0; i < model_.qdot_size; i++){
        dxdt[j] = double(qdd_[i]);
        j++;
    }
}

auto IncyWincy::State(bool ext) -> void
{
    states_.push_back(torch::zeros({1, model_.dof_count*3 + int(fext_.size())*6}, torch::kF64));

    uint j = 0;

    for (auto& x: q_)
    {
        states_[c_][0][j] = x;
        j++;
    }

    for (auto& x: qd_)
    {
        states_[c_][0][j] = x;
        j++;
    }

    for (auto& x: qdd_)
    {
        states_[c_][0][j] = x;
        j++;
    }

    if (ext)
    {
        for (auto& x: fext_)
        {
            states_[c_][0][j] = x[0];
            states_[c_][0][j+1] = x[1];
            states_[c_][0][j+2] = x[2];
            states_[c_][0][j+3] = x[3];
            states_[c_][0][j+4] = x[4];
            states_[c_][0][j+5] = x[5];
            j += 6; 
        }
    }
}

auto IncyWincy::Update() -> bool
{
    // Get the new coordinates.
    x_new_ = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_, q_, model_.GetBodyId("Body"), RigidBodyDynamics::Math::Vector3d::Zero())[0];
    z_new_ = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_, q_, model_.GetBodyId("Body"), RigidBodyDynamics::Math::Vector3d::Zero())[2];

    // Insert state, action, reward, next_state and done into memory.
    double n_z = (RigidBodyDynamics::CalcBodyWorldOrientation(model_, q_, model_.GetBodyId("Body"), false)*eN0_).transpose()*eN0_;

    rewards_.push_back(Reward(x_old_, x_new_, z_old_, z_new_, n_z, tau_, reset_));
    dones_.push_back(torch::full({1,1}, (reset_ ? 1. : 0.), torch::kF64));

    x_old_ = x_new_;
    z_old_ = z_new_;

    log_probs_.push_back(ac_->log_prob(actions_[c_]));

    c_++;

    return reset_;
}

auto IncyWincy::Process() -> void 
{
    values_.push_back(std::get<1>(ac_->forward(states_[c_-1])));

    returns_ = PPO::returns(rewards_, dones_, values_, .99, .95);

    torch::Tensor rets = torch::cat(returns_).detach();
    torch::Tensor logs = torch::cat(log_probs_).detach();
    torch::Tensor vals = torch::cat(values_).detach();
    torch::Tensor stas = torch::cat(states_);
    torch::Tensor acts = torch::cat(actions_);
    torch::Tensor advs = rets - vals.slice(0, 0, steps_); // size mismatch between val and ret cause of next val

    PPO::update(ac_, stas, acts, logs, rets, advs, opt_, steps_, ppo_epochs_, mini_batch_size_);

    c_ = 0;

    states_.clear();
    actions_.clear();
    rewards_.clear();
    dones_.clear();

    log_probs_.clear();
    values_.clear();
    returns_.clear();
}

auto IncyWincy::Reset() -> void
{
    q_ = q_init_;
    qd_.setZero();
    qdd_.setZero();
}

auto IncyWincy::Reward(double x_old, double x_new, double z_old, double z_new, double n_z, RigidBodyDynamics::Math::VectorNd torque, bool reset) -> torch::Tensor
{
    double r = (x_new - x_old) + (z_new - z_old) + 0.1*n_z - 0.1*torque.norm();
    if (reset) {
        r -= 100.;
    }
    torch::Tensor reward = torch::full({1,1}, r, torch::kF64);
    return reward;
}

template <typename Derived>
auto IncyWincy::Torque(const Eigen::MatrixBase<Derived>& q, 
                       const Eigen::MatrixBase<Derived>& q_init, 
                       const Eigen::MatrixBase<Derived>& dq, double q_max, double beta) -> RigidBodyDynamics::Math::VectorNd
{
    RigidBodyDynamics::Math::VectorNd tp = RigidBodyDynamics::Math::VectorNd::Zero(model_.dof_count-3); // without floating base

    for (uint i=0;i<tp.size();i++)
    {
        int sign = (std::signbit(q[i]-q_init[i]) ? 1 : -1);
        double te = q_max*sign*jointTorqueCurve_.calcValue(std::abs(q[i]-q_init[i]));
        tp[i] = te*(1+beta*dq[i]); 
    }
    
    return tp;
}