#pragma once

#include <iostream>
#include <chrono>
#include <string>
#include <fstream>
#include <iomanip>
#include <math.h>

#include <boost/numeric/odeint.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/odeint/integrate/integrate_adaptive.hpp>

#include <rbdl/rbdl.h>
#include <rbdl/addons/luamodel/luamodel.h>
#include <rbdl/addons/geometry/geometry.h>
#include <rbdl/addons/muscle/TorqueMuscleFunctionFactory.h>

#include "csvtools.h"
#include "ContactToolkit.h"

#include <torch/torch.h>

#include "Models.h"
#include "ProximalPolicyOptimization.h"


// Some typedefs.
using vector_type = boost::numeric::ublas::vector< double >;
using matrix_type = boost::numeric::ublas::matrix< double >;

// Physical description of the system.
class IncyWincy
{
public:
    IncyWincy(std::string filename, 
              std::string outLoc,
              RigidBodyDynamics::Model*,
              RigidBodyDynamics::Math::VectorNd& q_init,
              std::vector<std::string>& ballNames,
              std::vector<uint>& ballIds,
              std::vector<double>& ballRadii,
              uint steps,
              uint epochs,
              uint mini_batch_size,
              uint ppo_epochs,
              int64_t n_in,
              int64_t nout,
              double std);

    auto operator() (const vector_type& x, vector_type& dxdt, double t) -> vector_type;

    auto State(bool ext=true) -> void;
    auto Update() -> bool;
    auto Process() -> void;
    auto Reset() -> void;
    auto Reward(double x_old, double x_new, double z_old, double z_new, double n_z, RigidBodyDynamics::Math::VectorNd torque, bool reset) -> torch::Tensor;

    inline const uint& Counter() const { return c_; };

    template <typename Derived>
    auto Torque(const Eigen::MatrixBase<Derived>& q, 
                const Eigen::MatrixBase<Derived>& q_off, 
                const Eigen::MatrixBase<Derived>& dq, double q_max, double beta) -> RigidBodyDynamics::Math::VectorNd;

    RigidBodyDynamics::Model model_;

    RigidBodyDynamics::Math::VectorNd q_, qd_, qdd_, tau_;
    RigidBodyDynamics::Math::VectorNd q_init_, qd_init_, qdd_init_;

    double x_new_, x_old_, z_new_, z_old_; // for the reward

    bool reset_;

    //Hunt-Crossley contact terms. See
    // ContactToolkit::calcHuntCrossleyContactForce
    //for details
    double exponent_; //The spring force will increase with the deflection squared.
    double stiffness_; //The ball will settle to 1cm penetration
    double damping_; //lightly damped  

    //Friction model terms. See
    // ContactToolkit::createRegularizedFrictionCoefficientCurve for details
    double staticFrictionSpeed_;
    double staticFrictionCoefficient_;
    double dynamicFrictionSpeed_;
    double dynamicFrictionCoefficient_;
    double viscousFrictionSlope_;
    double veps_;

    RigidBodyDynamics::Addons::Geometry::SmoothSegmentedFunction frictionCoefficientCurve_;

    double angleAtZeroTorque_;
    double dangle_;
    double stiffnessAtLowTorque_;      
    double stiffnessAtOneNormTorque_;
    double curviness_;

    RigidBodyDynamics::Addons::Geometry::SmoothSegmentedFunction jointTorqueCurve_;

    double z_, dz_; //pentration depth and velocity
    HuntCrossleyContactInfo hcInfo_;
    std::vector<RigidBodyDynamics::Math::SpatialVector> fext_;

    // Spheres.
    std::vector<std::string> ballNames_;
    std::vector<uint> ballIds_;
    std::vector<double> ballRadii_;

    // Forces and torques.
    RigidBodyDynamics::Math::Vector3d fK0n_;
    RigidBodyDynamics::Math::Vector3d tK0n_;
    RigidBodyDynamics::Math::Vector3d fK0t_;
    RigidBodyDynamics::Math::Vector3d tK0t_;

    double mu_; //Friction coefficient

    // Transformations.
    RigidBodyDynamics::Math::Matrix3d EB0_;
    RigidBodyDynamics::Math::Vector3d r0B0_;  //Position of the ball
    RigidBodyDynamics::Math::Vector3d rBKB_;  //B : ball.
    RigidBodyDynamics::Math::Vector3d r0K0_;  //K : contact point
    RigidBodyDynamics::Math::Vector3d v0K0_;  //velocity of the contact point
    RigidBodyDynamics::Math::Vector3d v0K0t_; //tangential velocity of the contact point
    RigidBodyDynamics::Math::Vector3d r0P0_;  //Origin of the plane
    RigidBodyDynamics::Math::Vector3d eN0_;   //Normal of the plane
    RigidBodyDynamics::Math::Vector3d eT0_;   //Tangental direction of the plane: in 2d this isn't necessary
                                              //here we compute it to show how this is done in a
                                              //numerically stable way in 3d.

    // ---------------- DEEP LEARNING
    uint steps_;
    uint epochs_;
    uint mini_batch_size_;
    uint ppo_epochs_;

    int64_t n_in_; // q, qd, qdd, fext
    int64_t n_out_; // control tau
    double std_;

    ActorCritic ac_;
    // torch::optim::Adam opt_;

    VT states_;
    VT actions_;
    VT rewards_;
    VT dones_;

    VT log_probs_;
    VT values_;
    VT returns_;

    uint c_; // counter
};
