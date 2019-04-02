#include <iostream>
#include <chrono>
#include <string>
#include <fstream>
#include <iomanip>

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
#include "Timer.h"
#include "NumericalJacobian.h"

using namespace boost::numeric::ublas;
using namespace boost::numeric::odeint;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;


// Here comes the actual physical model of the system we want to describe, see Matt's implementation.
// This function needs to be of system_func_ptr type.
Model model;

VectorNd q, qd, qdd, tau;
VectorNd q_init, qd_init, qdd_init;

VectorNd q_off; // offset for forces in torque

double y_old, y_new; // for the reward

bool reset = false;
bool reset_episode = true;

//Hunt-Crossley contact terms. See
// ContactToolkit::calcHuntCrossleyContactForce
//for details
double exponent; //The spring force will increase with the deflection squared.
double stiffness; //The ball will settle to 1cm penetration
double damping; //lightly damped  

//Friction model terms. See
// ContactToolkit::createRegularizedFrictionCoefficientCurve for details
double staticFrictionSpeed;
double staticFrictionCoefficient;
double dynamicFrictionSpeed;
double dynamicFrictionCoefficient;
double viscousFrictionSlope;
double veps;

Addons::Geometry::SmoothSegmentedFunction frictionCoefficientCurve;

double angleAtZeroTorque;
double dangle;
double stiffnessAtLowTorque;      
double stiffnessAtOneNormTorque;
double curviness;

Addons::Geometry::SmoothSegmentedFunction jointTorqueCurve;

double z, dz; //pentration depth and velocity
HuntCrossleyContactInfo hcInfo;
std::vector<SpatialVector> fext;

// Spheres.
std::vector<uint> ballIds;
std::vector<double> ballRadii;

// Forces and torques.
Vector3d fK0n;
Vector3d tK0n;
Vector3d fK0t;
Vector3d tK0t;

double mu; //Friction coefficient

// Transformations.
Matrix3d EB0;
Vector3d r0B0;  //Position of the ball
Vector3d rBKB;  //B : ball.
Vector3d r0K0;  //K : contact point
Vector3d v0K0;  //velocity of the contact point
Vector3d v0K0t; //tangential velocity of the contact point
Vector3d r0P0;  //Origin of the plane
Vector3d eN0;   //Normal of the plane
Vector3d eT0;   //Tangental direction of the plane: in 2d this isn't necessary
                //here we compute it to show how this is done in a
                //numerically stable way in 3d.

// ---------------- DEEP LEARNING
ActorCritic ac;

VT states;
VT actions;
VT rewards;
VT dones;

VT log_probs;
VT values;
VT returns;

std::vector<int> idx; // index vector for actuation
bool notanumber = false;
uint c; // counter


auto compute_reward(double v_y, double n_z, VectorNd torque, bool reset) -> torch::Tensor
{
    // r = v_y + 0.05 n_z + (z>0 or sth) - 0.01 |torque|
    double r = v_y + 0.1*n_z - 0.1*torque.norm();
    if (reset) {
        r -= 10.;
    }
    torch::Tensor reward = torch::full({1,1}, r, torch::kF64);
    return reward;
}

auto compute_reward(double y_old, double y_new, double n_z, VectorNd torque, bool reset) -> torch::Tensor
{
    double r = 1e3*(y_new - y_old) + n_z - 1e1*torque.norm();
    if (reset) {
        r -= 10.;
    }
    torch::Tensor reward = torch::full({1,1}, r, torch::kF64);
    return reward;
}

double v_y;
double n_z;

auto to_state(VectorNd& q, VectorNd& qd, VectorNd qdd, std::vector<SpatialVector>& fext, bool ext=false) -> torch::Tensor
{    
    torch::Tensor state;
    if (ext) {
        state = torch::zeros({1, model.dof_count*3 + int(fext.size())*6}, torch::kF64);
    }
    else
    {
        state = torch::zeros({1, model.dof_count*3}, torch::kF64);
    }

    uint j = 0;

    for (auto& x: q)
    {
        state[0][j] = x;
        j++;
    }

    for (auto& x: qd)
    {
        state[0][j] = x;
        j++;
    }

    for (auto& x: qdd)
    {
        state[0][j] = x;
        j++;
    }

    if (ext)
    {
        for (auto& x: fext)
        {
            state[0][j] = x[0];
            state[0][j+1] = x[1];
            state[0][j+2] = x[2];
            state[0][j+3] = x[3];
            state[0][j+4] = x[4];
            state[0][j+5] = x[5];
            j += 6; 
        }
    }

    return state;
}

// reinforcement learning is prone to diverge, we set torques to prevent from divergence
// tp = te(1+beta*dq)
// where te is modelled by a bezier curve. You need to pass q, q_off etc without floating base.
template <typename Derived>
auto torque(const Eigen::MatrixBase<Derived>& q, 
            const Eigen::MatrixBase<Derived>& q_off, 
            const Eigen::MatrixBase<Derived>& dq, double q_max, double beta) -> VectorNd
{
    VectorNd tp = VectorNd::Zero(model.dof_count-3); // without floating base

    for (uint i=0;i<tp.size();i++)
    {
        // std::cout << "name: " << model.GetBodyName(ballIds[i+1]) << std::endl;
        int sign = (std::signbit(q[i]-q_off[i]) ? 1 : -1);
        double te = q_max*sign*jointTorqueCurve.calcValue(std::abs(q[i]-q_off[i]));
        tp[i] = te*(1+beta*dq[i]); 
    }
    
    return tp;
}

// Physical system we are describing, here a spider.
auto IncyWincy(const vector_type& x) -> vector_type {

    vector_type dxdt = zero_vector<double>(model.dof_count*2);

    //q
    int j = 0;
    for(unsigned int i=0; i<model.q_size; i++){                
        q[i] = x(j);
        j++;
    }

    //qd
    for(unsigned int i=0; i<model.qdot_size; i++){
        qd[i] = x(j);
        j++;
    }

    // //tau = 0, this is the trigger for the agent
    // double q_max = 0.25;

    // auto act_val = ac->forward(states[c]);
    // actions[c] = std::get<0>(act_val); // thats the bug! pushs back
    // values[c] = std::get<1>(act_val);

    tau.setZero();
    for(int i = 0; i < idx.size(); i++){             
        tau[idx[i]+3] = *(actions[c].data<double>() + i);
        if (tau[idx[i]+3] != tau[idx[i]+3]) {
            notanumber = true;
        }
    }

    int no_float = model.q_size-3; // no floating base
    tau.bottomRows(no_float) += torque(q.bottomRows(no_float), 
                                       q_off.bottomRows(no_float), 
                                       qd.bottomRows(no_float), 0.25, 0.01);

    for (unsigned int i=0; i<ballIds.size(); i++){

        //Calculate the contact forces and populate fext
        r0B0 = CalcBodyToBaseCoordinates(model,q,ballIds[i],Vector3dZero,true);
        ContactToolkit::calcSpherePlaneContactPointPosition(r0B0,ballRadii[i],eN0,r0K0);

        //if the contact point is in the sphere compute contact data
        z = (r0K0-r0P0).dot(eN0);
        dz=0.; //this is zero until contact is made

        if( z < 0. ){
            if (!strcmp(model.GetBodyName(ballIds[i]).c_str(), "Body"))
            {
                reset = true;
            }

            //Get the point of contact resolved in the coordinates of the ball          
            EB0   = CalcBodyWorldOrientation(model,q,ballIds[i],true);
            rBKB  = EB0*(r0K0-r0B0);

            //Get the velocity of the point of contact
            v0K0 = CalcPointVelocity(model,q,qd,ballIds[i],rBKB,true);

            //Evaluate Hunt-Crossley Contact forces
            dz = (v0K0).dot(eN0); //assuming the plane is fixed.
            ContactToolkit::calcHuntCrossleyContactForce(z,dz,stiffness,exponent,damping,hcInfo);

            //Resolve the scalar force into the root frame as a wrench
            fK0n = hcInfo.force*eN0;
            tK0n = VectorCrossMatrix(r0K0)*fK0n;

            //Now go and compute the applied friction forces
            v0K0t = v0K0 - dz*eN0;
            ContactToolkit::calcTangentialVelocityDirection(v0K0t,veps,eT0);
            mu = frictionCoefficientCurve.calcValue(v0K0t.norm());
            fK0t = -mu*hcInfo.force*eT0;
            tK0t = VectorCrossMatrix(r0K0)*fK0t;
            //Apply it to fext
            if (model.IsFixedBodyId(ballIds[i])) { // rbdl performs some reparametrization for fixed bodies, we need to address for that
                fext[model.GetParentBodyId(ballIds[i])][0] += tK0n[0] + tK0t[0];
                fext[model.GetParentBodyId(ballIds[i])][1] += tK0n[1] + tK0t[1];
                fext[model.GetParentBodyId(ballIds[i])][2] += tK0n[2] + tK0t[2];

                fext[model.GetParentBodyId(ballIds[i])][3] += fK0n[0] + fK0t[0];
                fext[model.GetParentBodyId(ballIds[i])][4] += fK0n[1] + fK0t[1];
                fext[model.GetParentBodyId(ballIds[i])][5] += fK0n[2] + fK0t[2];
            }
            else {
                fext[ballIds[i]][0] = tK0n[0] + tK0t[0];
                fext[ballIds[i]][1] = tK0n[1] + tK0t[1];
                fext[ballIds[i]][2] = tK0n[2] + tK0t[2];

                fext[ballIds[i]][3] = fK0n[0] + fK0t[0];
                fext[ballIds[i]][4] = fK0n[1] + fK0t[1];
                fext[ballIds[i]][5] = fK0n[2] + fK0t[2];
            }

        }else{
            //zero the entry of fext associated with the ball.     
            if (!model.IsFixedBodyId(ballIds[i])) {  
                fext[ballIds[i]]=SpatialVector::Zero();
            }
            hcInfo.force        = 0.;
            hcInfo.springForce  = 0.;
            hcInfo.dampingForce = 0.;          
            fK0n = Vector3dZero;
            tK0n = Vector3dZero;
            fK0t = Vector3dZero;
            tK0t = Vector3dZero;
        }
    }

    ForwardDynamics(model,q,qd,tau,qdd,&fext);
    for (auto& f: fext) { // set forces to zero after each iteration
        f.setZero();
    }

    //populate dxdt
    //If you are using a quaternion joint, you must map wx,wy,wz to the 
    //derivatives of the 4 quaternion components using the omegaToQDot 
    //function
    j = 0;
    for(unsigned int i = 0; i < model.q_size; i++){
        dxdt[j] = double(qd[i]);
        if (qd[i] != qd[i]) {
            notanumber = true;
        }
        j++;
    }
    for(unsigned int i = 0; i < model.qdot_size; i++){
        dxdt[j] = double(qdd[i]);
        if (qdd[i] != qdd[i]) {
            notanumber = true;
        }
        j++;
    }

    return dxdt;
};


// Main.
int main(int argc, char** argv) {

    std::string fileName;
    std::string outLoc;
    std::vector<double> v_init(3, 0.);
    uint n_epochs = 0;
    std::string pretrained_model;

    printf("Run with -h to get help.\n");

    for (uint i = 0; i < argc; i++) {

        if (!strcmp(argv[i], "-m")) {

            fileName = argv[i+1];
        }

        if (!strcmp(argv[i], "-v")) {

            v_init[0] = std::stod(argv[i+1]);
            v_init[1] = std::stod(argv[i+2]);
            v_init[2] = std::stod(argv[i+3]);
        }

        if (!strcmp(argv[i], "-o")) {

            outLoc = argv[i+1];
        }

        if (!strcmp(argv[i], "-e")) {

            n_epochs = std::stoi(argv[i+1]);
        }

        if (!strcmp(argv[i], "-p")) {

            pretrained_model =argv[i+1];
        }


        if (!strcmp(argv[i], "-h")) {

            std::cout << "Flags\n" <<
                "for the model with -m /location/of/lua.lua\n" <<
                "for the initial velocity with -v 0.01\n" << 
                "for the output location with -o /output/location/\n" <<
                "for the number of epochs with -e number_of_epochs\n" << 
                "for a pretrained model with -p /location/of/model.pt" <<std::endl;
        }
    }

    if (fileName.empty()) {

        std::cerr << "Please provide the location of the model with the flag -m <location>" << std::endl;
        abort();
    }

    // Load the model.     
    std::vector<std::string> ballNames{"Body",
                                       "FrontalLeftBodyJoint",
                                       "FrontalRightBodyJoint",
                                       "DorsalLeftBodyJoint",
                                       "DorsalRightBodyJoint",
                                       "FrontalLeftUpperJoint",
                                       "FrontalRightUpperJoint",
                                       "DorsalLeftUpperJoint",
                                       "DorsalRightUpperJoint",
                                       "FrontalLeftFoot",
                                       "FrontalRightFoot",
                                       "DorsalLeftFoot",
                                       "DorsalRightFoot"}; // all balls for the contact forces

    if (!Addons::LuaModelReadFromFile(fileName.c_str(),&model)){

        std::cerr << "Error loading LuaModel: " << fileName << std::endl;
        abort();
    }

    for (unsigned int i=0;i<ballNames.size(); ++i) {
        ballIds.push_back(model.GetBodyId(ballNames[i].c_str()));
    }
    ballRadii = {0.25, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
    r0P0 = Vector3d(0.,0.,0.);
    eN0 = Vector3d(0.,0.,1.);

    //Hunt-Crossley contact terms. See
    // ContactToolkit::calcHuntCrossleyContactForce
    //for details
    exponent = 2.0; //The spring force will increase with the deflection squared.
    stiffness = 100.;//9.81/pow(0.01,2.); //The ball will settle to 1cm penetration
    damping = 1.0; //lightly damped  

    //Friction model terms. See
    // ContactToolkit::createRegularizedFrictionCoefficientCurve for details
    staticFrictionSpeed        = 0.001;// 0.008;//0.001;
    staticFrictionCoefficient  = 0.8;// 6.4;  //0.8;
    dynamicFrictionSpeed       = 0.01;// 0.08; //0.01;
    dynamicFrictionCoefficient = 0.6;// 4.8;  //0.6;
    viscousFrictionSlope       = 0.1;// 0.1;  //0.1;
    veps = staticFrictionSpeed/100.0;

    angleAtZeroTorque = 0; // offset for the joint torques is set via calcValue(...)
    dangle = M_PI/4.;
    stiffnessAtLowTorque = 0;      
    stiffnessAtOneNormTorque = 10.;//1.1/std::abs(angleAtZeroTorque-dangle); // minimum possible stiffness
    curviness = 1;

    //If veps is too small, we really might have problems
    assert(veps > std::sqrt(std::numeric_limits<double>::epsilon()));

    q   = VectorNd::Zero(model.dof_count);
    qd  = VectorNd::Zero(model.dof_count);
    qdd = VectorNd::Zero(model.dof_count);
    tau = VectorNd::Zero(model.dof_count);
    vector_type x = zero_vector<double>(model.dof_count*2);

    q_init   = VectorNd::Zero(model.dof_count);
    qd_init  = VectorNd::Zero(model.dof_count);
    qdd_init = VectorNd::Zero(model.dof_count);

    q_off = VectorNd::Zero(model.dof_count);
    // q_off.bottomRows(model.dof_count-3) << M_PI/2., -M_PI/2., -M_PI/2., M_PI/2., M_PI/2., -M_PI/2., -M_PI/2., M_PI/2.;
    idx = {0, 1, 3, 4, 6, 7, 9, 10};
    q_off.bottomRows(model.dof_count-3) << -M_PI/4., M_PI/4., -M_PI/4., M_PI/4., -M_PI/4., M_PI/4., M_PI/4., M_PI/4., -M_PI/4., -M_PI/4., -M_PI/4., M_PI/4.;

    // q[1] = 0.8; //ball starts 0.3m off the ground
    q[1] = CalcBaseToBodyCoordinates(model, q_off, model.GetBodyId(ballNames[ballNames.size()-1].c_str()), Vector3d::Zero())[2]+ballRadii[ballRadii.size()-1];
    // q.bottomRows(model.dof_count-3) << 2.0, -1.5, -2.0, 1.5, 2.0, -1.5, -2.0, 1.5;
    q.bottomRows(model.dof_count-3) = q_off.bottomRows(model.dof_count-3);
    qd[0]= v_init[0];
    qd[1]= v_init[1];
    qd[2]= v_init[2];

    q_init = q;
    qd_init = qd;
    qdd_init = qdd;

    // Get current coordinates.
    y_new = CalcBodyToBaseCoordinates(model, q, model.GetBodyId("Body"), Vector3d::Zero())[1];
    y_old = y_new;

    for(unsigned int i=0; i<q.rows();++i){
        x(i) =q[i];
        x(i+q.rows()) = qd[i];        
    }

    fext = std::vector<SpatialVector>(model.mBodies.size(), SpatialVector::Zero());

    ContactToolkit::createRegularizedFrictionCoefficientCurve(
                        staticFrictionSpeed, staticFrictionCoefficient,
                        dynamicFrictionSpeed,dynamicFrictionCoefficient,
                        viscousFrictionSlope,"mu",frictionCoefficientCurve);

    Addons::Muscle::TorqueMuscleFunctionFactory::createPassiveTorqueAngleCurve(angleAtZeroTorque,
                                                                               angleAtZeroTorque+dangle,
                                                                               stiffnessAtLowTorque,      
                                                                               stiffnessAtOneNormTorque, 
                                                                               curviness, "tp", jointTorqueCurve);

    // ------------------ DEEP LEARNING
    double avg_reward = 0.;
    double best_reward = -std::numeric_limits<double>::max();
    double avg_entropy = 0.;

    uint steps = 1024;
    uint epochs = n_epochs;
    uint mini_batch_size = 256;
    uint ppo_epochs = 4;//uint(steps/mini_batch_size);

    int64_t n_in = model.dof_count*3 + int(fext.size())*6; // q, qd, qdd, fext
    int64_t n_out = 8;//model.dof_count - 3; // control tau
    double std = 1e-4;
    double mu_max = 1e-1;

    ac = ActorCritic(n_in, n_out, mu_max, std); // Cost?
    ac->normal(0., 1e-2);
    ac->to(torch::kFloat64);

    if (!pretrained_model.empty()) {
        torch::load(ac, pretrained_model);
    }

    torch::optim::Adam opt(ac->parameters(), 1e-3);

    // ------------------ USE THE INTEGRATOR SCHEME
    double t = 0.;
    double tp = 0.;
    unsigned int npts = 1e3;

    double dt = 1e-2;

    std::vector<double> rowData(model.dof_count+1);
    std::vector<std::vector< double > > matrixData;

    rowData[0] = 0;
    for(uint z=0; z < model.dof_count; z++){
        rowData[z+1] = x(z);
    }
    matrixData.push_back(rowData);

    c = 0;

    // Store results after each episode.
    std::string emptyHeader("");

    std::ofstream out;
    out.open(outLoc + "reward_and_entropy.csv");

    Timer(START);
    for (uint e=0;e<epochs;e++)
    {
        printf("epoch %d/%d\n", e+1, epochs);

        for(uint i=0; i <= npts; i++){

            printf("\riter %d/%d", i, npts);

            // Get current state of the environment.
            states.push_back(to_state(q, qd, qdd, fext, true));

            // printf("iteration %d/%d\n", i, npts);

            t += dt;

            // actions.push_back(torch::zeros({1,n_out}, torch::kF64));
            // values.push_back(torch::zeros({1,1}, torch::kF64));    //tau = 0, this is the trigger for the agent
            auto act_val = ac->forward(states[c]);
            actions.push_back(std::get<0>(act_val)); // thats the bug! pushs back
            values.push_back(std::get<1>(act_val));

            size_t num_of_steps = integrate_const(make_dense_output< runge_kutta_dopri5< vector_type > >( 1e-3 , 1e-3),
                                                  System(IncyWincy), // within the system we want the agent to perform actions
                                                  x, tp, t, dt); // fast and explicit

            if (notanumber) 
            {
                c = 0;

                states.clear();
                actions.clear();
                rewards.clear();
                dones.clear();

                log_probs.clear();
                values.clear();
                returns.clear();

                printf("\nNan encountered, stopping current epoch.\n");
                break; 
            }
            else 
            {

                // Get the new coordinates.
                y_new = CalcBodyToBaseCoordinates(model, q, model.GetBodyId("Body"), Vector3d::Zero())[1];

                // Insert state, action, reward, next_state and done into memory.
                v_y = CalcPointVelocity(model,q,qd,model.GetBodyId("Body"),Vector3d::Zero(),true)[1];
                n_z = (CalcBodyWorldOrientation(model, q, model.GetBodyId("Body"), false)*eN0).transpose()*eN0;
                rewards.push_back(compute_reward(v_y, n_z, tau, reset));
                // rewards.push_back(compute_reward(y_old, y_new, n_z, tau, reset));
                dones.push_back(torch::full({1,1}, (reset ? 1. : 0.), torch::kF64));

                avg_reward += *(rewards[c].data<double>())/npts;
                avg_entropy += *(ac->entropy().data<double>())/npts;

                y_old = y_new;

                log_probs.push_back(ac->log_prob(actions[c]));

                c++;

                if (c%steps==0)
                {
                    printf("\nUpdating network.\n");
                    values.push_back(std::get<1>(ac->forward(states[c-1])));

                    returns = PPO::returns(rewards, dones, values, .8, .95);

                    torch::Tensor rets = torch::cat(returns).detach();
                    torch::Tensor logs = torch::cat(log_probs).detach();
                    torch::Tensor vals = torch::cat(values).detach();
                    torch::Tensor stas = torch::cat(states);
                    torch::Tensor acts = torch::cat(actions);
                    torch::Tensor advs = rets - vals.slice(0, 0, steps); // size mismatch between val and ret cause of next val

                    PPO::update(ac, stas, acts, logs, rets, advs, opt, steps, ppo_epochs, mini_batch_size, 1e-3); // higher entropy factor, for less loss??

                    c = 0;

                    states.clear();
                    actions.clear();
                    rewards.clear();
                    dones.clear();

                    log_probs.clear();
                    values.clear();
                    returns.clear();
                }

                tp = t;

                rowData[0] = t;
                for(uint z=0; z < model.dof_count; z++){
                    rowData[z+1] = x(z);
                }

                matrixData.push_back(rowData);

                // Reset the environment.
                if (reset)
                {
                    q = q_init;
                    qd = qd_init;
                    qdd = qdd_init;

                    for(unsigned int j=0; j<q.rows();++j){
                        x(j) =q_init[j];
                        x(j+q.rows()) = qd_init[j];        
                    }

                    t += dt;

                    rowData[0] = t;
                    for(uint z=0; z < model.dof_count; z++){
                        rowData[z+1] = x(z);
                    }
                    matrixData.push_back(rowData);
                    reset = false;
                    reset_episode = false;
                }
            }
        }

        if (!notanumber) {

            if (avg_reward > best_reward)
                {
                    best_reward = avg_reward;
                    torch::save(ac, "models/best_model_at_epoch_" + std::to_string(e+1) + ".pt");
                }

                out << e+1 << ", " << avg_reward << ", " << avg_entropy << "\n";

                std::string fileNameOut(outLoc + "animation_epoch_" + std::to_string(e+1) + ".csv");   // ii implicit integrator
                printMatrixToFile(matrixData,emptyHeader,fileNameOut);
        }

        printf("\naverage reward %f\n", avg_reward);

        notanumber = false;

        avg_reward = 0.;
        avg_entropy = 0.;

        t = 0.;

        matrixData.clear();

        // Reset after each epoch.
        q = q_init;
        qd = qd_init;
        qdd = qdd_init;

        for(unsigned int j=0; j<q.rows();++j){
            x(j) =q_init[j];
            x(j+q.rows()) = qd_init[j];        
        }

        rowData[0] = t;
        for(uint z=0; z < model.dof_count; z++){
            rowData[z+1] = x(z);
        }
        matrixData.push_back(rowData);
    }
    auto time = Timer(STOP);
    printf("Elapsed time: %.2f s\n", time/1000.);

    out.close();

    return 0;
}
