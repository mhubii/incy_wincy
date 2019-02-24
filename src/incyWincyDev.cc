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

#include "csvtools.h"
#include "ContactToolkit.h"

using namespace std::chrono;
using namespace boost::numeric::ublas;
using namespace boost::numeric::odeint;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;


// Speed benchmarking :).
enum {
    START,
    STOP
};

auto Timer(int set) -> double {

    double msec;

    // Set a timer.
    switch (set)
    {
        case START:
            static auto t1 = high_resolution_clock::now();
            msec = 0.;
            break;
        case STOP:
            static auto t2 = high_resolution_clock::now();
            msec = duration_cast<milliseconds>(t2 - t1).count();
            break;
        default:
            break;
    }
    return msec;
}


// Some typedefs.
using vector_type = vector< double >;
using matrix_type = matrix< double >;


// Function pointer of the system we want to describe.
using system_func_ptr = auto(*)(const vector_type&) -> vector_type;


// Physical description of my system.
class System
{
public:
    System(system_func_ptr system) : system_(system) {  }; // takes a function pointer to the physical description

    auto operator() (const vector_type& x, vector_type& dxdt, double t) -> void {

        dxdt = system_(x);
    }; // Performs a function call on system function pointer

    // Physics of the system are described by this function pointer.
    system_func_ptr system_;
};


// Need a function template that takes in the forward dynamics of my system and returns me the numerical jacobian.
// It would therefore be easiest to have a simple system, like the bouncing ball.
// I will implement a simple test that compares the numerical jacobian of the bouncing ball with that of the analytical.


class NumericalJacobian : public System
{
public:
    NumericalJacobian(system_func_ptr system) : System(system) {   };

    auto operator() (const vector_type& x, matrix_type& J , const double& t, vector_type& dfdt) -> void {

        // Determine the Jacobian.
        uint c = x.size();

        for (uint i = 0; i < c; i++) {

            column(J, i) = 0.5/h_*(system_(x + unit_vector<double>(c, i)*h_) - system_(x - unit_vector<double>(c, i)*h_)); // compute finite differences
        }
    };

    double h_ = std::sqrt(std::numeric_limits<double>::min());
}; // for rosenbrock


/*
class NumericalJacobian : public System
{
public:
    NumericalJacobian(system_func_ptr system) : System(system) {   };

    auto operator() (const vector_type& x, matrix_type& J , const double& t) -> void {

        // Determine the Jacobian.
        uint c = x.size();

        for (uint i = 0; i < c; i++) {

            column(J, i) = 0.5/h_*(system_(x + unit_vector<double>(c, i)*h_) - system_(x - unit_vector<double>(c, i)*h_)); // compute finite differences
        }
    };

    double h_ = std::sqrt(std::numeric_limits<double>::min());
}; // for implicit euler
*/






// Here comes the actual physical model of the system we want to describe, see Matt's implementation.
// This function needs to be of system_func_ptr type.
Model model;

VectorNd q, qd, qdd, tau;

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

    //tau = 0, this is the trigger for the agent
    for(unsigned int i=0; i<model.qdot_size; i++){                
        tau[i] = 0; // agent perform an action, sth like agent.act(x), then integrate somehow and return the reward for the action
    }

    for (unsigned int i=0; i<ballIds.size(); i++){

        //Calculate the contact forces and populate fext
        r0B0 = CalcBodyToBaseCoordinates(model,q,ballIds[i],Vector3dZero,true);
        ContactToolkit::calcSpherePlaneContactPointPosition(r0B0,ballRadii[i],eN0,r0K0);

        //if the contact point is in the sphere compute contact data
        z = (r0K0-r0P0).dot(eN0);
        dz=0.; //this is zero until contact is made
        if( z < 0. ){
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
        j++;
    }
    for(unsigned int i = 0; i < model.qdot_size; i++){
        dxdt[j] = double(qdd[i]);
        j++;
    }

    return dxdt;
};


// Main.
int main(int argc, char** argv) {

    std::string fileName;
    std::string outLoc;
    std::vector<double> v_init(3, 0.);

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

        if (!strcmp(argv[i], "-h")) {

            std::cout << "Flags\n" <<
                "for the model with -m /location/of/lua.lua\n" <<
                "for the initial velocity with -v 0.01" << 
                "for the output location with -o /output/location/" << std::endl;
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
    staticFrictionSpeed        = 0.001;
    staticFrictionCoefficient  = 0.8;
    dynamicFrictionSpeed       = 0.01;
    dynamicFrictionCoefficient = 0.6;
    viscousFrictionSlope       = 0.1;
    veps = staticFrictionSpeed/100.0;

    //If veps is too small, we really might have problems
    assert(veps > std::sqrt(std::numeric_limits<double>::epsilon()));

    q   = VectorNd::Zero(model.dof_count);
    qd  = VectorNd::Zero(model.dof_count);
    qdd = VectorNd::Zero(model.dof_count);
    tau = VectorNd::Zero(model.dof_count);
    vector_type x = zero_vector<double>(model.dof_count*2);

    q[1] = 2.; //ball starts 1m off the ground
    qd[0]= v_init[0];
    qd[1]= v_init[1];
    qd[2]= v_init[2];

    //bend the legs sligthly
    for(unsigned int i=3;i<q.rows();++i){
        q[i] += 0.1;    
    }

    for(unsigned int i=0; i<q.rows();++i){
        x(i) =q[i];
        x(i+q.rows()) = qd[i];
    }

    fext = std::vector<SpatialVector>(model.mBodies.size(), SpatialVector::Zero());

    ContactToolkit::createRegularizedFrictionCoefficientCurve(
                        staticFrictionSpeed, staticFrictionCoefficient,
                        dynamicFrictionSpeed,dynamicFrictionCoefficient,
                        viscousFrictionSlope,"mu",frictionCoefficientCurve);

    // ------------------ USE THE INTEGRATOR SCHEME
    double t;
    double t0 = 0.;
    double t1 = 8.0;
    double tp = 0.;
    unsigned int npts = 100;

    double dt = (t1-t0)/(npts-1);

    std::vector<double> rowData(model.dof_count+1);
    std::vector<std::vector< double > > matrixData;

    rowData[0] = 0;
    for(uint z=0; z < model.dof_count; z++){
        rowData[z+1] = x(z);
    }
    matrixData.push_back(rowData);

    Timer(START);
    for(uint i=0; i <= npts; i++){

        printf("iteration %d/%d\n", i, npts);

        t = t0 + dt*i;

        // size_t num_of_steps = integrate_const(make_dense_output< rosenbrock4< double > >( 1.0e-6 , 1.0e-6 ),
        //                                       std::make_pair( System(IncyWincy) , NumericalJacobian(IncyWincy) ) ,
        //                                       x, tp, t, dt); // way too sloooow
        // size_t num_of_steps = integrate_const(make_dense_output< rosenbrock4< double > >( 1.0e-1 , 1.0e-1 ),
        //                                       std::make_pair( System(IncyWincy) , NumericalJacobian(IncyWincy) ) ,
        //                                       x, tp, t, dt); // little faster, still too slow
        // size_t num_of_steps = integrate_const(implicit_euler < double >(),
        //                                       std::make_pair( System(IncyWincy) , NumericalJacobian(IncyWincy) ) ,
        //                                       x, tp, t, dt); // diverges on impact
        size_t num_of_steps = integrate_const(make_dense_output< runge_kutta_dopri5< vector_type > >( 5.0e-2 , 5.0e-2 ),
                                              System(IncyWincy),
                                              x, tp, t, dt); // fast and explicit

        tp = t;

        rowData[0] = t;
        for(uint z=0; z < model.dof_count; z++){
            rowData[z+1] = x(z);
        }

        matrixData.push_back(rowData);
    }
    auto time = Timer(STOP);
    printf("Elapsed time: %f\n", time);


    // Store results.
    std::string emptyHeader("");
    std::ostringstream stream;
    stream << std::setprecision(1);
    stream << v_init[0] << "_" << v_init[1] << "_" << v_init[2];
    std::string fileNameOut(outLoc + "animation_ii_vinit_" + stream.str() + ".csv");   // ii implicit integrator
    printMatrixToFile(matrixData,emptyHeader,fileNameOut);

    // Output time and initial velocity.
    std::ofstream out;
    out.open(outLoc + "time_ii.csv", std::ios_base::app);
    out << v_init[0] << ", " << v_init[1] << ", " << v_init[2] << ", " << time << "\n";
    out.close();

    // ------------------ SYNTAX EXPERIMENTS

    // double h = std::sqrt(std::numeric_limits<double>::min());

    // // find out how ublas matrices can be manipulated
    // vector_type vector1 = scalar_vector<double>(2, 4.);
    // vector_type vector2 = scalar_vector<double>(2, 4.);
    // vector_type vector3 = zero_vector<double>(2);
    // vector_type vector4 = unit_vector<double>(3, 1)*h;
    // matrix_type matrix = zero_matrix<double>(2, 2);

    // column(matrix, 0) = vector1;
    // vector3 = vector1 + vector2;
    // std::cout << vector3 << std::endl;
    // std::cout << vector3.size() << std::endl;
    // std::cout << vector4 << std::endl;

    return 0;
}
