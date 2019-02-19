#include <iostream>

#include <boost/numeric/odeint.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <rbdl/rbdl.h>
#include <rbdl/addons/luamodel/luamodel.h>
#include <rbdl/addons/geometry/geometry.h>

#include "csvtools.h"
#include "ContactToolkit.h"

using namespace boost::numeric::ublas;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

// Some typedefs.
using vector_type = vector< double >;
using matrix_type = matrix< double >;


// Function pointer of the system we want to describe.
using system_func_ptr = auto(*)(const vector_type&, double) -> vector_type;


// Physical description of my system.
class System
{
public:
    System(system_func_ptr system) : system_(system) {  }; // takes a function pointer to the physical description

    auto operator() (const vector_type& x, vector_type& dxdt, double t) -> void {

        dxdt = system_(x, t);
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

            column(J, i) = 0.5/h_*(system_(x + unit_vector<double>(c, i)*h_, t) - system_(x - unit_vector<double>(c, i)*h_, t)); // compute finite differences
        }
    };

    double h_ = std::sqrt(std::numeric_limits<double>::min());
};







// Here comes the actual physical model of the system we want to describe, see Matt's implementation.
// This function needs to be of system_func_ptr type.
Model model;

VectorNd q, qd, qdd, tau;

Vector3d r0P0;
Vector3d eN0;

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
uint ballId;
double ballRadius;

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

auto BouncingBall(vector_type& x, double t) -> vector_type {

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

    //tau = 0
    for(unsigned int i=0; i<model.qdot_size; i++){                
        tau[i] = 0;
    }

    //Calculate the contact forces and populate fext
    r0B0 = CalcBodyToBaseCoordinates(model,q,ballId,Vector3dZero,true);
    ContactToolkit::calcSpherePlaneContactPointPosition(r0B0,ballRadius,eN0,r0K0);

    //if the contact point is in the sphere compute contact data
    z = (r0K0-r0P0).dot(eN0);
    dz=0.; //this is zero until contact is made
    if( z < 0. ){
        //Get the point of contact resolved in the coordinates of the ball          
        EB0   = CalcBodyWorldOrientation(model,q,ballId,true);
        rBKB  = EB0*(r0K0-r0B0);

        //Get the velocity of the point of contact
        v0K0 = CalcPointVelocity(model,q,qd,ballId,rBKB,true);

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
        fext[ballId][0] = tK0n[0] + tK0t[0];
        fext[ballId][1] = tK0n[1] + tK0t[1];
        fext[ballId][2] = tK0n[2] + tK0t[2];

        fext[ballId][3] = fK0n[0] + fK0t[0];
        fext[ballId][4] = fK0n[1] + fK0t[1];
        fext[ballId][5] = fK0n[2] + fK0t[2];

    }else{
        //zero the entry of fext associated with the ball.          
        fext[ballId]=SpatialVector::Zero();
        hcInfo.force        = 0.;
        hcInfo.springForce  = 0.;
        hcInfo.dampingForce = 0.;          
        fK0n = Vector3dZero;
        tK0n = Vector3dZero;
        fK0t = Vector3dZero;
        tK0t = Vector3dZero;
    }

    ForwardDynamics(model,q,qd,tau,qdd,&fext);

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

    for (uint i = 0; i < argc; i++) {

        if (!strcmp(argv[i], "-m")) {

            fileName = argv[i+1];
        }
    }

    if (fileName.empty()) {

        std::cerr << "Please provide the location of the model with the flag -m <location>" << std::endl;
        abort();
    }

    // Load the model.     
    std::string ballName("Ball");

    if (!Addons::LuaModelReadFromFile(fileName.c_str(),&model)){

        std::cerr << "Error loading LuaModel: " << fileName << std::endl;
        abort();
    }

    ballId = model.GetBodyId(ballName.c_str());
    ballRadius = 0.5;
    r0P0 = Vector3d(0.,0.,0.);
    eN0 = Vector3d(0.,0.,1.);

    //Hunt-Crossley contact terms. See
    // ContactToolkit::calcHuntCrossleyContactForce
    //for details
    exponent = 2.0; //The spring force will increase with the deflection squared.
    stiffness = 9.81/pow(0.01,2.); //The ball will settle to 1cm penetration
    damping = 0.1; //lightly damped  

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

    unsigned int numWorkTermsInState = 2; //1 normal work term
                                          //1 friction term

    q   = VectorNd::Zero(model.dof_count);
    qd  = VectorNd::Zero(model.dof_count);
    qdd = VectorNd::Zero(model.dof_count);
    tau = VectorNd::Zero(model.dof_count);

    q[1] = 1.; //ball starts 1m off the ground
    qd[0]= 1.;

    // for(unsigned int i=0; i<q.rows();++i){
    //     x[i] =q[i];
    //     x[i+q.rows()] = qd[i];
    // }

    fext = std::vector<SpatialVector>(model.mBodies.size(), SpatialVector::Zero());

    ContactToolkit::createRegularizedFrictionCoefficientCurve(
                        staticFrictionSpeed, staticFrictionCoefficient,
                        dynamicFrictionSpeed,dynamicFrictionCoefficient,
                        viscousFrictionSlope,"mu",frictionCoefficientCurve);


    // USE THE INTEGRATOR SCHEME






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