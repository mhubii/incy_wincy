/*====================================================================
 * cam-plane pendulum example
 * Copyright (c) 2015 Matthew Millard 
 * <matthew.millard@iwr.uni-heidelberg.de>
 *
 *///=================================================================


#include <string>
#include <iostream>
#include <ctime>
#include <fstream>
#include <stdio.h> 
#include <rbdl/rbdl.h>
#include <rbdl/addons/luamodel/luamodel.h>
#include <rbdl/addons/geometry/geometry.h>
#include "csvtools.h"

#include "ContactToolkit.h"

#include <boost/numeric/odeint/stepper/runge_kutta_cash_karp54.hpp>
#include <boost/numeric/odeint/stepper/controlled_runge_kutta.hpp>
#include <boost/numeric/odeint/integrate/integrate_adaptive.hpp>
#include <boost/numeric/odeint/stepper/generation/make_controlled.hpp>
//using namespace std;
using namespace boost::numeric::odeint;

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

//====================================================================
// Boost stuff
//====================================================================




typedef std::vector< double > state_type;
typedef runge_kutta_cash_karp54< state_type > error_stepper_type;
typedef controlled_runge_kutta< error_stepper_type > controlled_stepper_type;


class rbdlToBoost {

  public:
    rbdlToBoost(Model* model,
                std::string &ballName,
                double ballRadius,
                Vector3d &pointOnPlane, 
                Vector3d &planeNormal,
                double stiffness,
                double exponent,
                double damping,
                double staticFrictionSpeed,
                double staticFrictionCoefficient,
                double dynamicFrictionSpeed,
                double dynamicFrictionCoefficient,
                double viscousFrictionSlope,
                unsigned int numberOfWorkTermsInState
                ):model(model),r(ballRadius),r0P0(pointOnPlane),
                  eN0(planeNormal),k(stiffness),p(exponent),beta(damping),
                  staticFrictionSpeed(staticFrictionSpeed),
                  staticFrictionCoefficient(staticFrictionCoefficient),
                  dynamicFrictionSpeed(dynamicFrictionSpeed),
                  dynamicFrictionCoefficient(dynamicFrictionCoefficient),
                  viscousFrictionSlope(viscousFrictionSlope),
                  numberOfWorkTermsInState(numberOfWorkTermsInState)
    {

        q = VectorNd::Zero(model->dof_count);
        qd = VectorNd::Zero(model->dof_count);
        qdd = VectorNd::Zero(model->dof_count);
        tau = VectorNd::Zero(model->dof_count);
        fext.resize(model->mBodies.size());
        for(unsigned int i=0; i<fext.size();++i){
            fext[i]=SpatialVector::Zero();
        }
        printf("number of bodies: %d\n", (int)model->mBodies.size());
        ballId = model->GetBodyId(ballName.c_str());

        printf("ball name %s, ball id %d\n", model->GetBodyName(ballId).c_str(), ballId);
        fK0n = Vector3dZero;
        tK0n = Vector3dZero;
        fK0t = Vector3dZero;
        tK0t = Vector3dZero;

        ContactToolkit::createRegularizedFrictionCoefficientCurve(
                          staticFrictionSpeed, staticFrictionCoefficient,
                          dynamicFrictionSpeed,dynamicFrictionCoefficient,
                          viscousFrictionSlope,"mu",frictionCoefficientCurve);

        frictionCoefficientCurve.printCurveToCSVFile("",
                           "frictionCoefficientCurve",0,dynamicFrictionSpeed*2);
        //Set the velocity at which the relaxed method is used to compute
        //the direction vector of the tangential velocity of the contact
        //point
        veps = staticFrictionSpeed/100.0;

        //If veps is too small, we really might have problems
        assert(veps > std::sqrt(std::numeric_limits<double>::epsilon()));


    }

    void operator() (const state_type &x,
                     state_type &dxdt, 
                     const double t){

        //q
        int j = 0;
        for(unsigned int i=0; i<model->q_size; i++){                
            q[i] = double(x[j]);
            j++;
        }

        //qd
        for(unsigned int i=0; i<model->qdot_size; i++){
            qd[i] = double(x[j]);
            j++;
        }

        //tau = 0
        for(unsigned int i=0; i<model->qdot_size; i++){                
            tau[i] = 0;
        }

        //Calculate the contact forces and populate fext
        r0B0 = CalcBodyToBaseCoordinates(*model,q,ballId,Vector3dZero,true);
        ContactToolkit::calcSpherePlaneContactPointPosition(r0B0,r,eN0,r0K0);

        //if the contact point is in the sphere compute contact data
        z = (r0K0-r0P0).dot(eN0);
        dz=0.; //this is zero until contact is made
        if( z < 0. ){
          //Get the point of contact resolved in the coordinates of the ball          
          EB0   = CalcBodyWorldOrientation(*model,q,ballId,true);
          rBKB  = EB0*(r0K0-r0B0);

          //Get the velocity of the point of contact
          v0K0 = CalcPointVelocity(*model,q,qd,ballId,rBKB,true);

          //Evaluate Hunt-Crossley Contact forces
          dz = (v0K0).dot(eN0); //assuming the plane is fixed.
          ContactToolkit::calcHuntCrossleyContactForce(z,dz,k,p,beta,hcInfo);

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

        ForwardDynamics(*model,q,qd,tau,qdd,&fext);

        //populate dxdt
        //If you are using a quaternion joint, you must map wx,wy,wz to the 
        //derivatives of the 4 quaternion components using the omegaToQDot 
        //function
        j = 0;
        for(unsigned int i = 0; i < model->q_size; i++){
            dxdt[j] = double(qd[i]);
            j++;
        }
        for(unsigned int i = 0; i < model->qdot_size; i++){
            dxdt[j] = double(qdd[i]);
            j++;
        }

        dworkN = hcInfo.force*dz;
        dxdt[j] = dworkN;
        j++;
        dxdt[j] = fK0t.dot(v0K0t);
        j++;
        assert((numberOfWorkTermsInState
                + model->q_size 
                + model->qdot_size) == j);

    }

    /*
      Ascii Vector Notation:
      rBKB
        r: vector
        B: from the origin of the Ball frame
        K: to the contact point K
        B: expressed in the coordinates of the ball frame.
      EB0:
        E: rotation matrix
        B: to Frame B
        0: from Frame 0
      eN0:
        e: unit vector
        N: Normal direction
        0: expressed in the coordinates of the root frame (0)
      fK0
        f: force
        K: At point K
        0: expressed in the coordinates of the root frame
      tK0
        t: torque
        K: At point K
        0: expressed in the coordinates of the root frame

    */

    //Multibody Variables
    Model* model;
    VectorNd q, qd, qdd, tau;    

    //Normal-Contact Model Working Variables
    unsigned int ballId;
    double z, dz; //pentration depth and velocity
    HuntCrossleyContactInfo hcInfo;
    std::vector< SpatialVector > fext;
    double dworkN, dworkT; //Work in the normal and tangential directions
    double mu; //Friction coefficient
    Matrix3d EB0;
    Vector3d r0B0; //Position of the ball
    Vector3d rBKB; //B : ball.
    Vector3d r0K0; //K : contact point
    Vector3d v0K0; //velocity of the contact point
    Vector3d v0K0t; //tangential velocity of the contact point
    Vector3d r0P0; //Origin of the plane
    Vector3d eN0;  //Normal of the plane
    Vector3d eT0; //Tangental direction of the plane: in 2d this isn't necessary
                  //here we compute it to show how this is done in a
                  //numerically stable way in 3d.
    Vector3d fK0n, tK0n; //contact force and moment
    Vector3d fK0t, tK0t; //tangential friction force and moment

    //Normal Contact-Model Parameters
    double r; //ball radius
    double k; //stiffness
    double p; //exponential power on the spring compression
    double beta; //damping

    //Friction Model Parameters
    double staticFrictionSpeed;
    double staticFrictionCoefficient;
    double dynamicFrictionSpeed;
    double dynamicFrictionCoefficient;
    double viscousFrictionSlope;
    double veps; //Velocity at which the relaxed method is used to compute
                 //the direction vector of the tangential velocity
    RigidBodyDynamics::Addons::Geometry
        ::SmoothSegmentedFunction frictionCoefficientCurve;

    unsigned int numberOfWorkTermsInState;

};

struct pushBackStateAndTime
{
    std::vector< state_type >& states;
    std::vector< double >& times;

    pushBackStateAndTime( std::vector< state_type > &states , 
                              std::vector< double > &times )
    : states( states ) , times( times ) { }

    void operator()( const state_type &x , double t )
    {
        states.push_back( x );
        times.push_back( t );
    }
};

void f(const state_type &x, state_type &dxdt, const double t);

/* Problem Constants */
// Main.
int main(int argc, char** argv) {

    std::string fileName;
    std::string outLoc;
    double v_init;

    printf("Run with -h to get help.\n");

    for (uint i = 0; i < argc; i++) {

        if (!strcmp(argv[i], "-m")) {

            fileName = argv[i+1];
        }

        if (!strcmp(argv[i], "-v")) {

            v_init = std::stod(argv[i+1]);
        }


        if (!strcmp(argv[i], "-o")) {

            outLoc = argv[i+1];
        }

        if (!strcmp(argv[i], "-h")) {

            std::cout << "Flags\n" <<
                "for the model with -m /location/of/lua.lua\n" <<
                "for the output location with -o /output/location" << std::endl;
        }
    }
  
    rbdl_check_api_version (RBDL_API_VERSION);

  RigidBodyDynamics::Model model;

  std::string ballName("Ball");

  if (!Addons::LuaModelReadFromFile(fileName.c_str(),&model)){
    std::cerr << "Error loading LuaModel: " << fileName << std::endl;
    abort();
  }

  double radius = 0.5;
  Vector3d pointOnPlane = Vector3d(0.,0.,0.);
  Vector3d planeNormal  = Vector3d(0.,0.,1.);

  //Hunt-Crossley contact terms. See
  // ContactToolkit::calcHuntCrossleyContactForce
  //for details
  double exponent = 2.0; //The spring force will increase with the deflection squared.
  double stiffness = 9.81/pow(0.01,2.); //The ball will settle to 1cm penetration
  double damping = 0.1; //lightly damped  

  //Friction model terms. See
  // ContactToolkit::createRegularizedFrictionCoefficientCurve for details

  double staticFrictionSpeed        = 0.001;
  double staticFrictionCoefficient  = 0.8;
  double dynamicFrictionSpeed       = 0.01;
  double dynamicFrictionCoeffient   = 0.6;
  double viscousFrictionSlope       = 0.1;

  unsigned int numWorkTermsInState = 2; //1 normal work term
                                        //1 friction term


  VectorNd q, qd, x, tau;
  q.resize(model.dof_count);
  qd.resize(model.dof_count);
  tau.resize(model.dof_count);
  x.resize(model.dof_count*2);
  q.setZero();
  qd.setZero();
  x.setZero();
  tau.setZero();

  q[1] = 1.; //ball starts 1m off the ground
  qd[0]= v_init;

  for(unsigned int i=0; i<q.rows();++i){
    x[i] =q[i];
    x[i+q.rows()] = qd[i];
  }

  rbdlToBoost rbdlModel(&model,
                        ballName,
                        radius,
                        pointOnPlane,
                        planeNormal,
                        stiffness,
                        exponent,
                        damping,
                        staticFrictionSpeed,
                        staticFrictionCoefficient,
                        dynamicFrictionSpeed,
                        dynamicFrictionCoeffient,
                        viscousFrictionSlope,
                        numWorkTermsInState);

    state_type xState(x.size()+numWorkTermsInState);
    state_type dxState(x.size()+numWorkTermsInState);
    for(unsigned int i=0; i<x.size(); ++i){
      xState[i]   = x[i];
    }

    double t;
    double t0 = 0;
    double t1 = 1.0;
    unsigned int npts      = 100;

    double absTolVal = 1e-8;
    double relTolVal = 1e-8;

    double dt = (t1-t0)/(npts-1);
    double ke,pe,w =0;
    unsigned int k=0;

    std::vector<std::vector< double > > matrixData, matrixForceData;
    std::vector<std::vector< double > > matrixErrorData;
    std::vector< double > rowData(model.dof_count+1);
    std::vector< double > rowForceData(10);
    std::vector< double > rowErrorData(2);

    double a_x = 1.0 , a_dxdt = 1.0;
    controlled_stepper_type
    controlled_stepper(
        default_error_checker< double ,
                               range_algebra ,
                               default_operations >
        ( absTolVal , relTolVal , a_x , a_dxdt ) );

    double tp = 0;
    rowData[0] = 0;
    for(unsigned int z=0; z < model.dof_count; z++){
        rowData[z+1] = xState[z];
    }
    matrixData.push_back(rowData);

    SpatialVector fA0 = SpatialVector::Zero();
    for(unsigned int i=0; i<rowForceData.size();++i){
      rowForceData[i]=0.;
    }

    matrixForceData.push_back(rowForceData);

    double kepe0 = 0;
    double th,dth;

    //                 ,             ,             ,             ,             ,             ,             ,
    printf("          t,        theta,   d/dt theta,           ke,           pe,            w, ke+pe-w-kepe0\n");

    clock_t begin = clock();
    for(unsigned int i=0; i<= npts; ++i){
      t = t0 + dt*i;

      integrate_adaptive(
          controlled_stepper ,
          rbdlModel , xState , tp , t , (t-tp)/10 );
      tp = t;


      for(unsigned int j=0; j<x.rows();++j){
        x[j] = xState[j];
      }
      k=0;
      for(unsigned int j=0; j<model.q_size;++j){
        q[j] = xState[k];
        ++k;
      }
      for(unsigned int j=0; j<model.qdot_size;++j){
        qd[j] = xState[k];
        ++k;
      }
      w = 0.;
      for(unsigned int j=0; j<numWorkTermsInState;++j){
        w += xState[k];
        ++k;
      }


      pe = Utils::CalcPotentialEnergy(model,
                                      q,true);

      ke = Utils::CalcKineticEnergy(model,
                                    q,
                                    qd,true);

      rowData[0] = t;
      for(unsigned int z=0; z < model.dof_count; z++){
          rowData[z+1] = xState[z];
      }
      matrixData.push_back(rowData);

      if(i==0) kepe0 = (ke+pe-w);

      rowErrorData[0] = t;
      rowErrorData[1] = (ke + pe -w) - kepe0;
      matrixErrorData.push_back(rowErrorData);

      printf("%e, %e, %e, %e, %e, %e, %e\n",
                  t, q[2],qd[2],ke,
                  pe,w,(ke+pe-w-kepe0));

      //Make sure the model state is up to date.
      rbdlModel(xState,dxState,tp);

      rowForceData[0] = t;
      //contact point location
      rowForceData[1] = rbdlModel.r0K0[0];
      rowForceData[2] = rbdlModel.r0K0[1];
      rowForceData[3] = rbdlModel.r0K0[2];
      //force at the contact point
      rowForceData[4] = rbdlModel.fK0n[0] + rbdlModel.fK0t[0];
      rowForceData[5] = rbdlModel.fK0n[1] + rbdlModel.fK0t[1];
      rowForceData[6] = rbdlModel.fK0n[2] + rbdlModel.fK0t[2];
      //moment at the contact point
      rowForceData[7] = 0.;//This contact model generates no contact moments
      rowForceData[8] = 0.;//This contact model generates no contact moments
      rowForceData[9] = 0.;//This contact model generates no contact moments

      matrixForceData.push_back(rowForceData);


      bool here=true;

    }
    clock_t end = clock();
    double sec = (end - begin)/CLOCKS_PER_SEC;
    std::cout << std::endl;
    std::string emptyHeader("");
    std::string fileNameOut("animation.csv");
    printMatrixToFile(matrixData,emptyHeader,fileNameOut);
    fileNameOut.assign("animationForces.ff");
    printMatrixToFile(matrixForceData,emptyHeader,fileNameOut);

    fileNameOut = "kepe.csv";
    printMatrixToFile(matrixErrorData,emptyHeader,fileNameOut);

    // Output time and initial velocity.
    std::ofstream out;
    out.open(outLoc + "time.csv", std::ios_base::app);
    out << v_init << ", " << std::to_string(sec) << "\n";
    out.close();



   return 0;
        
}
