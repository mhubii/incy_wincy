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

#include "IncyWincyDev.h"
#include "Timer.h"

using namespace boost::numeric::ublas;
using namespace boost::numeric::odeint;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

int main(int argc, char** argv)
{
    // ------------------------ READ IN STUFF
    std::string fileName;
    std::string outLoc;
    uint n_epochs = 0;

    printf("Run with -h to get help.\n");

    for (uint i = 0; i < argc; i++) {

        if (!strcmp(argv[i], "-m")) {

            fileName = argv[i+1];
        }

        if (!strcmp(argv[i], "-o")) {

            outLoc = argv[i+1];
        }

        if (!strcmp(argv[i], "-e")) {

            n_epochs = std::stoi(argv[i+1]);
        }


        if (!strcmp(argv[i], "-h")) {

            std::cout << "Flags\n" <<
                "for the model with -m /location/of/lua.lua\n" <<
                "for the initial velocity with -v 0.01\n" << 
                "for the output location with -o /output/location/\n" <<
                "for the number of epochs with -e number_of_epochs" << std::endl;
        }
    }

    if (fileName.empty()) {

        std::cerr << "Please provide the location of the model with the flag -m <location>" << std::endl;
        abort();
    }

    // ------------------------ LOAD MODEL  
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

    Model model;

    if (!Addons::LuaModelReadFromFile(fileName.c_str(),&model)){

        std::cerr << "Error loading LuaModel: " << fileName << std::endl;
        abort();
    }

    std::vector<uint> ballIds;
    for (unsigned int i=0;i<ballNames.size(); ++i) {
        ballIds.push_back(model.GetBodyId(ballNames[i].c_str()));
    }
    std::vector<double> ballRadii{0.25, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};

    vector_type x = zero_vector<double>(model.dof_count*2);

    VectorNd q_init   = VectorNd::Zero(model.dof_count);
    q_init[1] = CalcBaseToBodyCoordinates(model, q_init, model.GetBodyId(ballNames[ballNames.size()-1].c_str()), Vector3d::Zero())[2]+ballRadii[ballRadii.size()-1];
    q_init.bottomRows(model.dof_count-3) << M_PI/4., -M_PI/4., -M_PI/4., M_PI/4., M_PI/4., -M_PI/4., -M_PI/4., M_PI/4.;

    for(unsigned int i=0; i<q_init.rows();++i){
        x(i) =q_init[i];
        x(i+q_init.rows()) = 0.;        
    }

    // ------------------------ DEEP LEARNING
    uint steps = 128;
    uint epochs = n_epochs;
    uint mini_batch_size = 16;
    uint ppo_epochs = uint(steps/mini_batch_size);

    int64_t n_in = model.dof_count*3 + int(model.mBodies.size())*6; // q, qd, qdd, fext
    int64_t n_out = model.dof_count - 3; // control tau
    double std = 1e-2;

    // ------------------------ INCY WINCY
    IncyWincy incy_wincy(fileName, 
                         outLoc,
                         &model,
                         q_init,
                         ballNames,
                         ballIds,
                         ballRadii,
                         steps,
                         epochs,
                         mini_batch_size,
                         ppo_epochs,
                         n_in,
                         n_out,
                         std);

    // ------------------------ INTEGRATOR
    double t = 0.;
    double tp = 0.;
    unsigned int npts = 10000;

    double dt = 1e-3;

    std::vector<double> rowData(model.dof_count+1);
    std::vector<std::vector< double > > matrixData;

    rowData[0] = 0;
    for(uint z=0; z < model.dof_count; z++){
        rowData[z+1] = x(z);
    }
    matrixData.push_back(rowData);

    // Store results after each episode.
    std::string emptyHeader("");

    Timer(START);
    for (uint e=0;e<epochs;e++)
    {
        printf("epoch %d/%d\n", e+1, epochs);

        for(uint i=0; i <= npts; i++)
        {

            t += dt;

            // Get current state of the system.
            incy_wincy.State();

            // Integrator performs actions on the system.
            integrate_const(make_dense_output< runge_kutta_dopri5< vector_type > >( 5.0e-2 , 5.0e-2 ),
                            incy_wincy,
                            x, tp, t, dt);

            // Get the rewards, the dones, the log_probs etc.
            bool reset = incy_wincy.Update();

            if (incy_wincy.Counter() % steps == 0) 
            {
                incy_wincy.Process();
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
                incy_wincy.Reset();

                for(unsigned int j=0; j<q_init.rows();++j){
                    x(j) =q_init[j];
                    x(j+q_init.rows()) = 0.;        
                }

                t += dt;

                rowData[0] = t;
                for(uint z=0; z < model.dof_count; z++){
                    rowData[z+1] = x(z);
                }
                matrixData.push_back(rowData);
            }
        }

        t = 0.;

        std::string fileNameOut(outLoc + "animation_epoch_" + std::to_string(e+1) + ".csv");
        printMatrixToFile(matrixData,emptyHeader,fileNameOut);

        matrixData.clear();

        // Reset after each epoch.
        incy_wincy.Reset();

        for(unsigned int j=0; j<q_init.rows();++j){
            x(j) =q_init[j];
            x(j+q_init.rows()) = 0.;        
        }

        rowData[0] = t;
        for(uint z=0; z < model.dof_count; z++){
            rowData[z+1] = x(z);
        }
        matrixData.push_back(rowData);
    }
    auto time = Timer(STOP);
    printf("Elapsed time: %.2f s\n", time/1000.);

    return 0;
}