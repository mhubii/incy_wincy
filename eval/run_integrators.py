import subprocess
import numpy as np

vel_y =  np.arange(1, 3)
vel_z = -np.arange(8, 10)
omg_y =  np.arange(8, 10)

# run processes
subprocess.run(["ls", "-l"])

for i in range(vel_y.size):
    subprocess.run(["../build/bouncingBallBenchmark", 
        "-m", "../models/ballPlaneContact.lua",
        "-v", str(int(vel_y[i])), str(int(vel_z[i])), str(int(omg_y[i])),
        "-o", "out/"])
    subprocess.run(["../src/numerical_jacobian/build/numericalJacobian",
        "-m", "../models/ballPlaneContact.lua",
        "-v", str(int(vel_y[i])), str(int(vel_z[i])), str(int(omg_y[i])),
        "-o", "out/"])
