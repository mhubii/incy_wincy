import subprocess
import numpy as np

vel = np.arange(0, 5)

# run processes
subprocess.run(["ls", "-l"])

for v in vel:
    subprocess.run(["../build/bouncingBallBenchmark", 
        "-m", "../models/ballPlaneContact.lua",
        "-v", str(v),
        "-o", "out/"])
    subprocess.run(["../src/numerical_jacobian/build/numericalJacobian",
        "-m", "../models/ballPlaneContact.lua",
        "-v", str(v),
        "-o", "out/"])
