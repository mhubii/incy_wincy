import numpy as np
import matplotlib.pyplot as plt

# generate plots
def plot(t_ii, t_ei, a_ii, a_ei):
    # determine metrics
    mse = np.linalg.norm(a_ii - a_ei)
    mse /= a_ii.shape[0]

    y_ii = a_ii[:,1]
    z_ii = a_ii[:,2]

    y_ei = a_ei[:,1]
    z_ei = a_ei[:,2]

    plt.plot(y_ii, z_ii, label="imp, t {}ms, v ({},{},{})m/s, mse {:.2e}m".format(t_ii[3], t_ii[0], t_ii[1], t_ii[2], mse))
    plt.plot(y_ei, z_ei, "--", label="exp, t {}ms, v ({},{},{})m/s".format(t_ei[3], t_ei[0], t_ei[1], t_ei[2]))
    #plt.show()



if __name__ == "__main__":
    # load data, time stores v_init, time
    time_ii = np.genfromtxt("out/time_ii.csv", delimiter=",")
    time_ei = np.genfromtxt("out/time_ei.csv", delimiter=",")

    animation_ei = []
    animation_ii = []

    for v in time_ii:
        animation_ii.append(np.genfromtxt("out/animation_ii_vinit_{}_{}_{}.csv".format(int(v[0]), int(v[1]), int(v[2])), delimiter=","))
        animation_ei.append(np.genfromtxt("out/animation_ei_vinit_{}_{}_{}.csv".format(int(v[0]), int(v[1]), int(v[2])), delimiter=","))

    # plot results
    for i in range(time_ii.shape[0]):
        plot(time_ii[i], time_ei[i], animation_ii[i], animation_ei[i])
        plt.title("Bouncing Ball Benchmark with Explicit and Implicit Integrators")
        plt.xlabel("y/m")
        plt.ylabel("z/m")
        plt.legend()
        plt.savefig("compare.pdf")
