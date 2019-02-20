import numpy as np
import matplotlib.pyplot as plt

# generate plots
def plot(t_ii, t_ei, a_ii, a_ei):
    # determine metrics
    #mse = np.linalg.norm(a_ii - a_ei)

    x_ii = a_ii[:,0]
    z_ii = a_ii[:,2]

    x_ei = a_ei[:,0]
    z_ei = a_ei[:,2]

    plt.plot(x_ii, z_ii, label="explicit, time {}s".format(t_ii[1]))
    plt.plot(x_ei, z_ei, "--", label="implicit, time {}s".format(t_ei[1]))
    plt.legend()
    plt.show()



if __name__ == "__main__":
    # load data, time stores v_init, time
    time_ii = np.genfromtxt("out/time_ii.csv", delimiter=",")
    time_ei = np.genfromtxt("out/time_ei.csv", delimiter=",")

    animation_ei = []
    animation_ii = []

    for v in time_ii:
        animation_ii.append(np.genfromtxt("out/animation_ii_vinit_{}.csv".format(str(int(v[0]))), delimiter=","))
        animation_ei.append(np.genfromtxt("out/animation_ei_vinit_{}.csv".format(str(int(v[0]))), delimiter=","))

    # plot results
    for i in range(time_ii.shape[0]):
        plot(time_ii[i], time_ei[i], animation_ii[i], animation_ei[i])

