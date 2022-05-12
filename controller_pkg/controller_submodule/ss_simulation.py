from control.matlab import *  # MATLAB-like functions
import numpy as np
from LQR.control_submodule.car_model import CarModel
from matplotlib import pyplot as plt


class StateSpaceSimulation:
    def __init__(self):
        self.sample_size = 0
        self.num_states = 0
        self.num_outputs = 0
        self.x = np.zeros([1, 1], dtype=float)
        self.y = np.zeros([1, 1], dtype=float)
        self.lqr_car = CarModel()
        self.sysd = 0

    def build_system(self, Vx):
        self.sysd = self.lqr_car.build_error_model(Vx)

    def ss_simulation(self, A, B, C, D, x0, u):
        x0 = np.array(x0)
        u = np.array(u)
        self.sample_size = u.size
        self.num_states = A.shape[0]
        self.num_outputs = D.shape[0]
        self.x = np.zeros([self.num_states, self.sample_size], dtype=float)
        self.y = np.zeros([self.num_outputs, self.sample_size], dtype=float)
        self.x[:, 0] = x0.transpose()
        for k in range(0, self.sample_size - 1):
            self.y[:, k] = np.add(np.dot(C, self.x[:, k]).reshape(self.num_states, 1), np.dot(D, u[k])).transpose()
            self.x[:, k + 1] = np.add(np.dot(A, self.x[:, k]).reshape(self.num_states, 1), np.dot(B, u[k])).transpose()
        return self.x, self.y

    def plot_ss_results(self, t):
        legend_state_labels = []
        legend_output_labels = []
        plt.subplot(1, 2, 1)
        for k in range(0, self.num_states):
            plt.plot(t, self.x[k, :])
            legend_state_labels.append("x" + str(k+1))
        plt.xlabel("Samples (k)")
        plt.ylabel(f"State values")
        plt.legend(legend_state_labels)
        plt.subplot(1, 2, 2)
        for k in range(0, self.num_outputs):
            plt.plot(t, self.y[k, :])
            legend_output_labels.append("y" + str(k+1))
        plt.xlabel("Samples (k)")
        plt.ylabel(f"Output values")
        plt.legend(legend_output_labels)
        plt.show()


def plot_ss_sol_example():
    v = 5  # m/s
    my_sim = StateSpaceSimulation()
    my_sim.build_system(v)
    [A, B, C, D] = ssdata(my_sim.sysd)
    x0 = [1, 0, 0.3, 0]
    delta1 = .5 * np.ones(10, dtype=float)
    delta2 = np.zeros(10, dtype=float)
    delta = np.concatenate((delta1, delta2), axis=None)
    tsim = linspace(0, delta.shape[0], delta.shape[0])
    xsim, ysim = my_sim.ss_simulation(A, B, C, D, x0, delta)
    my_sim.plot_ss_results(tsim)


if __name__ == '__main__':
    plot_ss_sol_example()
