from control import *
from control.matlab import *  # MATLAB-like functions
import numpy as np
from LQR.control_submodule.car_model import CarModel


class LinearKalmanFilter:
    def __init__(self):
        self.sample_size = []  # length of input vector
        self.Pp = []  # error covariance
        self.P_mat = []  # storage for Pp over time
        self.K_mat = []  # K gain matrix
        self.xhat = []  # optimal state estimate
        self.yhat = []  # optimal output estimate
        self.xhat_mat = []  # storage for xhat over time
        self.yhat_mat = []  # storage for yhat over time
        self.lqr_car = CarModel()
        self.sysd = 0

    def build_system(self, Vx):
        self.sysd = self.lqr_car.build_error_model(Vx)

    def lkf(self, sys, x0, u, y, P0, Qo, Ro):
        self.sysd = sys
        [A, B, C, D] = ssdata(self.sysd)
        A = np.array(A)
        B = np.array(B)
        C = np.array(C)
        D = np.array(D)
        x0 = np.array(x0)
        u = np.array(u)
        y = np.array(y)
        P0 = np.array(P0)
        Qo = np.array(Qo)
        Ro = np.array(Ro)

        a_num_rows, a_num_cols = A.shape
        d_num_rows, d_num_cols = D.shape
        self.Pp = P0
        self.xhat = x0
        self.sample_size = u.size
        self.xhat_mat = np.zeros([a_num_rows, self.sample_size], dtype=float)
        self.yhat_mat = np.zeros([d_num_rows, self.sample_size], dtype=float)
        self.K_mat = np.zeros([a_num_rows, self.sample_size], dtype=float)

        A_t = A.transpose()
        C_t = C.transpose()
        num_states = A.shape[0]

        for k in range(0, self.sample_size):
            try:
                self.xhat_mat[:, k] = self.xhat.transpose()  # store the estimates

                # Time update
                self.xhat = np.add(np.dot(A, self.xhat).reshape(num_states, 1),
                                   np.dot(B, u[k]))  # predicted state estimate
                self.Pp = np.add(np.dot(np.dot(A, self.Pp), A_t), Qo)  # covariance

                # # Measurement update
                K = np.dot(np.dot(np.dot(A, self.Pp), C_t),
                           np.linalg.inv(np.add(np.dot(np.dot(C, self.Pp), C_t), Ro)))  # Kalman predictor gain
                self.xhat = np.add(np.dot((np.subtract(A, np.dot(K, C))), self.xhat).reshape(num_states, 1),
                                   np.add(np.dot(B, u[k]), np.dot(K, y[:, k]).reshape(num_states, 1)))
                self.Pp = np.subtract(self.Pp, np.dot(
                    np.dot(np.dot(np.dot(self.Pp, C_t), np.linalg.inv(np.add(np.dot(np.dot(C, self.Pp), C_t), Ro))), C),
                    self.Pp))

                # filtered output prediction
                self.yhat = np.dot(C, self.xhat)
                
            except:
                if self.debug:
                    print("Error occured during calculations")
                    print(f"A: {A}")
                    print(f"self.xhat: {self.xhat}")
                    print(f"np.dot(B, u[k]): {np.dot(B, u[k])}")
                    print(f"K: {K}")
                    print(f"y[:, k]: {y[:, k]}")
                    print(f"np.dot(K, y[:, k]): {np.dot(K, y[:, k])}")
                    print(f"np.dot(K, y[:, k]).reshape(num_states, 1)): {np.dot(K, y[:, k]).reshape(num_states, 1)}")
                    print(f"np.add(np.dot(B, u[k]), np.dot(K, y[:, k]).reshape(num_states, 1))): {np.add(np.dot(B, u[k]), np.dot(K, y[:, k]).reshape(num_states, 1))}")
                    print(f"self.xhat_mat: {self.xhat_mat}")
                    print(f"self.xhat: {self.xhat}")
                    print(f"A: {A}")
                    print(f"self.xhat: {self.xhat}")
                    print(f"np.dot(A, self.xhat) 2: {np.dot(A, self.xhat)}")
                    print(f"B: {B}")
                    print(f"u[k]: {u[k]}")
                    print(f"np.dot(B, u[k]): {np.dot(B, u[k])}")
                    print(f"K_mat: {self.K_mat}")
                    print(f"K: {K}")
                    print(f"y: {self.yhat}")
                    print(f"y_mat: {self.yhat_mat}")

        return self.xhat, self.Pp


def main():
    my_kalman = LinearKalmanFilter()
    my_kalman.build_system(5)
    x0 = [1, 1, 1, 1]
    u = [1, 1, 1, 1]
    y = np.array([[1, 1, 1, 1],
                  [1, 1, 1, 1],
                  [1, 1, 1, 1],
                  [1, 1, 1, 1]])
    P0 = np.diag([1, 1, 1, 1])
    Qo = np.diag([1, 1, 1, 1])
    Ro = [0.1]
    my_kalman.debug = True
    my_kalman.lkf(my_kalman.sysd, x0, u, y, P0, Qo, Ro)


if __name__ == '__main__':
    main()
