# Kalman filter class

# import gives us matrix operations
import numpy as np


class KF:
    def __init__(self, A, B, C, Q, R, X0, P0):
        self.A = np.array(A)
        self.B = np.array(B)
        self.C = np.array(C)
        self.Q = np.array(Q)
        self.R = np.array(R)
        self.x = np.array(X0)
        self.P = np.array(P0)

    def predict(self, u=None):
        # X^= A*X+B*u
        # matrix multiply A*X
        self.x = self.A @ self.x
        # matrix multiply B*u
        temp = self.B @ u
        # adds two parts together
        self.x = self.x + temp

        # P^=A*P*A^T+Q
        self.P = self.A @ self.P @ self.A.T + self.Q

    def update(self, y, u=None):
        y = np.array(y)

        # K=p*c(c*p*c^T+R)^-1
        temp = self.C @ self.P @ self.C.T + self.R
        temp = np.linalg.inv(temp)  # numpy function to do temp^-1
        K = self.P @ self.C.T @ temp

        # X^=X^+K(y-C*X^)
        temp = y - self.C @ self.x
        self.x = self.x + K @ temp

        # P=(I-K*C)*P
        I = np.eye(len(self.P))  # gives identity matrix the same size as P
        self.P = (I - K @ self.C) @ self.P

    def step(self, y, u):
        self.predict(u)
        self.update(y)
        return self.x
