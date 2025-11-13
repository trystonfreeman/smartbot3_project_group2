# Extended Kalman filter class

# import gives us matrix operations
import numpy as np


class EKF:
    def __init__(self, f, h, Fx, Hx, Q, R, X0, P0):
        self.f = f
        self.h = h
        self.Fx = Fx
        self.Hx = Hx
        self.Q = np.array(Q)
        self.R = np.array(R)
        self.x = np.array(X0)
        self.P = np.array(P0)

    def predict(self, u=None):
        # x^=f(x^,u)
        self.x = self.f(self.x, u)

        # P=Fk*P*F^T+Q
        Fk = self.Fx(self.x, u)
        self.P = Fk @ self.P @ Fk.T + self.Q

    def update(self, y, u=None):
        # rk=yk-h(x^)
        ypredict = self.h(self.x)
        rk = y - ypredict

        # Sk=Hk*P*Hk^T+R
        Hk = self.Hx(self.x)
        Sk = Hk @ self.p @ Hk.T + self.R

        # Kk=P*Hk^T*Sk^-1
        kk = self.P @ Hk.T @ np.linalg.inv(Sk)

        # x^=x^+K*rk
        self.x = self.x + kk @ rk

        # P=(I-K*Hk)*P
        IdenMatrix = np.eye(len(self.P))  # gives identity matrix the same size as P
        self.P = (IdenMatrix - kk @ Hk) @ self.P

    def step(self, y, u):
        self.predict(u)
        self.update(y)
        return self.x, self.P
