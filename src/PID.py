class PID:
    def __init__(self, KP, KI, KD, r_des=0, v_des=0, i_des=0):
        self.KP = KP
        self.KI = KI
        self.KD = KD
        self.r_des = r_des
        self.v_des = v_des
        self.i_des = i_des
        self.i = 0

    def step(self, r, v, dt):
        self.i += (r - self.r_des) * dt  # Update Integral term

        u = (
            self.KP * (r - self.r_des)
            + self.KI * (self.i - self.i_des)
            + self.KD * (v - self.v_des)
        )
        return u

    def reset(self):
        self.i = 0
