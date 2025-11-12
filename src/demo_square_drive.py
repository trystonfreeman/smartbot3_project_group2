# demo_2dsim.py
from dataclasses import dataclass, field
import logging
from pyclbr import Class
from time import time
import math
from math import pi
from smartbot_irl.robot import SmartBotType
from smartbot_irl.utils import SmartLogger
from smartbot_irl import Command, SensorData, SmartBot
from smartbot_irl.data import LaserScan, Frame
from teleop import get_key_command
import pandas as pd

logger = SmartLogger(level=logging.INFO)  # Print statements, but better!


class Params:
    def __init__(self):
        self.side_length: float = 2.0
        self.speed: float = 1.0
        self.turn_speed: float = 0.8
        self.t0: float = 0.0


# class State(Frame):
#     """Student defined custom state."""

#     t: pd.Series
#     x: pd.Series
#     y: pd.Series
#     yaw: pd.Series
#     v: pd.Series
#     turning: bool = False
#     index: int = 0


@dataclass
class State:
    """Just a dataclass that holds a DataFrame and scalar values."""

    state_vec: pd.DataFrame = field(
        default_factory=lambda: pd.DataFrame(columns=["t", "x", "y", "yaw", "v"])
    )
    index: int = 0
    t_elapsed: float = 0.0
    t_prev: float = 0.0
    turning: bool = False


def step(bot: SmartBotType, params: Params, state: State, t: float):
    """Drive a box"""
    index = state.index
    # state_now = state.state_vec.iloc[state.index] # Grab current row.

    cmd = Command()
    t_elapsed = t - state.t_prev

    if not state.turning:
        # Drive forward.
        cmd.linear_vel = params.speed
        cmd.angular_vel = 0.0

        # Check how far we've gone.
        if t_elapsed * params.speed >= params.side_length:
            state.turning = True
            state.t_prev = t
    else:
        cmd.linear_vel = 0.0
        cmd.angular_vel = params.turn_speed

        # Check how far we've turned.
        if t_elapsed * params.turn_speed >= pi / 2:
            state.turning = False
            state.t_prev = t
    state.index += 1
    bot.write(cmd)


if __name__ == "__main__":
    """ Create an instance of the SmartBot wrapper class for your specific
    smartbot. Then we run our control loop :meth:`step` forever until stopped
    (e.g. <Ctrl-c>)."""

    logger.info("Connecting to smartbot...")
    # bot = SmartBot(mode="real", drawing=True, smartbot_num=0)
    # bot.init(host="localhost", port=9090, yaml_path="default_conf.yml")

    bot = SmartBot(mode="real", drawing=True, smartbot_num=2)
    bot.init(host="192.168.33.2", port=9090, yaml_path="default_conf.yml")

    # bot = SmartBot(mode="sim", drawing=True, smartbot_num=3)
    # bot.init(drawing=True, smartbot_num=3)

    state = State()
    params = Params()
    params.t0 = time()  # Starting time (sec)

    try:
        while True:
            t = time()  # Get current time (sec).

            # Run your code.
            step(bot, params, state, t)

            dt = time() - t  # Check if your code is running fast enough.
            if dt > 0.05:
                logger.warn(f"Loop took {dt:2f}s!", rate=1)

            bot.spin()  # Get new sensor data.
    except KeyboardInterrupt:
        print("Shutting down...")
        bot.shutdown()
