import logging
import time
import math
from math import atan2
from math import pi
from smartbot_irl.robot import SmartBotType
from smartbot_irl.utils import SmartLogger
from smartbot_irl import Command, SensorData, SmartBot
from smartbot_irl.data import LaserScan

from PID import PID
# from teleop import get_key_command

logger = SmartLogger(level=logging.INFO)  # Print statements, but better!

KP_yaw = 10
KI_yaw = 20
KD_yaw = 0.3
des_yaw = 0
des_yaw_rate = 0
yaw_PID = PID(KP_yaw, KI_yaw, KD_yaw, des_yaw, des_yaw_rate)

KP_pos = 2
KI_pos = 1
KD_pos = 0
des_pos = 0
des_vel = 0
pos_PID = PID(KP_pos, KI_pos, KD_pos, des_pos, des_vel)


def get_range_forward(scan: LaserScan) -> float:
    """For coordinate conventions see REP 103 and REP 105:
    https://www.ros.org/reps/rep-0105.html
    https://www.ros.org/reps/rep-0103.html
    Find range directly forward. Scan starts at -2piRAD -> ranges[0].
    forward_range = (0RAD - 2piRAD) / angle_per_increment)
    """
    forward_range = scan.ranges[int((0 * pi - scan.angle_min) / scan.angle_increment)]
    logger.debug(f"{forward_range=}", rate=1)
    return forward_range


def step(bot: SmartBotType, dt):
    cmd = Command()
    """This is the main control loop for the robot. Code here should run in <50ms."""
    sensors = bot.read()

    # if sensors.scan is not None:
    # Look at the +X lidar range.
    # range_forward = get_range_forward(sensors.scan)
    # logger.info(f"{range_forward=}", rate=1)

    # Look at *every* valid attribute (i.e. is actually set) in the SensorData object.
    # for name, data in vars(sensors).items():
    # if data is not None:
    # logger.info(f"{name}: {data}\n", rate=5)
    goal_ang = 0.0
    dist_to_goal = 0.0
    if sensors.seen_hexes and sensors.seen_hexes.poses:
        marker = sensors.seen_hexes.poses[0]
        goal_ang = atan2(marker.x, marker.y) - pi / 2
        # print(goal_ang)
        dist_to_goal = math.hypot(marker.x, marker.y)

    v = math.sqrt(sensors.odom.vx**2 + sensors.odom.vy**2)

    yaw_control = yaw_PID.step(goal_ang, sensors.odom.wz, dt)
    pos_control = pos_PID.step(dist_to_goal, v, dt)
    cmd.angular_vel = -yaw_control

    cmd.angular_vel = 0

    if abs(goal_ang) < 0.05:
        cmd.linear_vel = pos_control
    cmd.linear_vel = 0
    bot.write(cmd)

    time.sleep(0.020)  # REMOVE. Simulate a non-trivial loop by sleeping 20ms.


if __name__ == "__main__":
    """ Create an instance of the SmartBot wrapper class for your specific
    smartbot. Then we run our control loop :meth:`step` forever until stopped
    (e.g. <Ctrl-c>)."""

    logger.info("Connecting to smartbot...")
    # bot = SmartBot(mode="real", drawing=True, smartbot_num=0)
    # bot.init(host="localhost", port=9090, yaml_path="default_conf.yml")

    # bot = SmartBot(mode="real", drawing=True, smartbot_num=2)
    # bot.init(host="192.168.33.2", port=9090, yaml_path="default_conf.yml")

    bot = SmartBot(mode="sim", drawing=True, smartbot_num=3)
    bot.init(drawing=True, smartbot_num=3)
    dt = 0
    try:
        while True:
            t_start = time.perf_counter()
            step(bot, dt)  # Run your code.

            dt = time.perf_counter() - t_start  # Check if your code is running fast enough.
            if dt > 0.05:
                logger.warn(f"Loop took {dt:2f}s!", rate=1)

            bot.spin()  # Get new sensor data.
    except KeyboardInterrupt:
        print("Shutting down...")
        bot.shutdown()
