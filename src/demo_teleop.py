# demo_2dsim.py
import logging
import time
import math
from math import pi
from smartbot_irl.robot import SmartBotType
from smartbot_irl.utils import SmartLogger
from smartbot_irl import Command, SensorData, SmartBot
from smartbot_irl.data import LaserScan
from teleop import get_key_command

logger = SmartLogger(level=logging.INFO)  # Print statements, but better!


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


def step(bot: SmartBotType):
    """This is the main control loop for the robot. Code here should run in <50ms."""
    sensors = bot.read()

    # Print out sensor data.
    logger.info(sensors.odom, rate=3)
    logger.info(sensors.odom.x, rate=3)
    logger.info(sensors.odom.y, rate=3)
    logger.info(sensors.odom.yaw, rate=3)

    # logger.info(sensors.imu, rate=3)
    # logger.info(sensors.aruco_poses, rate=3)
    # logger.info(sensors.manipulator_curr_preset, rate=3)
    # logger.info(sensors.gripper_curr_state, rate=3)
    # logger.info(sensors.joints, rate=3)


    # Look at the +X lidar range.
    if sensors.scan is not None:
        range_forward = get_range_forward(sensors.scan)
        logger.info(f"{range_forward=}", rate=5)

    # Print *every* populated sensor data attribute.
    # for name, data in vars(sensors).items():
    #     if data is not None:
    #         logger.info(f"{name}: {data}\n", rate=5)

    # Drive the robot.
    cmd = get_key_command(sensors)
    bot.write(cmd)

    time.sleep(0.020)  # REMOVE. Simulate a non-trivial loop by sleeping 20ms.


if __name__ == "__main__":
    """ Create an instance of the SmartBot wrapper class for your specific
    smartbot. Then we run our control loop :meth:`step` forever until stopped
    (e.g. <Ctrl-c>)."""

    logger.info("Connecting to smartbot...")
    # bot = SmartBot(mode="real", drawing=True, smartbot_num=0)
    # bot.init(host="localhost", port=9090, yaml_path="default_conf.yml")

    # bot = SmartBot(mode="real", drawing=True, smartbot_num=4)
    # bot.init(host="192.168.33.4", port=9090, yaml_path="default_conf.yml")

    bot = SmartBot(mode="sim", drawing=True, smartbot_num=3)
    bot.init(drawing=True, smartbot_num=3)

    try:
        while True:
            t_start = time.perf_counter()
            step(bot)  # Run your code.

            dt = time.perf_counter() - t_start  # Check if your code is running fast enough.
            if dt > 0.05:
                logger.warn(f"Loop took {dt:2f}s!", rate=1)

            bot.spin()  # Get new sensor data.
    except KeyboardInterrupt:
        print("Shutting down...")
        bot.shutdown()
