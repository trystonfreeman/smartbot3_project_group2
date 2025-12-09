# demo_2dsim.py
import logging
import time
import math
from math import pi, atan2
from smartbot_irl.robot import SmartBotType
from smartbot_irl.utils import SmartLogger
from smartbot_irl import Command, SensorData, SmartBot
from smartbot_irl.robot.smartbot_base import SmartBotBase
from smartbot_irl.data import LaserScan


logger = SmartLogger(level=logging.INFO)


def goDownToBlock(sensors: SensorData):
    # moves arm down to block
    cmd = Command()
    cmd.manipulator_presets = "DOWN"
    return cmd


def closeGripper(sensors: SensorData):
    # close Gripper to pickup block
    cmd = Command()
    cmd.gripper_closed = True
    cmd.manipulator_presets = "DOWN"
    return cmd


def holdBlock(sensors: SensorData):
    # move arm to hold position, after picking up block
    cmd = Command()
    cmd.manipulator_presets = "HOLD"
    cmd.gripper_closed = True
    return cmd


def lowerArmWhenHolding(sensors: SensorData):
    # lower arm while holding block
    cmd = Command()
    cmd.manipulator_presets = "DOWN"
    cmd.gripper_closed = True
    return cmd


def openGripper(sensors: SensorData):
    # open gripper to drop block
    cmd = Command()
    cmd.gripper_closed = False
    cmd.manipulator_presets = "DOWN"
    return cmd


def stowArm(sensors: SensorData):
    # stow the arm once block is dropped at home
    cmd = Command()
    cmd.manipulator_presets = "STOW"
    cmd.gripper_closed = False
    return cmd


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

    try:
        while True:
            t_start = time.perf_counter()
            sensors = bot.read()

            # Grab sequence
            cmd = goDownToBlock(sensors)
            bot.write(cmd)
            time.sleep(5)

            cmd = closeGripper(sensors)
            bot.write(cmd)
            time.sleep(2)

            cmd = holdBlock(sensors)
            bot.write(cmd)
            time.sleep(5)
            print("blockgrab")

            # Drop sequence
            cmd = lowerArmWhenHolding(sensors)
            bot.write(cmd)
            time.sleep(5)

            cmd = openGripper(sensors)
            bot.write(cmd)
            time.sleep(2)

            cmd = stowArm(sensors)
            bot.write(cmd)
            time.sleep(5)
            print("blockdrop")

            dt = time.perf_counter() - t_start
            if dt > 0.05:
                logger.warn(f"Loop took {dt:.2f}s!", rate=1)

            bot.spin()

    except KeyboardInterrupt:
        print("Shutting down...")
        bot.shutdown()
