import time
import math
import logging
import time
import math
from math import pi, atan2
from smartbot_irl.robot import SmartBotType
from smartbot_irl.utils import SmartLogger
from smartbot_irl import Command, SensorData, SmartBot
from smartbot_irl.robot.smartbot_base import SmartBotBase
from smartbot_irl.data import LaserScan
from Ant_Controller import ant_controller
from Goal_Managment import GoalManager 

def main():
    # Init logger (optional, but nice to have)
    logger = SmartLogger("ant_controller")

    # Init robot 
    bot = SmartBot(mode="real", drawing=True, smartbot_num=2)
    bot.init(host="192.168.33.2", port=9090, yaml_path="default_conf.yml")

    goal_manager = GoalManager(home_x=0.0, home_y=0.0)
    home_set = False

    # Control loop rate
    rate_hz = 20.0
    dt = 1.0 / rate_hz

    try:
        while True:
            # 1. Read sensors
            sensors: SensorData = bot.get_sensor_data()

            # 2. On first loop, set home to the robot's starting pose
            if not home_set and hasattr(sensors, "pose"):
                goal_manager.home_x = sensors.pose.x
                goal_manager.home_y = sensors.pose.y
                home_set = True
                logger.info(f"Home set at ({goal_manager.home_x:.2f}, {goal_manager.home_y:.2f})")

            # 3. If we see a hex marker, store its world location
            if hasattr(sensors, "pose") and sensors.seen_hexes and sensors.seen_hexes.poses:
                marker = sensors.seen_hexes.poses[0]
                robot_x = sensors.pose.x
                robot_y = sensors.pose.y
                robot_theta = sensors.pose.theta

                # marker.x, marker.y are in robot frame
                goal_manager.add_goal_from_relative(
                    robot_x, robot_y, robot_theta,
                    marker.x, marker.y
                )

                logger.info(
                    f"Stored goal #{len(goal_manager.goals)} "
                    f"at world ({goal_manager.goals[-1].x:.2f}, {goal_manager.goals[-1].y:.2f})"
                )

            # 4. Run the low-level ant controller (search + go-to-visible-goal)
            cmd: Command = ant_controller(sensors)

            # 5. Send command to robot
            bot.send_command(cmd)

            # 6. Sleep to maintain loop rate
            time.sleep(dt)

    except KeyboardInterrupt:
        stop_cmd = Command()
        bot.send_command(stop_cmd)
        time.sleep(0.05)
        bot.send_command(stop_cmd)
        bot.shutdown()
        logger.info("Shutting down ant controller loop.")


if __name__ == "__main__":
    main()
