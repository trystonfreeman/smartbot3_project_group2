import time
import math
from smartbot_irl.utils import SmartLogger
from smartbot_irl import Command, SensorData, SmartBot
from Ant_Controller import ant_controller
from Goal_Managment import Goal, GoalManager


def main():
    # Init logger (optional, but nice to have)
    logger = SmartLogger('ant_controller')

    # Init robot
    bot = SmartBot(mode='real', drawing=True, smartbot_num=2)
    bot.init(host='192.168.33.2', port=9090, yaml_path='default_conf.yml')

    goal_manager = GoalManager(home_x=0.0, home_y=0.0)
    home_set = False

    print('Press Enter to go')
    input()
    time.sleep(0.05)
    bot.spin()
    print(Goal)

    # Control loop rate
    rate_hz = 20.0
    dt = 1.0 / rate_hz

    try:
        while True:
            sensors: SensorData = bot.read()
            bot.spin()
            # ---- Pose from odometry ----
            odom = sensors.odom  # Odometry from type_maps

            robot_x = odom.x
            robot_y = odom.y
            robot_theta = odom.yaw  # use yaw as heading

            # 2. On first loop, set home to the robot's starting pose
            if not home_set:
                goal_manager.home_x = robot_x
                goal_manager.home_y = robot_y
                home_set = True
                logger.info(f'Home set at ({goal_manager.home_x:.2f}, {goal_manager.home_y:.2f})')

                # 3. If we see a hex marker, store its world location
                if sensors.seen_hexes and sensors.seen_hexes.poses:
                    marker = sensors.seen_hexes.poses[0]

                    # marker.x, marker.y are in ROBOT frame
                    rel_x = marker.x
                    rel_y = marker.y

                    # Convert to WORLD frame to check for duplicates
                    gx = robot_x + rel_x * math.cos(robot_theta) - rel_y * math.sin(robot_theta)
                    gy = robot_y + rel_x * math.sin(robot_theta) + rel_y * math.cos(robot_theta)

                    # Only add if we don't already have a goal near this spot
                    if not goal_manager.has_goal_near(x=gx, y=gy, tol=0.3):  # 30 cm radius
                        goal_manager.add_goal_world(x=gx, y=gy)

                        last_goal = goal_manager.goals[-1]
                        logger.info(
                            f'Stored goal #{len(goal_manager.goals)} '
                            f'at world ({last_goal.x:.2f}, {last_goal.y:.2f})'
                        )

            # 4. Run the low-level ant controller
            cmd: Command = ant_controller(sensors)
            bot.write(cmd)
            time.sleep(dt)

    except KeyboardInterrupt:
        stop_cmd = Command()
        bot.write(stop_cmd)
        time.sleep(0.05)
        bot.write(stop_cmd)
        bot.shutdown()
        logger.info('Shutting down ant controller loop.')


if __name__ == '__main__':
    main()
