from typing import Literal

import time
import math
from enum import Enum, auto
from smartbot_irl.utils import SmartLogger
from smartbot_irl import Command, SensorData, SmartBot
from Ant_Controller import ant_controller
from Goal_Managment import Goal, GoalManager
from Goal_Managment import world_to_robot


class Mode(Enum):
    SEARCHING = auto()
    GOING_TO_GOAL = auto()
    WAITING_AT_GOAL = auto()
    RETURNING_HOME = auto()


def search(
    sensors, robot_x, robot_y, robot_theta, goal_manager
) -> (
    tuple[float, float | None, bool, Literal[Mode.GOING_TO_GOAL]]
    | tuple[float, float | None, bool, Literal[Mode.SEARCHING]]
):
    # Use the live marker as the goal if visible
    from Ant_Controller import COMPUTE_GOAL_FROM_MARKERS

    goal_ang, dist, visible = COMPUTE_GOAL_FROM_MARKERS(sensors)

    if goal_manager.has_unvisited_goals():
        return goal_ang, dist, visible, Mode.GOING_TO_GOAL

    return goal_ang, dist, visible, Mode.SEARCHING


def go_to_goal(
    robot_x, robot_y, robot_theta, goal_manager
) -> (
    tuple[None, None, Literal[False], Literal[Mode.RETURNING_HOME]]
    | tuple[None, None, Literal[False], Literal[Mode.WAITING_AT_GOAL]]
    | tuple[float, float, Literal[True], Literal[Mode.GOING_TO_GOAL]]
):
    goal = goal_manager.current_goal()
    if goal is None:
        goal = goal_manager.select_closest_unvisited(robot_x, robot_y)

        if goal is None:
            return None, None, False, Mode.RETURNING_HOME

    rel_x, rel_y = world_to_robot(robot_x, robot_y, robot_theta, goal.x, goal.y)
    goal_ang = math.atan2(rel_y, rel_x)
    dist = math.hypot(rel_x, rel_y)

    # Close enough?
    if dist < 0.25:
        goal_manager.mark_current_goal_visited()
        return None, None, False, Mode.WAITING_AT_GOAL

    return goal_ang, dist, True, Mode.GOING_TO_GOAL
    # DEBUG


"""print(
        f'[GO_TO_GOAL] current_goal=({goal.x:.2f}, {goal.y:.2f}), '
        f'robot=({robot_x:.2f}, {robot_y:.2f}, {robot_theta:.2f}), '
        f'rel=({rel_x:.2f}, {rel_y:.2f}), dist={dist:.3f}'
    ) """


def waiting_at_goal(
    wait_start_time, goal_manager, robot_x, robot_y
) -> (
    tuple[None, None, Literal[False], Literal[Mode.RETURNING_HOME]]
    | tuple[None, None, Literal[False], Literal[Mode.GOING_TO_GOAL]]
    | tuple[float, None, Literal[False], Literal[Mode.WAITING_AT_GOAL]]
):
    now = time.time()

    if now - wait_start_time > 2.0:
        # Move to next goal
        next_goal = goal_manager.select_closest_unvisited(robot_x, robot_y)
        if next_goal is None:
            return None, None, False, Mode.RETURNING_HOME
        return None, None, False, Mode.GOING_TO_GOAL

    # While waiting, stop the robot
    return 0.0, None, False, Mode.WAITING_AT_GOAL


def return_home(
    robot_x, robot_y, robot_theta, goal_manager
) -> (
    tuple[None, None, Literal[False], Literal[Mode.SEARCHING]]
    | tuple[float, float, Literal[True], Literal[Mode.RETURNING_HOME]]
):
    rel_x, rel_y = world_to_robot(
        robot_x, robot_y, robot_theta, goal_manager.home_x, goal_manager.home_y
    )

    goal_ang = math.atan2(rel_y, rel_x)
    dist = math.hypot(rel_x, rel_y)

    if dist < 0.15:
        # Arrived home — restart search
        return None, None, False, Mode.SEARCHING

    return goal_ang, dist, True, Mode.RETURNING_HOME


def main():
    # Init logger (optional, but nice to have)
    logger = SmartLogger(name='ant_controller')

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

    mode = Mode.SEARCHING
    wait_start = None
    logger.info(f'Initial MODE = {mode.name}')

    try:
        while True:
            sensors: SensorData = bot.read()
            bot.spin()

            # ---- Pose from odometry ----
            odom = sensors.odom
            robot_x = odom.x
            robot_y = odom.y
            robot_theta = odom.yaw  # use yaw as heading
            # Add throttle

            # 1) On first loop, set home to the robot's starting pose
            if not home_set:
                goal_manager.home_x = robot_x
                goal_manager.home_y = robot_y
                home_set = True
                logger.info(f'Home set at ({goal_manager.home_x:.2f}, {goal_manager.home_y:.2f})')

            # 2) Always: if we see a hex marker, store its world location
            if sensors.seen_hexes and sensors.seen_hexes.poses:
                marker_pose = sensors.seen_hexes.poses[0]
                marker_id = sensors.seen_hexes.marker_ids[0]

                rel_x = marker_pose.x
                rel_y = marker_pose.y

                gx = robot_x + rel_x * math.cos(robot_theta) - rel_y * math.sin(robot_theta)
                gy = robot_y + rel_x * math.sin(robot_theta) + rel_y * math.cos(robot_theta)

                # Try adding goal — add_goal_world returns False if duplicate
                if goal_manager.add_goal_world(gx, gy, marker_id=marker_id):
                    logger.info(
                        f'Stored goal #{len(goal_manager.goals)} at world '
                        f'({gx:.2f}, {gy:.2f}), marker_id={marker_id}'
                    )

            # --- 3) Decide what to do based on mode: compute goal for ant_controller ---
            prev_mode = mode
            goal_ang = 0.0
            dist_to_goal = None
            goal_visible = False

            if mode == Mode.SEARCHING:
                goal_ang, dist_to_goal, goal_visible, mode = search(
                    sensors, robot_x, robot_y, robot_theta, goal_manager
                )

            elif mode == Mode.GOING_TO_GOAL:
                goal_ang, dist_to_goal, goal_visible, mode = go_to_goal(
                    robot_x, robot_y, robot_theta, goal_manager
                )

            elif mode == Mode.WAITING_AT_GOAL:
                # Ensure wait_start is initialized when we first enter WAITING
                if wait_start is None:
                    wait_start = time.time()

                goal_ang, dist_to_goal, goal_visible, new_mode = waiting_at_goal(
                    wait_start, goal_manager, robot_x, robot_y
                )
                # If we leave WAITING, reset wait_start
                if new_mode != Mode.WAITING_AT_GOAL:
                    wait_start = None
                mode = new_mode

            elif mode == Mode.RETURNING_HOME:
                goal_ang, dist_to_goal, goal_visible, mode = return_home(
                    robot_x, robot_y, robot_theta, goal_manager
                )

            if mode != prev_mode:
                logger.info(f'MODE TRANSITION: {prev_mode.name} -> {mode.name}')

            # 4) Run the low-level ant controller with the chosen goal

            if mode == Mode.WAITING_AT_GOAL:
                stop_cmd = Command()
                bot.write(stop_cmd)
            else:
                cmd: Command = ant_controller(
                    sensors,
                    goal_ang=goal_ang,
                    dist_to_goal=dist_to_goal,
                    goal_visible=goal_visible,
                )
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
