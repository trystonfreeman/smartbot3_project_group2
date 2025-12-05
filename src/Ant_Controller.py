import math

from smartbot_irl import Command
from smartbot_irl.data import LaserScan


def COMPUTE_GOAL_FROM_MARKERS(sensors) -> tuple[float, float | None, bool]:
    """
    Returns (goal_ang, dist_to_goal, visible)
    """
    if sensors.seen_hexes and sensors.seen_hexes.poses:
        marker = sensors.seen_hexes.poses[0]  # First detected marker

        rel_x = marker.x
        rel_y = marker.y

        goal_ang = math.atan2(rel_y, rel_x)
        dist_to_goal = math.hypot(rel_x, rel_y)

        return goal_ang, dist_to_goal, True
    else:
        # No marker visible
        return (0.0, None, False)


def Obstacle_Avoidance(scan, avoid_thresh):
    repulse_direction = 0.0
    pressure = 0.0

    for i, r in enumerate(scan.ranges):
        if not r or math.isnan(r) or math.isinf(r):
            continue
        if r < avoid_thresh:
            # angle relative to robot heading
            ang = scan.angle_min + i * scan.angle_increment
            w = 1.0 / max(r, 0.05)  # closer obstacles weigh stronger
            repulse_direction -= math.sin(ang) * w  # steer away from obstacle side
            pressure += w

    if pressure > 0:
        repulse_direction = repulse_direction / pressure
    return (repulse_direction, pressure)


def ant_controller(sensors, k_goal=2.0, k_avoid=5.5, base_speed=10, avoid_thresh=1) -> Command:
    cmd = Command()
    scan: LaserScan = sensors.scan
    if scan is None or not scan.ranges:
        return cmd  # do nothing until we have LIDAR

    # Step 1 determine Goal position.
    goal_ang, dist_to_goal, visible = COMPUTE_GOAL_FROM_MARKERS(sensors)

    # Step 2 Determine obstacle avoidance.
    repulse, pressure = Obstacle_Avoidance(scan, avoid_thresh)

    # Step 3 Goal and Obstacle avoidance.
    # Angular velocity.
    ang_vel = 0.0
    if visible:
        ang_vel += k_goal * goal_ang * 0.3
    if pressure > 0:
        ang_vel += k_avoid * repulse * 0.3

    search_ang_speed = 0.6
    search_lin_factor = 0.3

    searching = False

    if not visible and pressure == 0.0:
        ang_vel = search_ang_speed
        searching = True

    if abs(ang_vel) < 0.05:
        ang_vel = 0.0
    ang_vel = max(-7.0, min(7.0, ang_vel))

    # Linear velocity.

    slowdown = 1.0
    decay_rate = 2.0
    if dist_to_goal is not None:
        slowdown = 1.0 - math.exp(-decay_rate * dist_to_goal)
        slowdown = max(0.05, slowdown)  # don’t go to zero unless at goal

    v_scale = (1.0 / (1.0 + 0.1 * pressure)) * slowdown

    if searching:
        lin_vel = base_speed * search_lin_factor * (1.0 / (1.0 + 0.1 * pressure))
    else:
        lin_vel = base_speed * v_scale

    # Stop if we’re basically on top of the goal
    if visible and dist_to_goal is not None and dist_to_goal < 0.25:
        lin_vel = 0.0

    turn_in_place_thresh = 0.3  # radians.
    if visible and dist_to_goal is not None and abs(goal_ang) > turn_in_place_thresh:
        lin_vel = 0.0

    cmd.linear_vel = lin_vel
    cmd.angular_vel = ang_vel
    return cmd
