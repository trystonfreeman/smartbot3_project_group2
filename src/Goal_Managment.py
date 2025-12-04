from dataclasses import dataclass, field
from typing import List, Optional, Tuple
import math

@dataclass
class Goal:
    x: float
    y: float
    visited: bool = False

@dataclass
class GoalManager:
    home_x: float
    home_y: float
    goals: List[Goal] = field(default_factory=list)
    current_goal_index: Optional[int] = None
    

    def add_goal_world(self, x: float, y: float) -> None:
        """Add a new goal in world coordinates."""
        self.goals.append(Goal(x, y))
    
    def add_goal_from_relative(
        self,
        robot_x: float,
        robot_y: float,
        robot_theta: float,
        rel_x: float,
        rel_y: float,
    ) -> None:
        # Convert a goal from robot frame (rel_x, rel_y) to world frame and store it
        # R(theta) * [rel_x, rel_y] + [robot_x, robot_y]
        gx = robot_x + rel_x * math.cos(robot_theta) - rel_y * math.sin(robot_theta)
        gy = robot_y + rel_x * math.sin(robot_theta) + rel_y * math.cos(robot_theta)
        self.add_goal_world(gx, gy)

    def _distance_sq(self, x1: float, y1: float, x2: float, y2: float) -> float:
        return (x1 - x2) ** 2 + (y1 - y2) ** 2

    def select_closest_unvisited(self, robot_x: float, robot_y: float) -> Optional[Goal]:
        best_idx = None
        best_dist = float("inf")

        for i, g in enumerate(self.goals):
            if g.visited:
                continue
            d2 = self._distance_sq(robot_x, robot_y, g.x, g.y)
            if d2 < best_dist:
                best_dist = d2
                best_idx = i

        self.current_goal_index = best_idx
        if best_idx is None:
            return None
        return self.goals[best_idx]

    def mark_current_goal_visited(self) -> None:
        if self.current_goal_index is not None:
            self.goals[self.current_goal_index].visited = True
            self.current_goal_index = None


    def current_goal(self) -> Optional[Goal]:
        if self.current_goal_index is None:
            return None
        return self.goals[self.current_goal_index]

def world_to_robot(
    robot_x: float,
    robot_y: float,
    robot_theta: float,
    goal_x: float,
    goal_y: float,
) -> Tuple[float, float]:
    dx = goal_x - robot_x
    dy = goal_y - robot_y

    # Rotate by -theta (world -> robot)
    rel_x =  dx * math.cos(robot_theta) + dy * math.sin(robot_theta)
    rel_y = -dx * math.sin(robot_theta) + dy * math.cos(robot_theta)
    return rel_x, rel_y
