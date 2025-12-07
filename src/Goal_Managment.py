from dataclasses import dataclass, field  # noqa: F401
from typing import List, Optional, Tuple  # noqa: F401
import math


@dataclass
class Goal:
    x: float
    y: float
    visited: bool = False
    marker_id: int | None = None


class GoalManager:
    def __init__(self, home_x: float, home_y: float, min_goal_spacing: float = 0.3):
        self.home_x = home_x
        self.home_y = home_y
        self.min_goal_spacing = min_goal_spacing
        self.goals: list[Goal] = []
        self.current_goal_index: Optional[int] = None  # <-- the missing piece

    # ---------- utilities ----------

    def num_goals(self) -> int:
        return len(self.goals)

    def has_goal_near(self, x: float, y: float, tol: float) -> bool:
        """Check if there is already a goal within tol meters of (x, y)."""
        for g in self.goals:
            dx = g.x - x
            dy = g.y - y
            if dx * dx + dy * dy < tol * tol:
                return True
        return False

    def has_goal_for_marker(self, marker_id: int) -> bool:
        return any(g.marker_id == marker_id for g in self.goals)

    def add_goal_world(self, x: float, y: float, marker_id: int | None = None) -> bool:
        """
        Add a goal in world frame iff:
        - we don't already have a goal for this marker_id (if provided)
        - and it's not too close to an existing goal.
        Returns True if added, False if skipped.
        """
        if marker_id is not None and self.has_goal_for_marker(marker_id):
            # print(f'Skipping marker {marker_id}: already stored')
            return False

        if self.has_goal_near(x, y, self.min_goal_spacing):
            # print(f'Skipping goal near ({x:.2f}, {y:.2f}) as duplicate')
            return False

        self.goals.append(Goal(x=x, y=y, marker_id=marker_id))
        return True

    # ---------- current-goal handling ----------

    def current_goal(self) -> Optional[Goal]:
        """Return the currently selected goal, or None if none is selected."""
        if self.current_goal_index is None:
            return None
        if not (0 <= self.current_goal_index < len(self.goals)):
            return None
        return self.goals[self.current_goal_index]

    def select_closest_unvisited(self, robot_x: float, robot_y: float) -> Optional[Goal]:
        """
        Pick the closest unvisited goal to (robot_x, robot_y).
        Sets current_goal_index and returns that Goal, or None if no unvisited goals.
        """
        best_idx = None
        best_d2 = None

        for i, g in enumerate(self.goals):
            if g.visited:
                continue
            dx = g.x - robot_x
            dy = g.y - robot_y
            d2 = dx * dx + dy * dy
            if best_d2 is None or d2 < best_d2:
                best_d2 = d2
                best_idx = i

        if best_idx is None:
            self.current_goal_index = None
            return None

        self.current_goal_index = best_idx
        return self.goals[best_idx]

    def mark_current_goal_visited(self) -> None:
        """Mark the current goal as visited and clear current_goal_index."""
        if self.current_goal_index is None:
            return
        if 0 <= self.current_goal_index < len(self.goals):
            self.goals[self.current_goal_index].visited = True
        self.current_goal_index = None

    def has_unvisited_goals(self) -> bool:
        return any(not g.visited for g in self.goals)


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
    rel_x = dx * math.cos(robot_theta) + dy * math.sin(robot_theta)
    rel_y = -dx * math.sin(robot_theta) + dy * math.cos(robot_theta)
    return rel_x, rel_y
