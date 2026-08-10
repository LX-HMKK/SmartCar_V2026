"""Build validated forward-only Nav2 goals independently from action state."""
import math

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose

from smartcar_task.planning_segments import allows_precise_terminal_through_poses
from smartcar_task.protocols import navigation_behavior_tree
from smartcar_task.waypoints import is_heading_locked


class Nav2GoalFactory:
    """Own native Nav2 behavior-tree selection and pose conversion."""

    def __init__(
        self,
        node,
        precise_behavior_tree,
        transit_behavior_tree,
        through_poses_behavior_tree,
        transit_through_poses_behavior_tree,
        precise_through_poses_behavior_tree,
        return_through_poses_behavior_tree,
    ):
        self._node = node
        self._precise_behavior_tree = str(precise_behavior_tree).strip()
        navigation_behavior_tree(
            "precise", self._precise_behavior_tree, transit_behavior_tree)
        self._transit_behavior_tree = str(transit_behavior_tree).strip()
        navigation_behavior_tree(
            "standard", self._precise_behavior_tree, self._transit_behavior_tree,
            heading_locked=False,
        )
        self._through_poses_behavior_tree = str(
            through_poses_behavior_tree).strip()
        self._transit_through_poses_behavior_tree = str(
            transit_through_poses_behavior_tree).strip()
        self._precise_through_poses_behavior_tree = str(
            precise_through_poses_behavior_tree).strip()
        self._return_through_poses_behavior_tree = str(
            return_through_poses_behavior_tree).strip()

    def navigate_goal(self, waypoint):
        if waypoint.direction != "forward":
            raise ValueError("navigation_goal_requires_forward_direction")
        goal = NavigateToPose.Goal()
        goal.pose = self.pose_stamped(waypoint)
        goal.behavior_tree = navigation_behavior_tree(
            waypoint.goal_profile,
            self._precise_behavior_tree,
            self._transit_behavior_tree,
            heading_locked=is_heading_locked(waypoint),
        )
        return goal

    def navigate_through_goal(self, waypoints):
        goals = tuple(waypoints)
        if len(goals) < 2:
            raise ValueError("navigation_through_requires_multiple_goals")
        if any(waypoint.direction != "forward" for waypoint in goals):
            raise ValueError("navigation_through_requires_forward_direction")
        nonstandard = [
            waypoint.id or str(index)
            for index, waypoint in enumerate(goals)
            if waypoint.goal_profile != "standard"
        ]
        if nonstandard and not allows_precise_terminal_through_poses(goals):
            raise ValueError(
                "navigation_through_nonstandard_goal_profile:"
                + ",".join(nonstandard))
        goal = NavigateThroughPoses.Goal()
        goal.poses = [self.pose_stamped(waypoint) for waypoint in goals]
        goal.behavior_tree = self.through_behavior_tree(
            is_heading_locked(goals[-1]),
            goals[-1].task == "return",
            goals[-1].goal_profile,
        )
        return goal

    def through_behavior_tree(
        self,
        terminal_heading_locked,
        terminal_is_return=False,
        terminal_goal_profile="standard",
    ):
        if terminal_goal_profile == "precise":
            behavior_tree = self._precise_through_poses_behavior_tree
            label = "precise"
        elif terminal_is_return:
            behavior_tree = self._return_through_poses_behavior_tree
            label = "return"
        elif not terminal_heading_locked:
            behavior_tree = self._transit_through_poses_behavior_tree
            label = "transit"
        else:
            behavior_tree = self._through_poses_behavior_tree
            label = "forward"
        if not behavior_tree:
            raise ValueError(f"{label}_through_poses_behavior_tree must not be empty")
        return behavior_tree

    def pose_stamped(self, waypoint):
        qx, qy, qz, qw = waypoint.orientation
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if (
            not math.isfinite(norm)
            or norm <= 1.0e-3
            or abs(norm - 1.0) > 1.0e-3
        ):
            raise ValueError(
                "navigation goal orientation must be a unit quaternion"
            )
        pose = PoseStamped()
        pose.header.stamp = self._node.get_clock().now().to_msg()
        pose.header.frame_id = waypoint.frame_id
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (
            waypoint.position
        )
        pose.pose.orientation.x, pose.pose.orientation.y = qx, qy
        pose.pose.orientation.z, pose.pose.orientation.w = qz, qw
        return pose
