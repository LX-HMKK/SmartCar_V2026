"""Build validated Nav2 goals independently from their action lifecycle."""
import math

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose

from smartcar_task.planning_segments import (
    allows_precise_terminal_through_poses,
    allows_reverse_handoff_through_poses,
)
from smartcar_task.protocols import navigation_behavior_tree
from smartcar_task.waypoints import is_heading_locked


class Nav2GoalFactory:
    """Own behavior-tree selection and pose conversion for Nav2 actions."""

    def __init__(
        self,
        node,
        reverse_behavior_tree,
        reverse_handoff_behavior_tree,
        precise_forward_behavior_tree,
        forward_transit_behavior_tree,
        through_poses_behavior_tree="",
        reverse_through_poses_behavior_tree="",
        reverse_locked_through_poses_behavior_tree="",
        reverse_return_through_poses_behavior_tree="",
        forward_transit_through_poses_behavior_tree="",
        forward_precise_through_poses_behavior_tree="",
        forward_return_through_poses_behavior_tree="",
    ):
        self._node = node
        self._reverse_behavior_tree = str(reverse_behavior_tree).strip()
        navigation_behavior_tree(True, self._reverse_behavior_tree)
        self._reverse_handoff_behavior_tree = str(
            reverse_handoff_behavior_tree).strip()
        navigation_behavior_tree(
            True,
            self._reverse_behavior_tree,
            goal_profile="reverse_handoff",
            reverse_handoff_behavior_tree=self._reverse_handoff_behavior_tree,
        )
        self._precise_forward_behavior_tree = str(
            precise_forward_behavior_tree).strip()
        navigation_behavior_tree(
            False,
            self._reverse_behavior_tree,
            goal_profile="precise",
            precise_forward_behavior_tree=self._precise_forward_behavior_tree,
        )
        self._forward_transit_behavior_tree = str(
            forward_transit_behavior_tree).strip()
        navigation_behavior_tree(
            False,
            self._reverse_behavior_tree,
            forward_transit_behavior_tree=self._forward_transit_behavior_tree,
            heading_locked=False,
        )
        self._forward_transit_through_poses_behavior_tree = str(
            forward_transit_through_poses_behavior_tree).strip()
        self._forward_precise_through_poses_behavior_tree = str(
            forward_precise_through_poses_behavior_tree).strip()
        self._forward_return_through_poses_behavior_tree = str(
            forward_return_through_poses_behavior_tree).strip()
        self._through_poses_behavior_tree = str(
            through_poses_behavior_tree).strip()
        self._reverse_through_poses_behavior_tree = str(
            reverse_through_poses_behavior_tree).strip()
        self._reverse_locked_through_poses_behavior_tree = str(
            reverse_locked_through_poses_behavior_tree).strip()
        self._reverse_return_through_poses_behavior_tree = str(
            reverse_return_through_poses_behavior_tree).strip()

    def navigate_goal(self, waypoint, reverse_direction):
        behavior_tree = navigation_behavior_tree(
            reverse_direction,
            self._reverse_behavior_tree,
            goal_profile=waypoint.goal_profile,
            precise_forward_behavior_tree=self._precise_forward_behavior_tree,
            reverse_handoff_behavior_tree=self._reverse_handoff_behavior_tree,
            forward_transit_behavior_tree=self._forward_transit_behavior_tree,
            heading_locked=is_heading_locked(waypoint),
        )
        goal = NavigateToPose.Goal()
        goal.pose = self.pose_stamped(waypoint)
        goal.behavior_tree = behavior_tree
        return goal

    def navigate_through_goal(self, waypoints, reverse_direction):
        goals = tuple(waypoints)
        if len(goals) < 2:
            raise ValueError("navigation_through_requires_multiple_goals")
        if any(waypoint.direction != goals[0].direction for waypoint in goals):
            raise ValueError("navigation_through_direction_mismatch")
        nonstandard = [
            waypoint.id or str(index)
            for index, waypoint in enumerate(goals)
            if waypoint.goal_profile != "standard"
        ]
        if nonstandard and not (
            allows_reverse_handoff_through_poses(goals)
            or allows_precise_terminal_through_poses(goals)
        ):
            raise ValueError(
                "navigation_through_nonstandard_goal_profile:"
                + ",".join(nonstandard))
        goal = NavigateThroughPoses.Goal()
        goal.poses = [self.pose_stamped(waypoint) for waypoint in goals]
        goal.behavior_tree = self.through_behavior_tree(
            reverse_direction,
            is_heading_locked(goals[-1]),
            goals[-1].task == "return",
            goals[-1].goal_profile,
        )
        return goal

    def through_behavior_tree(
        self,
        reverse_direction,
        terminal_heading_locked,
        terminal_is_return=False,
        terminal_goal_profile="standard",
    ):
        if not reverse_direction and terminal_goal_profile == "precise":
            behavior_tree = self._forward_precise_through_poses_behavior_tree
            direction = "forward_precise"
        elif not reverse_direction and terminal_is_return:
            behavior_tree = self._forward_return_through_poses_behavior_tree
            direction = "forward_return"
        elif reverse_direction and terminal_heading_locked and terminal_is_return:
            behavior_tree = self._reverse_return_through_poses_behavior_tree
            direction = "reverse_return"
        elif reverse_direction and terminal_heading_locked:
            behavior_tree = self._reverse_locked_through_poses_behavior_tree
            direction = "reverse_locked"
        elif reverse_direction:
            behavior_tree = self._reverse_through_poses_behavior_tree
            direction = "reverse"
        elif not terminal_heading_locked:
            behavior_tree = self._forward_transit_through_poses_behavior_tree
            direction = "forward_transit"
        else:
            behavior_tree = self._through_poses_behavior_tree
            direction = "forward"
        if not behavior_tree:
            raise ValueError(
                f"{direction}_through_poses_behavior_tree must not be empty")
        return behavior_tree

    def pose_stamped(self, waypoint):
        qx, qy, qz, qw = waypoint.orientation
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if not math.isfinite(norm) or (
            norm > 1.0e-3 and abs(norm - 1.0) > 1.0e-3
        ):
            raise ValueError(
                "navigation goal orientation must be a unit quaternion or "
                "the free-heading zero sentinel"
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
