#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-
from __future__ import annotations

import ast
import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
from pmadmu_planner.msg import Formation, GoalPose, Trajectory, Trajectories, FollowerFeedback
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class MainController:
    def __init__(self) -> None:
        rospy.init_node("MainController")

        self.start_pose: None|PoseStamped = None
        self.goal_pose: None|PoseStamped = None
        self.markers: dict[str, Marker] = {}

        robot_names : str = str(rospy.get_param('~robot_names'))
        self.unique_mir_ids : list[str] = robot_names.split(',')


        self.rviz_goal_subscriber : rospy.Subscriber = rospy.Subscriber("/formation_planner/start_pos", PoseStamped, self.set_startpoint)
        self.rviz_goal_subscriber : rospy.Subscriber = rospy.Subscriber("/formation_planner/goal_pos", PoseStamped, self.set_goalpoint)
        return None

    
    def set_startpoint(self, start_pos : PoseStamped) -> None:
        rospy.loginfo(f"[MainC] Start Pose received at ({start_pos.pose.position.x:.3f} / {start_pos.pose.position.y:.3f}), phi = {np.rad2deg(start_pos.pose.orientation.z):.2f}°")
        self.start_pose = start_pos
        # Robot Visualization in Rviz
        robot_starting_poses : list[Pose]|None = self.calculate_robot_positions(self.start_pose)
        if robot_starting_poses is not None:
            self.visualize_positions("start_poses", robot_starting_poses, color=(0, 1.0, 0.5))
        return None
    

    def set_goalpoint(self, goal_pos : PoseStamped) -> None:
        rospy.loginfo(f"[MainC] Goal Pose received at ({goal_pos.pose.position.x:.3f} / {goal_pos.pose.position.y:.3f}), phi = {np.rad2deg(goal_pos.pose.orientation.z):.2f}°")
        self.goal_pose = goal_pos
        # Robot Visualization in Rviz
        robot_goal_poses : list[Pose]|None = self.calculate_robot_positions(self.goal_pose)
        if robot_goal_poses is not None:
            self.visualize_positions("goal_poses", robot_goal_poses, color=(1.0, 0.5, 0))
        return None
    

    def calculate_robot_positions(self, leader_pose : PoseStamped) -> list[Pose] | None:
        robot_positions : str = str(rospy.get_param('~robot_positions'))
        robot_positions_list = ast.literal_eval(robot_positions)

        if len(self.unique_mir_ids) != len(robot_positions_list):
            rospy.logerr(f"[MainC] There must be the same number of robots ({len(self.unique_mir_ids)}) and positions ({len(robot_positions_list)}). Please adjust the launch file!")
            return None
        
        robot_poses : list[Pose] = []

        leader_quarternion = (
            leader_pose.pose.orientation.x,
            leader_pose.pose.orientation.y,
            leader_pose.pose.orientation.z,
            leader_pose.pose.orientation.w)

        _, _, leader_yaw = euler_from_quaternion(leader_quarternion)
        
        for index, robot_name in enumerate(self.unique_mir_ids):
            robot_position : list[float] = robot_positions_list[index]
            if len(robot_position) != 3:
                rospy.logerr(f"[MainC] Position for {robot_name} is invalid. Must contain 3 Values for [x, y, rotation] but contains {len(robot_name)} Values. Please adjust the launch file!")
                return None
            robot_pose : Pose = Pose()
            robot_pose.position.x = leader_pose.pose.position.x + robot_position[0] * np.cos(leader_yaw) - robot_position[1] * np.sin(leader_yaw)
            robot_pose.position.y = leader_pose.pose.position.y + robot_position[0] * np.sin(leader_yaw) + robot_position[1] * np.cos(leader_yaw)
            robot_pose.position.z = 0.0

            yaw : float = leader_yaw + robot_position[2]

            q = quaternion_from_euler(0, 0, yaw)
            robot_pose.orientation.x = q[0]
            robot_pose.orientation.y = q[1]
            robot_pose.orientation.z = q[2]
            robot_pose.orientation.w = q[3]
            robot_poses.append(robot_pose)
            rospy.logdebug(f"[MainC] {robot_name} received a goal position. relative: {robot_position} -> absolute: [{robot_pose.position.x}, {robot_pose.position.y}]; angle {robot_pose.orientation.z}")
        return robot_poses


    def visualize_positions(self, namespace: str, robot_positions : list[Pose], color:tuple[float, float, float]) -> None:
        marker_array : MarkerArray = MarkerArray()
        marker_array.markers = []
        for i, pose in enumerate(robot_positions):
            marker: Marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.type = Marker.CUBE

            marker.id = i
            marker.pose = pose
            marker.scale.x = 1.0
            marker.scale.y = 0.6
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r, marker.color.g, marker.color.b = color
            marker_array.markers.append(marker)
        marker_array_publisher: rospy.Publisher = rospy.Publisher(f'/formation_planner/{namespace}', MarkerArray, queue_size=10, latch=True)
        marker_array_publisher.publish(marker_array)
        return None



if __name__ == '__main__':
    main_controller : MainController = MainController()
    rospy.spin()
