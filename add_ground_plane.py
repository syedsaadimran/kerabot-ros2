#!/usr/bin/env python3
"""
add_ground_plane.py — Adds a persistent Ground Plane Collision Object to MoveIt
================================================================================
Creates a 2m x 2m x 0.1m box collision object in MoveIt's PlanningScene
with its top surface positioned exactly at Z = 0.0m (the base_link mounting plane).

This prevents MoveIt from planning trajectories that pass through the table/floor.

Usage:
    python3 add_ground_plane.py          # Adds the ground plane
    python3 add_ground_plane.py --remove # Removes the ground plane
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene


def create_ground_plane_co(remove=False):
    co = CollisionObject()
    co.header.frame_id = "base_link"
    co.id = "ground_plane"

    if remove:
        co.operation = CollisionObject.REMOVE
        return co

    co.operation = CollisionObject.ADD

    # Create a 2m x 2m x 0.1m thick table/floor box primitive
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = [2.0, 2.0, 0.1]  # X, Y, Z size in metres

    # Position box so top surface is exactly at Z = 0.0m
    # Center is at Z = -0.05m (half of thickness 0.1m)
    pose = Pose()
    pose.position.x = 0.0
    pose.position.y = 0.0
    pose.position.z = -0.05
    pose.orientation.w = 1.0

    co.primitives.append(box)
    co.primitive_poses.append(pose)
    return co


def main():
    parser = argparse.ArgumentParser(description="Add/Remove ground plane in MoveIt")
    parser.add_argument("--remove", action="store_true", help="Remove ground plane from scene")
    args = parser.parse_args()

    rclpy.init()
    node = Node("add_ground_plane_node")

    # Service client for ApplyPlanningScene
    client = node.create_client(ApplyPlanningScene, "apply_planning_scene")
    pub = node.create_publisher(PlanningScene, "planning_scene", 10)

    node.get_logger().info("Connecting to /apply_planning_scene service...")
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("ApplyPlanningScene service not available! Is move_group running?")
        sys.exit(1)

    co = create_ground_plane_co(remove=args.remove)

    scene = PlanningScene()
    scene.is_diff = True
    scene.world.collision_objects.append(co)

    req = ApplyPlanningScene.Request()
    req.scene = scene

    # 1. Apply via service
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

    if future.result() and future.result().success:
        action = "Removed" if args.remove else "Added"
        node.get_logger().info(f"✅ Successfully {action} 'ground_plane' (Z_top = 0.0m) to MoveIt planning scene.")
    else:
        node.get_logger().error("❌ Failed to apply planning scene update via service. Publishing to topic fallback...")
        pub.publish(scene)
        time.sleep(0.5)

    # 2. Also publish to /planning_scene topic for RViz / MoveGroup sync
    for _ in range(5):
        pub.publish(scene)
        time.sleep(0.1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
