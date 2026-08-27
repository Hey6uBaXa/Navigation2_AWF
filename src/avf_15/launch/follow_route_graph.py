#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import ComputeRoute
from nav2_msgs.action import FollowPath


class RouteGraphFollower(Node):
    def __init__(self):
        super().__init__('route_graph_follower')

        self.declare_parameter('start_id', 0)
        self.declare_parameter('goal_id', 5)
        self.declare_parameter('controller_id', 'FollowPath')
        self.declare_parameter('goal_checker_id', 'general_goal_checker')

        self.start_id = int(self.get_parameter('start_id').value)
        self.goal_id = int(self.get_parameter('goal_id').value)
        self.controller_id = self.get_parameter('controller_id').value
        self.goal_checker_id = self.get_parameter('goal_checker_id').value

        self.compute_route_client = ActionClient(
            self,
            ComputeRoute,
            '/compute_route'
        )

        self.follow_path_client = ActionClient(
            self,
            FollowPath,
            '/follow_path'
        )

    def compute_route(self):
        self.get_logger().info('Waiting for /compute_route...')

        if not self.compute_route_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('/compute_route not available')
            return None

        goal_msg = ComputeRoute.Goal()
        goal_msg.start_id = self.start_id
        goal_msg.goal_id = self.goal_id

        # Важно:
        # False = использовать ID узлов графа, а не Pose
        goal_msg.use_poses = False

        # False = не брать start из TF, использовать start_id
        goal_msg.use_start = False

        self.get_logger().info(
            f'Computing route: {self.start_id} -> {self.goal_id}'
        )

        send_goal_future = self.compute_route_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('ComputeRoute goal rejected')
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result

        if result.error_code != 0:
            self.get_logger().error(
                f'ComputeRoute failed. error_code={result.error_code}'
            )
            return None

        if len(result.path.poses) == 0:
            self.get_logger().error('ComputeRoute returned empty path')
            return None

        self.get_logger().info(
            f'Route path received. Poses: {len(result.path.poses)}'
        )

        return result.path

    def follow_path(self, path):
        self.get_logger().info('Waiting for /follow_path...')

        if not self.follow_path_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('/follow_path not available')
            return False

        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        goal_msg.controller_id = self.controller_id
        goal_msg.goal_checker_id = self.goal_checker_id

        self.get_logger().info(
            f'Sending path to controller: {self.controller_id}'
        )

        send_goal_future = self.follow_path_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('FollowPath goal rejected')
            return False

        self.get_logger().info('FollowPath goal accepted')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()

        self.get_logger().info(
            f'FollowPath finished with status: {result.status}'
        )

        return True


def main():
    rclpy.init()

    node = RouteGraphFollower()

    path = node.compute_route()

    if path is None:
        node.get_logger().error('No route path. Exit.')
        node.destroy_node()
        rclpy.shutdown()
        return

    node.follow_path(path)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()