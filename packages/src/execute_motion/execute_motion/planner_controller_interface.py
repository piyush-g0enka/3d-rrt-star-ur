# This file contains code to implement RRT strat and the entire motion pipeline for pick and place

import sys
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from waypoint_msgs.srv import WaypointFollower
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Point
from rclpy.parameter import Parameter
import numpy as np
import math
import random
from scipy.spatial import cKDTree
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import math
import random
from scipy.spatial import cKDTree
import rclpy
# from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# RRT star planner node


class PlannerControllerInterface(Node):

    def __init__(self):
        super().__init__('planner_controller_node')
        sim_time = Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
        self.set_parameters([sim_time])

        # Start position of robot
        self._pickup_start = (-0.076, 0.350, 0.1)

        # Pickeup position
        self._pickup_end = (0.36, 0.36, 0.05)
        self._drop_start = self._pickup_end

        # Drop position
        self._drop_end = (0.36, -0.36, 0.1)

        self._moved_to_pick_up_flag = False
        self._picked_up_flag = False
        self._moved_to_drop_flag = False
        self._dropped_flag = False
        main_timer_cb_group = MutuallyExclusiveCallbackGroup()
        self._way_point_cli = self.create_client(
            WaypointFollower, 'waypoint_follower')
        self._vacuum_gripper_cli = self.create_client(
            SetBool, '/vacuum_gripper/custom_switch')

        while not self._way_point_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('waypoint service not available, waiting again...')

        self._way_point_request = WaypointFollower.Request()
        self._way_point_request.waypoints = []
        self._main_timer = self.create_timer(
            15, self._main_timer_cb, callback_group=main_timer_cb_group)

    # RRT star algorithm
    def path_planner(self, start, finish):

        # Defining obstacles

        size_factor = 1
        cube3_z = 0.0

        vertices = np.array([[-0.6, 0.2, cube3_z],
                            [-0.4, 0.2, cube3_z],
                            [-0.4, 0.4, cube3_z],
                            [-0.6, 0.4, cube3_z],
                            [-0.6, 0.2, cube3_z + 0.2],
                            [-0.4, 0.2, cube3_z + 0.2],
                            [-0.4, 0.4, cube3_z + 0.2],
                            [-0.6, 0.4, cube3_z + 0.2]])

        faces = [
            [0, 1, 2, 3],
            [4, 5, 6, 7],
            [0, 1, 5, 4],
            [2, 3, 7, 6],
            [0, 3, 7, 4],
            [1, 2, 6, 5]
        ]

        vertices2 = np.array([[0.5, -0.3, cube3_z],
                              [0.7, -0.3, cube3_z],
                              [0.7, -0.1, cube3_z],
                              [0.5, -0.1, cube3_z],
                              [0.5, -0.3, cube3_z + 0.6],
                              [0.7, -0.3, cube3_z + 0.6],
                              [0.7, -0.1, cube3_z + 0.6],
                              [0.5, -0.1, cube3_z + 0.6]])

        faces2 = [
            [0, 1, 2, 3],
            [4, 5, 6, 7],
            [0, 1, 5, 4],
            [2, 3, 7, 6],
            [0, 3, 7, 4],
            [1, 2, 6, 5]
        ]

        vertices3 = np.array([[-0.2, -0.2, cube3_z],
                              [0.2, -0.2, cube3_z],
                              [0.2, 0.2, cube3_z],
                              [-0.2, 0.2, cube3_z],
                              [-0.2, -0.2, cube3_z + 0.75],
                              [0.2, -0.2, cube3_z + 0.75],
                              [0.2, 0.2, cube3_z + 0.75],
                              [-0.2, 0.2, cube3_z + 0.75]])

        faces3 = [
            [0, 1, 2, 3],
            [4, 5, 6, 7],
            [0, 1, 5, 4],
            [2, 3, 7, 6],
            [0, 3, 7, 4],
            [1, 2, 6, 5]
        ]

        # Adding obstacle number 4 (vertices4) with updated coordinates
        vertices4 = np.array([[0.1, -0.237, cube3_z],
                              [0.5, -0.237, cube3_z],
                              [0.5, -0.037, cube3_z],
                              [0.1, -0.037, cube3_z],
                              [0.1, -0.237, cube3_z + 0.2],
                              [0.5, -0.237, cube3_z + 0.2],
                              [0.5, -0.037, cube3_z + 0.2],
                              [0.1, -0.037, cube3_z + 0.2]])

        faces4 = [
            [0, 1, 2, 3],
            [4, 5, 6, 7],
            [0, 1, 5, 4],
            [2, 3, 7, 6],
            [0, 3, 7, 4],
            [1, 2, 6, 5]
        ]

        # Calculate the center points of the obstacles
        cube1_center = np.mean(vertices, axis=0)
        cube2_center = np.mean(vertices2, axis=0)
        cube3_center = np.mean(vertices3, axis=0)
        cube4_center = np.mean(vertices4, axis=0)
        bloat3 = 1.3

        # Modify the vertices of the obstacles based on their center points and the size factor
        vertices = cube1_center + size_factor * (vertices - cube1_center)
        vertices2 = cube2_center + size_factor * (vertices2 - cube2_center)
        vertices3 = cube3_center + bloat3 * (vertices3 - cube3_center)
        vertices4 = cube4_center + (vertices4 - cube4_center)

        plane_vertices = np.array([[-1, -1, cube3_z],
                                   [1, -1, cube3_z],
                                   [1, 1, cube3_z],
                                   [-1, 1, cube3_z]])

        plane_faces = [[0, 1, 2, 3]]

        d1, d4, d5, d6 = 0.1519, 0.1149, 0.0819, 0.0819
        a2, a3 = -0.24365, -0.21325

        joint_limits = [
            (-np.pi, np.pi),
            (-np.pi, np.pi),
            (-np.pi, np.pi),
            (-np.pi, np.pi),
            (-np.pi, np.pi),
            (-np.pi, np.pi)
        ]

        def forward_kinematics(joint_angles):
            theta1, theta2, theta3, theta4, theta5, theta6 = joint_angles
            c1, c2, c3, c4, c5, c6 = np.cos(joint_angles)
            s1, s2, s3, s4, s5, s6 = np.sin(joint_angles)
            T01 = np.array([[c1, -s1, 0, 0], [s1, c1, 0, 0],
                           [0, 0, 1, d1], [0, 0, 0, 1]])
            T12 = np.array([[c2, -s2, 0, 0], [0, 0, -1, 0],
                           [s2, c2, 0, 0], [0, 0, 0, 1]])
            T23 = np.array([[c3, -s3, 0, a2], [s3, c3, 0, 0],
                           [0, 0, 1, 0], [0, 0, 0, 1]])
            T34 = np.array([[c4, -s4, 0, a3], [s4, c4, 0, 0],
                           [0, 0, 1, d4], [0, 0, 0, 1]])
            T45 = np.array([[c5, -s5, 0, 0], [0, 0, -1, -d5],
                           [s5, c5, 0, 0], [0, 0, 0, 1]])
            T56 = np.array([[c6, -s6, 0, 0], [0, 0, 1, d6],
                           [-s6, -c6, 0, 0], [0, 0, 0, 1]])
            T02 = np.dot(T01, T12)
            T03 = np.dot(T02, T23)
            T04 = np.dot(T03, T34)
            T05 = np.dot(T04, T45)
            T06 = np.dot(T05, T56)
            end_effector_pos = T06[:3, 3]
            return end_effector_pos

        # Points of robot workspace
        def generate_sphere_points(num_points, max_reach):
            points = []
            for _ in range(num_points):
                x = random.uniform(-max_reach, max_reach)
                y = random.uniform(-max_reach, max_reach)
                z = random.uniform(-max_reach, max_reach)
                if x**2 + y**2 + z**2 <= max_reach**2:
                    points.append([x, y, z])
            return np.array(points)

        max_reach = d1 + abs(a2) + abs(a3) + d4 + d5 + d6
        num_points = 2000
        sphere_points = generate_sphere_points(num_points, max_reach)
        STEP_SIZE = 0.05
        MAX_ITERATIONS = 10000
        GOAL_TOLERANCE = 0.02
        SEARCH_RADIUS = 0.1
        START = start
        GOAL = finish

        # Euclidean distance
        def distance(p1, p2):
            return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 + (p1[2] - p2[2])**2)

        def find_nearest_node(tree, point):
            distances, indices = tree.query(point)
            return tuple(tree.data[indices])

        # Check if path is inside obstacle

        def is_inside_cube(point):
            return ((vertices[0][0] <= point[0] <= vertices[1][0] and
                    vertices[0][1] <= point[1] <= vertices[2][1] and
                    vertices[0][2] <= point[2] <= vertices[4][2]) or
                    (vertices2[0][0] <= point[0] <= vertices2[1][0] and
                    vertices2[0][1] <= point[1] <= vertices2[2][1] and
                    vertices2[0][2] <= point[2] <= vertices2[4][2]) or
                    (vertices3[0][0] <= point[0] <= vertices3[1][0] and
                    vertices3[0][1] <= point[1] <= vertices3[2][1] and
                    vertices3[0][2] <= point[2] <= vertices3[4][2]) or
                    (vertices4[0][0] <= point[0] <= vertices4[1][0] and
                    vertices4[0][1] <= point[1] <= vertices4[2][1] and
                    vertices4[0][2] <= point[2] <= vertices4[4][2]))

        # check if node is below robot table
        def is_inside_plane(point):
            return point[2] <= cube3_z

        def is_intersecting_cube(p1, p2):
            cube_faces = [
                [(vertices[i][0], vertices[i][1], vertices[i][2]) for i in face] for face in faces
            ]
            cube_faces2 = [
                [(vertices2[i][0], vertices2[i][1], vertices2[i][2]) for i in face] for face in faces2
            ]
            cube_faces3 = [
                [(vertices3[i][0], vertices3[i][1], vertices3[i][2]) for i in face] for face in faces3
            ]
            cube_faces4 = [
                [(vertices4[i][0], vertices4[i][1], vertices4[i][2]) for i in face] for face in faces4
            ]

            for face in cube_faces + cube_faces2 + cube_faces3 + cube_faces4:
                v1, v2, v3, _ = [np.array(v) for v in face]
                normal = np.cross(v2 - v1, v3 - v1)
                min_proj_cube = min(np.dot(v, normal) for v in face)
                max_proj_cube = max(np.dot(v, normal) for v in face)
                min_proj_line = min(np.dot(p1, normal), np.dot(p2, normal))
                max_proj_line = max(np.dot(p1, normal), np.dot(p2, normal))
                if max_proj_line < min_proj_cube or min_proj_line > max_proj_cube:
                    return False
            return True

        def is_intersecting_plane(p1, p2):
            plane_face = [(plane_vertices[i][0], plane_vertices[i]
                           [1], plane_vertices[i][2]) for i in plane_faces[0]]
            for i in range(len(plane_face)):
                v1 = plane_face[i]
                v2 = plane_face[(i + 1) % len(plane_face)]
                v3 = plane_face[(i + 2) % len(plane_face)]
                if is_intersecting_triangle(p1, p2, v1, v2, v3):
                    return True
            return False

        def is_intersecting_triangle(p1, p2, v1, v2, v3):
            p1 = np.array(p1)
            p2 = np.array(p2)
            v1 = np.array(v1)
            v2 = np.array(v2)
            v3 = np.array(v3)
            e1 = v2 - v1
            e2 = v3 - v1
            n = np.cross(e1, e2)
            det = -np.dot(p2 - p1, n)
            if det == 0:
                return False
            t = np.dot(v1 - p1, n) / det
            if t < 0 or t > 1:
                return False
            q = p1 + t * (p2 - p1)
            if np.dot(np.cross(v2 - v1, q - v1), n) < 0:
                return False
            if np.dot(np.cross(v3 - v2, q - v2), n) < 0:
                return False
            if np.dot(np.cross(v1 - v3, q - v3), n) < 0:
                return False
            return True

        def rewire(tree, parent, cost, new_node):
            near_indices = tree.query_ball_point(new_node, SEARCH_RADIUS)
            for i in near_indices:
                near_node = tuple(tree.data[i])
                if not is_intersecting_cube(near_node, new_node):
                    new_cost = cost[new_node] + distance(near_node, new_node)
                    if near_node not in cost or new_cost < cost[near_node]:
                        parent[near_node] = new_node
                        cost[near_node] = new_cost

        # Make path smoother
        def prune_path(path, threshold=0.1):
            pruned_path = [path[0]]
            current_node = path[0]
            for i in range(1, len(path)):
                if distance(current_node, path[i]) > threshold:
                    pruned_path.append(path[i])
                    current_node = path[i]
            if pruned_path[-1] != path[-1]:
                pruned_path.append(path[-1])
            return pruned_path

        # RRT star alogrithm
        def rrt_star(start, goal):
            tree = cKDTree([start])
            parent = {start: None}
            cost = {start: 0}
            # fig = plt.figure(figsize=(8, 8))
            # ax = fig.add_subplot(111, projection='3d')
            # ax.scatter(sphere_points[:, 0], sphere_points[:, 1], sphere_points[:, 2], c='b', marker='.', alpha=0.1)
            # ax.scatter(start[0], start[1], start[2], c='r', marker='o', s=100, label='Start')
            # ax.scatter(goal[0], goal[1], goal[2], c='g', marker='o', s=100, label='Goal')
            # ax.scatter(0, 0, 0, c='k', marker='x', s=100, label='Origin')
            # polygons = [Poly3DCollection([vertices[face]]) for face in faces]
            # polygons2 = [Poly3DCollection([vertices2[face]]) for face in faces2]
            # polygons3 = [Poly3DCollection([vertices3[face]]) for face in faces3]
            # polygons4 = [Poly3DCollection([vertices4[face]]) for face in faces4]
            # for poly in polygons + polygons2 + polygons3 + polygons4:
            #     poly.set_color('k')
            #     poly.set_alpha(0.5)
            #     ax.add_collection3d(poly)
            # plane_polygon = Poly3DCollection([plane_vertices[plane_faces[0]]])
            # plane_polygon.set_color('b')
            # plane_polygon.set_alpha(0.0)
            # ax.add_collection3d(plane_polygon)
            # plt.ion()
            # plt.show()
            for i in range(MAX_ITERATIONS):
                if i % 5 == 0:
                    rand_point = GOAL
                else:
                    rand_point = tuple(
                        sphere_points[random.randint(0, len(sphere_points) - 1)])
                nearest_node = find_nearest_node(tree, rand_point)
                direction = (rand_point[0] - nearest_node[0], rand_point[1] -
                             nearest_node[1], rand_point[2] - nearest_node[2])
                direction_norm = math.sqrt(
                    direction[0]**2 + direction[1]**2 + direction[2]**2)
                if direction_norm > STEP_SIZE:
                    direction = (direction[0] * STEP_SIZE / direction_norm, direction[1] *
                                 STEP_SIZE / direction_norm, direction[2] * STEP_SIZE / direction_norm)
                new_node = (nearest_node[0] + direction[0], nearest_node[1] +
                            direction[1], nearest_node[2] + direction[2])
                if is_inside_cube(new_node):
                    continue
                if is_inside_plane(new_node):
                    continue
                if is_intersecting_cube(nearest_node, new_node) or is_intersecting_plane(nearest_node, new_node):
                    continue
                tree = cKDTree(list(tree.data) + [new_node])
                parent[new_node] = nearest_node
                cost[new_node] = cost[nearest_node] + \
                    distance(nearest_node, new_node)
                rewire(tree, parent, cost, new_node)
                parent_node = parent[new_node]
                # ax.plot([parent_node[0], new_node[0]], [parent_node[1], new_node[1]], [parent_node[2], new_node[2]], c='b', linewidth=1)
                # plt.pause(0.001)
                if distance(new_node, goal) < GOAL_TOLERANCE:
                    final_node = new_node
                    break

            if final_node is not None:
                path = []
                current_node = final_node
                while current_node is not None:
                    path.append(current_node)
                    current_node = parent[current_node]
                path.reverse()

                final_path = path.copy()

                path_x, path_y, path_z = zip(*final_path)
                # ax.scatter(path_x, path_y, path_z, c='r', marker='o', s=20, label='Original Path')

                pruned_path = prune_path(final_path)
                pruned_path_x, pruned_path_y, pruned_path_z = zip(*pruned_path)
                # ax.scatter(pruned_path_x, pruned_path_y, pruned_path_z, c='g', marker='o', s=40, label='Pruned Path')

                # ax.legend()
                # plt.ioff()
                # plt.show()
                return final_path

            # ax.legend()
            # plt.ioff()
            # plt.show()
            return None

        path = rrt_star(START, GOAL)

        if path is not None:
            self.get_logger().info('path found')
            pruned_path = prune_path(path)
            for node in pruned_path:
                p = Point()
                p.x, p.y, p.z = node
                self._way_point_request.waypoints.append(p)
            return True
        else:
            self.get_logger().info('path not found')
            return False

    # Pick and place code

    def _main_timer_cb(self):
        self.get_logger().info('inside main_timer_cb')
        path_planning_status = False
        waypoint_status = False
        gripper_status = False

        # Go to pickup location
        if self._moved_to_pick_up_flag == False:
            self.get_logger().info('Inside pickup path planner')
            path_planning_status = self.path_planner(
                self._pickup_start, self._pickup_end)
            if path_planning_status:
                waypoint_status = self.waypoint_follower_cb(
                    self._way_point_request)
                if waypoint_status:
                    self._way_point_request.waypoints = []
                    self._moved_to_pick_up_flag = True
                    self.get_logger().info('Successfully reached pickup position')
                else:
                    self.get_logger().info('Waypoint status - False')

        # Turn ON gripper
        if self._moved_to_pick_up_flag == True and self._picked_up_flag == False:
            self.get_logger().info('Calling gripper to turn on')
            gripper_status = self.gripper_client_cb(True)
            if gripper_status:
                self._picked_up_flag = True
                # add wait here
                for kkk in range(0, 10000000):
                    pass
                self.get_logger().info('Successfully picked up')

        # Go to drop location
        if self._picked_up_flag == True and self._moved_to_drop_flag == False:
            path_planning_status = self.path_planner(
                self._drop_start, self._drop_end)
            if path_planning_status:
                waypoint_status = self.waypoint_follower_cb(
                    self._way_point_request)
                if waypoint_status:
                    self.get_logger().info('Successfully reached drop position')
                    self._way_point_request.waypoints = []
                    self._moved_to_drop_flag = True

                else:
                    self.get_logger().info('Waypoint status - False')

        # Turn OFF gripper
        if self._moved_to_drop_flag == True and self._dropped_flag == False:
            self.get_logger().info('Calling gripper to turn off')

            for kkk in range(0, 10000000):
                pass
            gripper_status = self.gripper_client_cb(False)
            if gripper_status:
                self._dropped_flag = True
                self.get_logger().info('Successfully dropped')

    # MoveGroup client

    def waypoint_follower_cb(self, way_point_req):
        self._way_point_request = way_point_req
        response = self._way_point_cli.call(self._way_point_request)
        return response.result if response is not None else False

    # Vacuum gripper client
    def gripper_client_cb(self, action):
        request = SetBool.Request()
        request.data = action
        response = self._vacuum_gripper_cli.call(request)
        if response is not None:
            return True
        else:
            self.get_logger().info('Failure in gripper client')
            return False
