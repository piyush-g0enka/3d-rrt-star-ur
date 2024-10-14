# This file contains code for RRT algorithm

import matplotlib.pyplot as plt
import numpy as np
import math
import random
from scipy.spatial import cKDTree
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

size_factor = 1.5
cube3_z = 0.1

# We define all the collision objects

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

# Adding obstacle number 4 (vertices4)
vertices4 = np.array([[0.1, -0.237, cube3_z],
                      [0.5, -0.237, cube3_z],
                      [0.5, -0.037, cube3_z],
                      [0.1, -0.037, cube3_z],
                      [0.1, -0.237, cube3_z + 0.4],
                      [0.5, -0.237, cube3_z + 0.4],
                      [0.5, -0.037, cube3_z + 0.4],
                      [0.1, -0.037, cube3_z + 0.4]])

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

# Modify the vertices of the obstacles based on their center points and the size factor
vertices = cube1_center + size_factor * (vertices - cube1_center)
vertices2 = cube2_center + size_factor * (vertices2 - cube2_center)
vertices3 = cube3_center + (vertices3 - cube3_center)
vertices4 = cube4_center + (vertices4 - cube4_center)

plane_vertices = np.array([[-1, -1, cube3_z],
                           [1, -1, cube3_z],
                           [1, 1, cube3_z],
                           [-1, 1, cube3_z]])

plane_faces = [[0, 1, 2, 3]]

d1, d4, d5, d6 = 0.1519, 0.1149, 0.0819, 0.0819
a2, a3 = -0.24365, -0.21325


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
GOAL_TOLERANCE = 0.15
START = (0.36, 0.36, 0.15)
GOAL = (0.36, -0.36, 0.15)


def distance(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 + (p1[2] - p2[2])**2)


def find_nearest_node(tree, point):
    distances, indices = tree.query(point)
    return tuple(tree.data[indices])


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


def is_inside_plane(point):
    return point[2] <= cube3_z


def is_intersecting_cube(p1, p2):
    cube_faces = [[(vertices[i][0], vertices[i][1], vertices[i][2])
                   for i in face] for face in faces]
    cube_faces2 = [[(vertices2[i][0], vertices2[i][1], vertices2[i][2])
                    for i in face] for face in faces2]
    cube_faces3 = [[(vertices3[i][0], vertices3[i][1], vertices3[i][2])
                    for i in face] for face in faces3]
    cube_faces4 = [[(vertices4[i][0], vertices4[i][1], vertices4[i][2])
                    for i in face] for face in faces4]

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
    plane_face = [(plane_vertices[i][0], plane_vertices[i][1],
                   plane_vertices[i][2]) for i in plane_faces[0]]
    v1, v2, v3, _ = [np.array(v) for v in plane_face]
    normal = np.cross(v2 - v1, v3 - v1)
    min_proj_plane = min(np.dot(v, normal) for v in plane_face)
    max_proj_plane = max(np.dot(v, normal) for v in plane_face)
    min_proj_line = min(np.dot(p1, normal), np.dot(p2, normal))
    max_proj_line = max(np.dot(p1, normal), np.dot(p2, normal))
    if max_proj_line < min_proj_plane or min_proj_line > max_proj_plane:
        return False
    return True


def is_path_valid(p1, p2):
    if is_inside_cube(p1) or is_inside_cube(p2):
        return False
    if is_inside_plane(p1) or is_inside_plane(p2):
        return False
    if is_intersecting_cube(p1, p2) or is_intersecting_plane(p1, p2):
        return False
    return True

# Smoothen path


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

# RRT main algorithm


def rrt(start, goal):
    tree = cKDTree([start])
    parent = {start: None}
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(sphere_points[:, 0], sphere_points[:, 1],
               sphere_points[:, 2], c='b', marker='.', alpha=0.1)
    ax.scatter(start[0], start[1], start[2], c='r',
               marker='o', s=100, label='Start')
    ax.scatter(goal[0], goal[1], goal[2], c='g',
               marker='o', s=100, label='Goal')
    ax.scatter(0, 0, 0, c='k', marker='x', s=100, label='Origin')
    for poly in polygons + polygons2 + polygons3 + polygons4:
        poly.set_color('k')
        poly.set_alpha(0.5)
        ax.add_collection3d(poly)
        plane_polygon.set_alpha(0.1)
    ax.add_collection3d(plane_polygon)
    plt.ion()
    plt.show()
    for i in range(MAX_ITERATIONS):
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
        ax.plot([nearest_node[0], new_node[0]], [nearest_node[1], new_node[1]], [
                nearest_node[2], new_node[2]], c='b', linewidth=1)
        plt.pause(0.001)
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
        ax.scatter(path_x, path_y, path_z, c='r', marker='o', s=20)

        pruned_path = prune_path(final_path)
        pruned_path_x, pruned_path_y, pruned_path_z = zip(*pruned_path)
        ax.scatter(pruned_path_x, pruned_path_y, pruned_path_z,
                   c='g', marker='o', s=40, label='Pruned Path')

        ax.legend()
        plt.ioff()
        plt.show()
        return final_path

    ax.legend()
    plt.ioff()
    plt.show()
    return None


polygons = [Poly3DCollection([vertices[face]]) for face in faces]
polygons2 = [Poly3DCollection([vertices2[face]]) for face in faces2]
polygons3 = [Poly3DCollection([vertices3[face]]) for face in faces3]
polygons4 = [Poly3DCollection([vertices4[face]]) for face in faces4]
plane_polygon = Poly3DCollection([plane_vertices[plane_faces[0]]])

path = rrt(START, GOAL)

if path is not None:
    print("Path found!")
    # print("Original path:")
    for node in path:
        # print(node)
        pass

    pruned_path = prune_path(path)
    print("\nPruned path:")
    for node in pruned_path:
        print(node)
else:
    print("Path not found.")
