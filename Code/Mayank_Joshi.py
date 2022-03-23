import os
import cv2
import math
import time
import argparse
import numpy as np
from queue import PriorityQueue


import utils as ut
import map_generator as map_gen
import animate_searching as anime


MIN_STEP_SIZE = 0
MAX_STEP_SIZE = 10


class Node:
    def __init__(self, node_state, parent_node=None, cost_to_come=0, childs=None):
        if childs is None:
            childs = []
        self.node_state = node_state
        self.parent_node = parent_node
        self.cost_to_come = cost_to_come
        self.childs = childs


def handle_theta_edge(curr_theta, angle):
    final_angle = curr_theta + angle
    if final_angle < 0:
        final_angle += 360
    if final_angle >= 360:
        final_angle -= 360

    return final_angle


def move_action_subfunction(curr_node, step_size, angle, end_location):
    # calculate the next node cost to come with respect to current node
    next_cost = step_size + curr_node.cost_to_come

    curr_theta = curr_node.node_state[2]
    next_theta = handle_theta_edge(curr_theta, angle)
    next_x = curr_node.node_state[0] + step_size * np.cos(np.radians(next_theta))
    next_y = curr_node.node_state[1] + step_size * np.sin(np.radians(next_theta))

    # create a new Node object
    next_node = Node((next_x, next_y, next_theta), curr_node, next_cost)

    return next_node


def get_next_nodes(curr_node, world_map, clearance, step_size, end_location):

    # initialize an empty list to store the next possible nodes
    next_nodes = []

    # move straight
    node1 = move_action_subfunction(curr_node, step_size, 0, end_location)
    if not ut.inside_obstacle_space(node1.node_state[0], node1.node_state[1], world_map, clearance):
        next_nodes.append(node1)

    # rotate 30 degrees clockwise and move
    node2 = move_action_subfunction(curr_node, step_size, -30, end_location)
    if not ut.inside_obstacle_space(node2.node_state[0], node2.node_state[1], world_map, clearance):
        next_nodes.append(node2)

    # rotate 30 degrees anticlockwise and move
    node3 = move_action_subfunction(curr_node, step_size, 30, end_location)
    if not ut.inside_obstacle_space(node3.node_state[0], node3.node_state[1], world_map, clearance):
        next_nodes.append(node3)

    # rotate 60 degrees clockwise and move
    node4 = move_action_subfunction(curr_node, step_size, -60, end_location)
    if not ut.inside_obstacle_space(node4.node_state[0], node4.node_state[1], world_map, clearance):
        next_nodes.append(node4)

    # rotate 60 degrees anticlockwise and move
    node5 = move_action_subfunction(curr_node, step_size, 60, end_location)
    if not ut.inside_obstacle_space(node5.node_state[0], node5.node_state[1], world_map, clearance):
        next_nodes.append(node5)

    # return the list containing next possible nodes with respect to the current node
    return next_nodes


def calculate_euclidean_distance(p1, p2):
    return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def reached_goal_node(current_location, end_location, goal_threshold):

    # check if current node location is equal to end location
    if calculate_euclidean_distance(current_location, end_location) <= goal_threshold:
        print("REACHED")
        if current_location[2] == end_location[2]:
            print("[INFO]: Reached Goal Node with Correct Orientation")
        return True
    return False


def round_to_half(number):
    return round(number * 2.0) / 2.0


def visited_node(visited_nodes, node, end_location):
    loc = int(round_to_half(node.node_state[1]) * 2), \
          int(round_to_half(node.node_state[0]) * 2), \
          int(round_to_half(node.node_state[2]) // 30)
    if visited_nodes[loc] > node.cost_to_come + calculate_euclidean_distance(node.node_state, end_location):
        visited_nodes[loc] = node.cost_to_come + calculate_euclidean_distance(node.node_state, end_location)
        return False

    return True


def start_a_star(world_map, start_location, end_location, clearance, step_size, goal_threshold, robot_radius):

    # create initial node object and set it to start location and parent Node to None
    inital_node = Node(start_location, None, 0)

    branches = []

    h, w = world_map.shape[:2]

    # create visited matrix to store the explored nodes
    visited_nodes = np.ones((h * 2, w * 2, 360 // 30), np.int32) * math.inf
    # visited_nodes = np.ones((h, w, 360 // 30), np.int32) * math.inf

    # create a priority queue object
    pq = PriorityQueue()

    pq.put([inital_node.cost_to_come, inital_node.node_state, inital_node])

    start = time.time()

    # iterate through open list dictionary till its size is not 0
    while not pq.empty():

        # pop a new node with lowest Cost from the priority queue
        cur_node = pq.get()[2]

        branches.append(cur_node)

        # check if current node is the goal node
        if reached_goal_node(cur_node.node_state, end_location, goal_threshold):

            frame_skip = 100
            total_time = time.time() - start
            print("Solved in {} seconds".format(total_time))
            if total_time < 1:
                frame_skip = 1

            # start visualization of node exploration
            world_map = anime.visualize_exploration(world_map, start_location, end_location, branches, robot_radius, frame_skip)

            #  start backtracking to the start location from end location
            anime.backtrack_to_start(world_map, cur_node, start_location, end_location, robot_radius)

            return True

        # generate new nodes from the current node
        next_nodes = get_next_nodes(cur_node, world_map, clearance, step_size, end_location)

        # iterate through the next possible nodes from the current node
        for node in next_nodes:

                loc = int(round_to_half(node.node_state[1]) * 2), \
                      int(round_to_half(node.node_state[0]) * 2), \
                      node.node_state[2] // 30
                if visited_nodes[loc] > node.cost_to_come + calculate_euclidean_distance(node.node_state, end_location):

                    cur_node.childs.append(node)

                    visited_nodes[loc] = node.cost_to_come + calculate_euclidean_distance(node.node_state, end_location)

                    pq.put([node.cost_to_come + calculate_euclidean_distance(node.node_state, end_location), node.node_state, node])

    print("NO SOLUTION FOUND")
    return False


def solve():

    parser = argparse.ArgumentParser()
    parser.add_argument('--init', default=None, type=str, help='start location of the robot. Example - 10,12,0'
                                                               ' [NOTE - without spaces], Default: Random')
    parser.add_argument('--goal', default=None, type=str, help='end location of the robot. Example - 150,120,30'
                                                               ' [NOTE - without spaces], Default: Random')
    parser.add_argument('--robotRadius', default=5, type=int, help='robot radius , DEFAULT: 5')
    parser.add_argument('--clearance', default=10, type=int, help='clearance dist from obstacles, DEFAULT: 5')
    parser.add_argument('--stepSize', default=10, type=int, help='step size of movement in units, between 1 and 10, DEFAULT: 10')
    parser.add_argument('--goalThresh', default=5, type=float, help='goal threshold distance, DEFAULT: 5')

    args = parser.parse_args()

    goal_state = args.goal
    initial_state = args.init
    robot_radius = int(args.robotRadius)
    clearance = int(args.clearance)
    step_size = int(args.stepSize)
    goal_thresh = float(args.goalThresh)

    # step size should be between 0 and 10
    step_size = max(MIN_STEP_SIZE, min(step_size, MAX_STEP_SIZE))

    if clearance > 50:
        print("[WARN]: Clearance value cannot be greater than 50, setting back to 5")
        clearance = 5

    print("[INFO]: Step Size set to ", step_size)
    print("[INFO]: Clearance from obstacles set to ", clearance)
    print("[INFO]: Robot Radius set to ", robot_radius)
    print("[INFO]: Total Clearance is sum of Robot Radius and Clearance ", robot_radius + clearance)

    map_file = "./final_map.png"

    if not os.path.exists(map_file):
        print("[ERROR]: MAP File Does Not Exists: {}".format(map_file))
        print("Do you want to generate a new world map? Press 1 for YES, 0 for NO")
        create_map = 1
        display_progress = 0
        try:
            create_map = int(input())
        except Exception as e:
            print("[ERROR]: Incorrect input, Try Again. Exiting!!!")
            exit()
        if create_map == 1:
            try:
                print("Do you want to see progress as the map is generated, Press 1 for YES, 0 for NO")
                display_progress = int(input())
            except Exception as e:
                print("[ERROR]: Incorrect input, Try Again. Exiting!!!")
                exit()
            map_gen.start_map_generation(display_progress)
        else:
            print("[INFO]: No Map Found, Exiting!!!")
            exit()

    world_map = cv2.imread(map_file, 0)

    if initial_state is None:
        print("[WARN]: User did not specify any initial location, selecting RANDOMLY")
        start_location = ut.get_random_location(world_map, robot_radius + clearance)
    else:
        try:
            start_location = tuple(map(int, initial_state.split(',')))
            if ut.inside_obstacle_space(start_location[0], start_location[1], world_map, robot_radius + clearance):
                print("[ERROR]: Start Location is Inside Obstacle Space, Please Try Again")
                exit()
        except Exception as e:
            print("[ERROR]: Incorrect Initial Location input, Try Again. Exiting!!!")
            exit()

    if goal_state is None:
        print("[WARN]: User did not specify any goal location, selecting RANDOMLY")
        end_location = ut.get_random_location(world_map, clearance + robot_radius)
    else:
        try:
            end_location = tuple(map(int, goal_state.split(',')))
            if ut.inside_obstacle_space(end_location[0], end_location[1], world_map, robot_radius + clearance):
                print("[ERROR]: End Location is Inside Obstacle Space, Please Try Again")
                exit()
        except Exception as e:
            print("[ERROR]: Incorrect End Location input, Try Again. Exiting!!!")
            exit()

    color_world_map = cv2.cvtColor(world_map, cv2.COLOR_GRAY2BGR)
    print("[INFO]: BLUE Circle is the initial location: ", start_location)
    cv2.circle(color_world_map, (start_location[0], start_location[1]), robot_radius, (255, 0, 0), -1)

    print("[INFO]: RED Circle is the goal location: ", end_location)
    cv2.circle(color_world_map, (end_location[0], end_location[1]), robot_radius, (0, 0, 255), -1)

    cv2.namedWindow("start_and_end_location", cv2.WINDOW_NORMAL)
    cv2.moveWindow("start_and_end_location", 10, 10)
    cv2.resizeWindow("start_and_end_location", 800, 500)
    cv2.imshow('start_and_end_location', color_world_map)
    cv2.waitKey(0)

    print("[INFO]: Started Solving ...")
    success = start_a_star(world_map, start_location, end_location, clearance + robot_radius, step_size, goal_thresh, robot_radius)
    print("SUCCESS: ", success)


if __name__ == "__main__":
    solve()