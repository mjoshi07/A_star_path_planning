import cv2


def visualize_exploration(world_map, start_location, end_location, branches, robot_radius, frame_skip):

    print("[INFO]: Started Nodes Exploration Visualization from the INITIAL node to the GOAL node")

    # convert the gray scale world map to bgr for visualization
    color_world_map = cv2.cvtColor(world_map, cv2.COLOR_GRAY2BGR)

    # create a window for visualization
    cv2.namedWindow("visualization", cv2.WINDOW_NORMAL)

    # move the window to (10, 10)
    cv2.moveWindow("visualization", 10, 10)

    # resize the window
    cv2.resizeWindow("visualization", 800, 500)

    i = 0
    for visited_node in branches:

        # get all the childs of the visited node
        childs = visited_node.childs
        for child_node in childs:

            # draw the childs of the visited node
            cv2.arrowedLine(color_world_map, (int(visited_node.node_state[0]), int(visited_node.node_state[1])),
                     (int(child_node.node_state[0]), int(child_node.node_state[1])),
                     (0, 255, 0), 1)

        # draw the start location with blue colored circle
        cv2.circle(color_world_map, (start_location[0], start_location[1]), robot_radius, (255, 0, 0), -1)

        # draw the end location with red colored circle
        cv2.circle(color_world_map, (end_location[0], end_location[1]), robot_radius, (0, 0, 255), -1)

        i += 1

        if i % frame_skip == 0:
            cv2.imshow("visualization", color_world_map)
            cv2.waitKey(1)

    # return the explored nodes visualization colored world map
    return color_world_map


def backtrack_to_start(color_world_map, cur_node, start_location, end_location, robot_radius):

    print("[INFO]: Started Backtracking from the GOAL node to the INITIAL node")

    # create a window for visualization
    cv2.namedWindow("visualization", cv2.WINDOW_NORMAL)

    # move the window to (10, 10)
    cv2.moveWindow("visualization", 10, 10)

    # resize the window
    cv2.resizeWindow("visualization", 800, 500)

    # iterate through all the nodes till their parent is not None
    while cur_node.parent_node is not None:

        # draw the current node location with dark green colored circle
        # cv2.circle(color_world_map, (int(cur_node.node_state[0]), int(cur_node.node_state[1])), 3, (0, 255, 0), -1)
        cv2.line(color_world_map, (int(cur_node.node_state[0]), int(cur_node.node_state[1])), (int(cur_node.parent_node.node_state[0]), int(cur_node.parent_node.node_state[1])), (255, 0, 0), 1)

        # draw the start location with blue colored circle
        cv2.circle(color_world_map, (start_location[0], start_location[1]), robot_radius, (255, 0, 0), -1)

        # draw the end location with red colored circle
        cv2.circle(color_world_map, (end_location[0], end_location[1]), robot_radius, (0, 0, 255), -1)

        # set current node to its parent node for next iteration
        cur_node = cur_node.parent_node

        # display the backtrack in progress
        cv2.imshow("visualization", color_world_map)

        # wait for 1 millisecond and then continue to next frame
        cv2.waitKey(1)

    # after visualization is complete, let the user close the window
    cv2.waitKey(0)