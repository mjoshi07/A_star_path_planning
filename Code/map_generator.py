import cv2
import numpy as np
import os

WIDTH = 400
HEIGHT = 250
EPSILON = 1e-7


def half_plane(empty_map, pt1, pt2, fill_right=True):
    map_copy = empty_map.copy()

    x = np.arange((map_copy.shape[1]))
    y = np.arange((map_copy.shape[0]))
    X, Y = np.meshgrid(x, y)

    slope = (pt1[1] - pt2[1]) / (pt1[0] - pt2[0] + EPSILON)
    bias = pt1[1] - slope * pt1[0]

    value = Y - slope * X - bias

    if fill_right:
        map_copy[value > 0] = 0
    else:
        map_copy[value <= 0] = 0

    return map_copy


def fill_polygon(empty_map, line_points, display_progress=False):
    line_1_points = line_points[0]
    line_2_points = line_points[1]
    line_3_points = [line_2_points[1], line_1_points[0]]

    line_4_points = [line_2_points[1], line_1_points[0]]
    line_5_points = line_points[2]
    line_6_points = line_points[3]

    half_plane_1 = half_plane(empty_map, line_1_points[0], line_1_points[1], True)

    half_plane_2 = half_plane(empty_map, line_2_points[0], line_2_points[1], False)

    half_plane_3 = half_plane(empty_map, line_3_points[0], line_3_points[1], False)

    half1 = half_plane_1 + half_plane_2 + half_plane_3

    half_plane_4 = half_plane(empty_map, line_4_points[0], line_4_points[1], True)

    half_plane_5 = half_plane(empty_map, line_5_points[0], line_5_points[1], True)

    half_plane_6 = half_plane(empty_map, line_6_points[0], line_6_points[1], False)

    half2 = half_plane_4 + half_plane_5 + half_plane_6
    print(display_progress)
    if display_progress:

        cv2.namedWindow('img', cv2.WINDOW_NORMAL)
        cv2.moveWindow('img', 10, 10)

        cv2.imshow('img', half_plane_1)
        cv2.waitKey(0)

        cv2.imshow('img', half_plane_2)
        cv2.waitKey(0)

        cv2.imshow('img', half_plane_3)
        cv2.waitKey(0)

        cv2.imshow('img', half1)
        cv2.waitKey(0)

        cv2.imshow('img', half_plane_4)
        cv2.waitKey(0)

        cv2.imshow('img', half_plane_5)
        cv2.waitKey(0)

        cv2.imshow('img', half_plane_6)
        cv2.waitKey(0)

        cv2.imshow('img', half2)
        cv2.waitKey(0)

    map_copy = cv2.bitwise_and(half2, half1)

    return map_copy


def fill_hexagon(empty_map, center, s2s, display_progress=False):
    hex_line_points = get_hex_line_points(center, s2s)

    line_1_points = hex_line_points[0]
    line_2_points = hex_line_points[1]
    line_3_points = hex_line_points[2]
    line_4_points = hex_line_points[3]
    line_5_points = hex_line_points[4]
    line_6_points = hex_line_points[5]

    half_plane_1 = half_plane(empty_map, line_1_points[0], line_1_points[1], True)

    half_plane_2 = half_plane(empty_map, line_2_points[0], line_2_points[1], False)

    half_plane_3 = half_plane(empty_map, line_3_points[0], line_3_points[1], False)

    half_plane_4 = half_plane(empty_map, line_4_points[0], line_4_points[1], False)

    half_plane_5 = half_plane(empty_map, line_5_points[0], line_5_points[1], False)

    half_plane_6 = half_plane(empty_map, line_6_points[0], line_6_points[1], True)

    half1 = half_plane_1 + half_plane_2

    half2 = half_plane_3 + half1

    half3 = half_plane_4 + half2

    half4 = half_plane_5 + half3

    if display_progress:
        cv2.namedWindow('img', cv2.WINDOW_NORMAL)
        cv2.moveWindow('img', 10, 10)
        cv2.imshow('img', half_plane_1)
        cv2.waitKey(0)
        cv2.imshow('img', half_plane_2)
        cv2.waitKey(0)
        cv2.imshow('img', half_plane_3)
        cv2.waitKey(0)
        cv2.imshow('img', half_plane_4)
        cv2.waitKey(0)
        cv2.imshow('img', half_plane_5)
        cv2.waitKey(0)
        cv2.imshow('img', half_plane_6)
        cv2.waitKey(0)
        cv2.imshow('img', half1)
        cv2.waitKey(0)
        cv2.imshow('img', half2)
        cv2.waitKey(0)
        cv2.imshow('img', half3)
        cv2.waitKey(0)
        cv2.imshow('img', half4)
        cv2.waitKey(0)

    map_copy = half_plane_6 + half4

    return map_copy


def fill_circle(empty_map, center, diameter):
    map_copy = empty_map.copy()

    x = np.arange(map_copy.shape[1])
    y = np.arange(map_copy.shape[0])

    xx, yy = np.meshgrid(x, y)

    map_copy[(xx - center[0]) ** 2 + (yy - center[1]) ** 2 - (diameter // 2) ** 2 <= 0] = 0

    return map_copy


def get_hex_line_points(center, s2s):
    """
            *
        *       *

        *       *
            *
        Order of points in our Hexagon

            1
        6       2

        5       3
            4


    edge = s2s // sqrt(3)

    Point 1 coordinates = (center_x, center_y + edge // 2 + s2s // 2 * sqrt(3))
    Point 2 coordinates = (center_x + s2s // 2, center_y + edge // 2)
    Point 3 coordinates = (center_x + s2s // 2, center_y - edge // 2)
    Point 4 coordinates = (center_x, center_y - edge // 2 - s2s // 2 * sqrt(3))
    Point 5 coordinates = (center_x - s2s // 2, center_y - edge // 2)
    Point 6 coordinates = (center_x - s2s // 2, center_y + edge // 2)

    """

    center_x, center_y = center[0], center[1]
    edge = s2s // np.sqrt(3)

    point_1 = [center_x, center_y - s2s // np.sqrt(3)]
    point_2 = [center_x + s2s // 2, center_y - edge // 2]
    point_3 = [center_x + s2s // 2, center_y + edge // 2]
    point_4 = [center_x, center_y + s2s // np.sqrt(3)]
    point_5 = [center_x - s2s // 2, center_y + edge // 2]
    point_6 = [center_x - s2s // 2, center_y - edge // 2]

    line_1_2 = [point_1, point_2]
    line_2_3 = [point_2, point_3]
    line_3_4 = [point_3, point_4]
    line_4_5 = [point_4, point_5]
    line_5_6 = [point_5, point_6]
    line_6_1 = [point_6, point_1]

    lines = [line_1_2, line_2_3, line_3_4, line_4_5, line_5_6, line_6_1]

    return lines


def fill_map(empty_map, line_points=None, center=None, s2s=None, type='circle', display_progress = False):
    if line_points is None:
        if type == 'circle':
            map_copy = fill_circle(empty_map, center, s2s)
        if type == 'hexagon':
            map_copy = fill_hexagon(empty_map, center, s2s, display_progress)
    else:
        map_copy = fill_polygon(empty_map, line_points, display_progress)

    return map_copy


def generate_map(display_progress=False):
    empty_map = np.ones((HEIGHT, WIDTH), np.uint8) * 255

    # create concave polygon
    line_1_points = [[36, 65], [115, 40]]
    line_2_points = [[115, 40], [80, 70]]
    line_3_points = [[80, 70], [105, 150]]
    line_4_points = [[105, 150], [36, 65]]

    # create hexagon
    hex_center = [200, 150]
    hex_s2s = 70

    # create circle
    circle_center = [300, 65]
    circle_diameter = 80

    map_with_polygon = fill_map(empty_map, [line_1_points, line_2_points, line_3_points, line_4_points], display_progress=display_progress)
    if display_progress:
        cv2.namedWindow('polygon', cv2.WINDOW_NORMAL)
        cv2.moveWindow('polygon', 10, 10)
        cv2.imshow('polygon', map_with_polygon)
        cv2.waitKey(0)

    map_with_hexagon = fill_map(empty_map, None, hex_center, hex_s2s, 'hexagon', display_progress)
    if display_progress:
        cv2.namedWindow('hexagon', cv2.WINDOW_NORMAL)
        cv2.moveWindow('hexagon', 10, 10)
        cv2.imshow('hexagon', map_with_hexagon)
        cv2.waitKey(0)

    map_with_circle = fill_map(empty_map, None, circle_center, circle_diameter, 'circle', display_progress)
    if display_progress:
        cv2.namedWindow('circle', cv2.WINDOW_NORMAL)
        cv2.moveWindow('circle', 10, 10)
        cv2.imshow('circle', map_with_circle)
        cv2.waitKey(0)

    final_map = cv2.bitwise_and(cv2.bitwise_and(map_with_polygon, map_with_hexagon), map_with_circle)
    if display_progress:
        cv2.namedWindow('final_map', cv2.WINDOW_NORMAL)
        cv2.moveWindow('final_map', 10, 10)
        cv2.imshow('final_map', final_map)
        cv2.waitKey(0)

    return final_map


def start_map_generation(display_progress):
    out_dir = "./"

    final_map = generate_map(display_progress)

    cv2.imwrite(os.path.join(out_dir, "final_map.png"), final_map)


if __name__ == "__main__":

    start_map_generation(display_progress=True)