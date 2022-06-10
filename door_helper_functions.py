import ast
import math
import numpy
import rospy

from lowlevel_skills.manipulation.calibrate import soft_calibrate, soft_calibrate_once

from myp_ros.srv import Generic_myP_Service


def soft_move_to_pose(pose="gait_1", slow=False):
    """
    Move to pose using soft mode.

    :param pose: pose name or pose angles
    :type pose: str or list
    :param slow: go slow to the pose
    :type slow: bool
    """
    if type(pose).__name__ == "str":
        pose_angles = get_pose_angles(pose)
    else:
        pose_angles = pose

    if slow:
        gain_factor = 0.75
        velocity_factor = 0.1
        velocity = 10
    else:
        gain_factor = 1.5
        velocity_factor = 0.3
        velocity = 25

    soft_calibrate_once()
    soft_move_joint([2, 3, 5], [pose_angles[1], pose_angles[2], pose_angles[4]], gain_factor=gain_factor*2, velocity_factor=velocity_factor, block=True)
    soft_move_joint(ALL_KIN_JOINTS, pose_angles, gain_factor=gain_factor, velocity_factor=velocity_factor, block=True)
    move_joint(ALL_KIN_JOINTS, pose_angles, velocity=velocity, block=True, relative=False)
    soft_calibrate_once()


def release_handle():
    """
    Release a handle and retreat from it.
    """
    # soften and reset the reference
    soft_calibrate()

    # release the handle
    open_gripper()

    # retreat from the handle
    retreat = 150
    try:
        move_tool(-retreat, 0, 0, orientation=[0, 0, 0], velocity=10, block=True, relative=True, frame="tool")
    except KinematicsError:
        # try another approach if there is no kinematic solution
        pose_angles = get_pose_angles("gait_1")
        soft_move_joint(1, pose_angles[0], gain_factor=0.75, velocity_factor=0.1, block=True)
    wait(1)

    # return to soft mode
    soft_calibrate_once()


def joint_1_sign():
    """
    Returns the sign of joint 1.

    :returns: sign of joint 1 angle
    :rtype: int
    """
    if read_actuator_position(1) >= 0:
        return 1
    else:
        return -1


def in_limit(value, limit):
    """
    Quantizer -limit, 0, limit.

    :param value: input value
    :type value: float
    :param limit: limit
    :type limit: float
    :returns: quantized value
    :rtype: float
    """
    if value >= limit:
        return limit
    elif value <= -limit:
        return -limit
    else:
        return 0


def detect_door(door_width, distance, side, opening_direction, previous_angle, debug=False):
    """
    Calls ROS service for detecting door and calculating door angle.

    :param door_width: door width [m]
    :type door_width: float
    :param distance: distance from the door [m]
    :type distance: float
    :param side: 1 if the robot needs to pull the door
                -1 if the robot needs to push the door
    :type side: int
    :param opening_direction: 1 if the door hinges are on the left while pushing and on the right while pulling
                             -1 if the door hinges are on the right while pushing and on the left while pulling
    :type opening_direction: int
    :param previous_angle: previous door angle [deg]
    :type previous_angle: float
    :param debug: print messages for debugging
    :type debug: bool
    :returns: success,
              door hinge coordinates in lidar frame [x [m], y [m]],
              door handle coordinates in lidar frame [x [m], y [m]],
              door angle from the door frame [deg]
    :rtype: bool, list of floats, list of floats, float
    """
    try:
        srv = rospy.ServiceProxy("get_door_segment", Generic_myP_Service)
        # {"door_parameters": [door_width, door_distance, side, opening_direction], "fov": 180}
        response = srv(str({"door_parameters": [door_width, distance, side, opening_direction], "previous_angle": previous_angle}))
        if response.error == "":
            response = ast.literal_eval(response.output)
            return True, response["door_hinge"], response["door_handle"], response["door_angle"]
        else:
            if debug:
                print(response.error)
    except rospy.ServiceException as e:
        if debug:
            print(e)
    return False, [0.0, 0.0], [0.0, 0.0], 0.0


def detect_door_frame(threshold=2.0, debug=False):
    """
    Calls ROS service for detecting opened door frame.

    :param threshold: maximal lidar range [m]
    :type threshold: float
    :param debug: print messages for debugging
    :type debug: bool
    :returns: success, start and end point of the door frame segment in polar coordinates [theta, r] in lidar frame
    :rtype: bool, list of floats, list of floats
    """
    try:
        srv = rospy.ServiceProxy("get_door_frame_segment", Generic_myP_Service)
        # {"fov": 180, "scan_threashold": 2.0}
        response = srv(str({"fov": 180, "scan_threshold": threshold}))
        if response.error == "":
            response = ast.literal_eval(response.output)
            response["door_frame_segment"][0][0] = response["door_frame_segment"][0][0] * 180 / math.pi
            response["door_frame_segment"][1][0] = response["door_frame_segment"][1][0] * 180 / math.pi
            return True, response["door_frame_segment"][0], response["door_frame_segment"][1]
        else:
            if debug:
                print(response.error)
    except rospy.ServiceException as e:
        if debug:
            print(e)
    return False, [0.0, 0.0], [0.0, 0.0]


def get_wall_angle(search_range=None, debug=False):
    """
    Calls ROS service for calculating wall angle.

    :param search_range: expected range of the wall angle [deg, deg]
    :type search_range: list
    :param debug: print messages for debugging
    :type debug: bool
    :returns: success, wall angle [deg]
    :rtype: bool, float
    """
    if search_range is None:
        search_range = [60.0, 120.0]

    try:
        srv = rospy.ServiceProxy("get_wall_angle", Generic_myP_Service)
        response = srv(str({"fov": 360, "search_range": search_range}))
        if response.error == "":
            response = ast.literal_eval(response.output)
            return True, response["wall_angle"]
        else:
            if debug:
                print(response.error)
    except rospy.ServiceException as e:
        if debug:
            print(e)
    return False, 0.0


def find_closest_x(y_band, debug=False):
    """
    Calls ROS service for finding the closest scan point to the robot along x axis.

    :param y_band: deviation from x axis [m, m]
    :type y_band: [float, float]
    :param debug: print messages for debugging
    :type debug: bool
    :returns: success, x distance to the point [m]
    :rtype: bool, float
    """
    x_points = list()
    max_iterations = 5
    for i in range(max_iterations):
        try:
            srv = rospy.ServiceProxy("get_closest_point_in_band", Generic_myP_Service)
            response = srv(str({"axis_side": [1, 1], "y_band": y_band}))
            if response.error == "":
                response = ast.literal_eval(response.output)
                x_points.append(response["min_x"])
            else:
                if debug:
                    print(response.error)
        except rospy.ServiceException as e:
            if debug:
                print(e)

    if len(x_points) > 0:
        return True, numpy.median(x_points)
    else:
        return False, 0.0

