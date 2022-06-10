from enum import Enum
import math

class Side(Enum):
    FRONT = 1
    BACK = -1


class DoorGeometry:
    """
    Class for door geometry representation.
    """
    def __init__(self, door_parameters):
        """
        Initialize a DoorGeometry instance.

        :param door_parameters: dictionary of door parameters:
                                    type: door_name
                                    opening_direction: -1
                                    handle: {height, axis_offset, depth, length, angle}
                                    aruco: {-1: {id, size, height, offset}, 1: {id, size, height, offset}}
        :type door_parameters: dict
        """
        if not ("opening_direction" in door_parameters.keys()):
            raise_error("Parameter opening_direction is missing in the door configuration!")

        if not ("handle" in door_parameters.keys()):
            raise_error("Parameter handle is missing in the door configuration!")
        else:
            if not ("height" in door_parameters["handle"].keys()):
                raise_error("Parameter height is missing in the door handle configuration!")
            if not ("axis_offset" in door_parameters["handle"].keys()):
                raise_error("Parameter axis_offset is missing in the door handle configuration!")
            if not ("depth" in door_parameters["handle"].keys()):
                raise_error("Parameter depth is missing in the door handle configuration!")
            if not ("length" in door_parameters["handle"].keys()):
                raise_error("Parameter length is missing in the door handle configuration!")
            if not ("angle" in door_parameters["handle"].keys()):
                raise_error("Parameter angle is missing in the door handle configuration!")

        if not ("aruco" in door_parameters.keys()):
            raise_error("Parameter aruco is missing in the door configuration!")
        else:
            if not (-1 in door_parameters["aruco"].keys()):
                raise_error("Parameter aruco/-1 is missing in the door configuration!")
            if not (1 in door_parameters["aruco"].keys()):
                raise_error("Parameter aruco/1 is missing in the door configuration!")

        self.dc__ = door_parameters

        # static tfs
        self.bottom_tf_handle_joint = None
        self.handle_joint_tf_handle = None
        self.handle_joint_tf_handle_pressed = None
        self.bottom_tf_aruco = None
        self.bottom_tf_handle = None
        self.aruco_tf_handle = None

        # dynamic tfs
        self.base_tf_door_hinge = None
        self.base_tf_door_bottom = None
        self.base_tf_door_handle_joint = None
        self.base_tf_door_handle = None
        self.base_tf_door_handle_pressed = None

        self.angle = 0.0
        self.side = None
        self.half_open = False
        self.pressed_handle_offset = 0.0

        # handle parameters
        self.handle = self.dc__["handle"]
        self.opening_direction = self.dc__["opening_direction"]
        self.aruco = self.dc__["aruco"]

        # door width in meters
        self.door_width = self.handle["axis_offset"] / 1000

        # distance from the door frame used for door detection
        # update this variable each time the platform is moved
        # initial value is usual distance from the door when Lio comes there with navigation
        self.door_distance = 1.0

    # TF COMPUTATIONS
    def compute_static_tfs__(self):
        """
        Computes all static door-specific transforms.
        """
        # transforms of "front" and "back" are symmetric for "right-turning" and "left-turning" doors (opening direction)
        # symmetry multiplier
        sm = self.side.value * self.opening_direction

        self.bottom_tf_handle_joint = service_robot.create_transform(
            [self.opening_direction * self.dc__["handle"]["axis_offset"], 0, self.dc__["handle"]["height"]],
            service_robot.create_orientation(orientation=[-1 * self.side.value * 90, 0, 180*(1-self.side.value)/2], type="euler"))
        
        self.handle_joint_tf_handle = service_robot.create_transform([-1 * sm * self.dc__["handle"]["length"], 0, self.dc__["handle"]["depth"]])

        handle_press_tf = service_robot.create_transform(None, service_robot.create_orientation(orientation=[0, 0, -1 * sm * self.handle["angle"]], type="euler"))
        self.handle_joint_tf_handle_pressed = self.handle_joint_tf_handle.move_by(handle_press_tf, reference_frame=service_robot.create_transform())

        self.pressed_handle_offset = self.handle_joint_tf_handle_pressed.translation.y

        self.bottom_tf_handle = self.bottom_tf_handle_joint.change_reference(self.handle_joint_tf_handle)

        # set aruco either absolute or relative to handle joint
        if "offset" in self.dc__["aruco"][self.side.value] and "height" in self.dc__["aruco"][self.side.value]:
            self.bottom_tf_aruco = service_robot.create_transform(
                [self.opening_direction * self.dc__["aruco"][self.side.value]["offset"],
                 0,
                 self.dc__["aruco"][self.side.value]["height"]],
                service_robot.create_orientation(orientation=[-1*self.side.value*90, 0, 180 * (1 + self.side.value)/2],
                                                 unit="deg", type="euler"))

            self.aruco_tf_handle = self.bottom_tf_aruco.inverse().change_reference(self.bottom_tf_handle)

        elif "offset_to_handle_joint" in self.dc__["aruco"][self.side.value] and\
                "height_to_handle_joint" in self.dc__["aruco"][self.side.value]:
            self.aruco_tf_handle_joint = service_robot.create_transform(
                [-sm * self.dc__["aruco"][self.side.value]["offset_to_handle_joint"],
                 self.dc__["aruco"][self.side.value]["height_to_handle_joint"],
                 0],
                service_robot.create_orientation(orientation=[0, 0, 180], unit="deg", type="euler")
            )
            self.aruco_tf_handle = self.aruco_tf_handle_joint.change_reference(self.handle_joint_tf_handle)
            self.bottom_tf_aruco = self.bottom_tf_handle_joint.change_reference(self.aruco_tf_handle_joint.inverse())
        else:
            raise_error("The aruco is not configured with the required parameters 'offset' and 'height' or " +\
                        "'offset_fromn_handle_joint' and 'height_from_handle_joint'.")

    def compute_dynamic_tfs__(self):
        """
        Computes all transforms that depend on the location of the door bottom.
        """
        # compute all door transforms in the robot base frame
        self.base_tf_door_handle_joint = self.base_tf_door_bottom.change_reference(self.bottom_tf_handle_joint)
        self.base_tf_door_handle = self.base_tf_door_handle_joint.change_reference(self.handle_joint_tf_handle)
        self.base_tf_door_handle_pressed = self.base_tf_door_handle_joint.change_reference(self.handle_joint_tf_handle_pressed)

    def set_bottom_from_aruco__(self, base_tf_aruco, door_angle=0):
        """
        Sets the door "into space" by computing the base_tf_door_bottom from the aruco detection.

        :param base_tf_aruco: transform from the base to the aruco code on the door
        :type base_tf_aruco: Transform
        :param door_angle: current door angle
        :type door_angle: float
        """
        if door_angle != 0:
            raise_error("Location of non-closed door based on aruco not implemented yet!")

        # orientation of the base_tf_door_bottom represents door angle
        self.base_tf_door_bottom = base_tf_aruco.change_reference(self.bottom_tf_aruco.inverse())

    def set_bottom_from_base_travel__(self, base_initial_tf_base):
        """
        Sets the current base_tf_door_bottom based on the travelled distance since the initial localisation of the door.

        :param base_initial_tf_base: transform from initial base pose to the current base pose
        :type base_initial_tf_base: Transform
        """
        self.base_tf_door_bottom = self.base_tf_door_hinge.move_by(base_initial_tf_base.inverse())

    def locate_with_door_hinge(self, base_tf_door_hinge, side):

        # compute set static tfs
        self.side = Side(side)
        self.compute_static_tfs__()
        self.base_tf_door_hinge = base_tf_door_hinge
        self.base_tf_door_bottom = service_robot.create_transform(base_tf_door_hinge)   # implement nice copy?

        # compute dynamic tfs
        self.compute_dynamic_tfs__()

    def locate_with_aruco(self, base_tf_aruco, side=None, aruco_marker_id=None):
        """
        Computes and sets all useful transforms on Door for the currently observed Side in the following order:
        - sets the side door has been observed from
        - computes static tfs
        - sets door bottom from aruco
        - sets initial bottom from aruco
        - computes dynamic tfs

        :param base_tf_aruco: transform in base frame of aruco marker
        :type base_tf_aruco: Transform
        :param side: side of the door
        :type side: int
        :param aruco_marker_id: aruco marker id
        :type aruco_marker_id: int
        """
        # resolve side
        if side is None and aruco_marker_id is None:
            raise_error("The side of the door or aruco marker ID have to be provided to locate a door")

        if side is None:
            for direction in {-1, 1}:
                if self.aruco.id == aruco_marker_id:
                    side = direction
                    break
            raise_error("The door is not configured with an Aruco marker ID " + str(aruco_marker_id))
        
        self.side = Side(side)
        self.compute_static_tfs__()
        self.set_bottom_from_aruco__(base_tf_aruco)

        # set the initial door position
        self.base_tf_door_hinge = service_robot.create_transform(self.base_tf_door_bottom.to_posture())
        
        self.compute_dynamic_tfs__()

    def get_handle_side(self, door_side=None):
        """
        Returns 'left'/'right' string depending on where the door handle is located
        """
        sides = {1: "left", -1: "right"}
        if door_side is not None:
            side = door_side
        elif self.side is not None:
            side = self.side
        else:
            raise_error("The door side has to be specified either via property or function argument!")
        return sides[self.opening_direction * side]

    # METHODS FOR DOOR MANIPULATION
    def compute_arc_tfs(self, bottom_tf_grasp, arc_angle, angular_resolution=1., direction=None):
        """
        Computes a list of transformations along an arc around door hinge
        at door_bottom_tf_tool distanced at angular_resolution.

        :param bottom_tf_grasp: transform from the door bottom to the handle grasping position
        :type bottom_tf_grasp: Transform
        :param arc_angle: maximal arc angle [deg]
        :type arc_angle: float
        :param angular_resolution: arc angle step [deg]
        :type angular_resolution: float
        :param direction: door direction
        :type direction: int
        :returns: pose transformations
        :rtype: list of Transforms
        """
        if direction is None:
            direction = self.opening_direction * math.copysign(1, arc_angle)

        # change to base frame
        base_tf_grasp = self.base_tf_door_bottom.change_reference(bottom_tf_grasp)

        increment_tf = service_robot.create_transform(None, [0, 0, direction * angular_resolution], units=["deg"], type="euler")

        # compute the tfs
        arc_tfs = [base_tf_grasp]
        for i in range(int(abs(arc_angle/angular_resolution))):
            pose = arc_tfs[-1].move_by(increment_tf, reference_frame=self.base_tf_door_bottom)
            arc_tfs.append(service_robot.create_transform(pose))

        return arc_tfs

    def update_angle_with_tool(self, base_tf_tool, base_initial_tf_base, handle_tf_grasp=None):
        """
        Updates the door angle based on where platform has moved and where the tool frame is located.
        The assumption is that the gripper holds the door handle at the base_tf_handle;
        optionally at an offset of the grasp can be specified.

        :param base_tf_tool: transform from the tool frame to the base frame
        :type base_tf_tool: Transform
        :param base_initial_tf_base: transform from current base to the initial base frame that will be used
                                     for moving the current base frame;
                                     if the platform did not moved, set it to service_robot.create_transform()
        :type base_initial_tf_base: Transform
        :param handle_tf_grasp: transform of the grasping point in handle joint frame
        :type handle_tf_grasp: Transform
        """
        if handle_tf_grasp is None:
            # assume grasp at the door handle
            handle_joint_tf_grasp = self.handle_joint_tf_handle
        else:
            raise_error("Not implemented!")
            
        # first update the location of the door
        self.set_bottom_from_base_travel__(base_initial_tf_base)
        
        # compute the door angle based on the tool position
        # angle from grasp to door
        r = self.opening_direction * self.bottom_tf_handle_joint.translation.x
        angle_door_to_grasp = math.atan2(handle_joint_tf_grasp.translation.z, r + handle_joint_tf_grasp.translation.x) * 180 / math.pi

        # angle from door frame to grasp
        angle_grasp = math.atan2(abs(self.base_tf_door_bottom.translation.y - base_tf_tool.translation.y),
                                 abs(base_tf_tool.translation.x - self.base_tf_door_bottom.translation.x)) * 180 / math.pi

        # angle from door frame to door
        self.update_angle_with_value(angle_grasp - angle_door_to_grasp)

    def update_angle_with_value(self, value):
        """
        Sets the absolute angle at which the door is open.

        :param value: absolute door opening angle value [deg]
        :type value: float
        """
        self.angle = value

        # rotate the door bottom frame
        rotation_tf = service_robot.create_transform(translation=None,
                                                     orientation=[0, 0, self.angle * self.opening_direction],
                                                     units=None, type="euler")
        self.base_tf_door_bottom = self.base_tf_door_hinge.move_by(rotation_tf,
                                                                   reference_frame=self.base_tf_door_hinge)

        # update handle tfs
        self.compute_dynamic_tfs__()

    # CONDITIONS
    def handle_pressed_condition(self, arm_pose):
        """
        Returns True if the current arm pose is below the the computed handle_pressed transform.

        :param arm_pose: pose of the arm
        :type arm_pose: Transform
        :returns: if the condition is satisfied
        :rtype: bool
        """
        print(str(arm_pose.translation.z) + " vs. " + str(self.base_tf_door_handle_pressed.translation.z))
        if arm_pose.translation.z < self.base_tf_door_handle_pressed.translation.z:
            return True

        return False

    # MISC
    def publish(self, base_frame_name, duration=1.0):
        """
        Publishes door bottom, handle joint and handle transforms.

        :param base_frame_name: reference frame of the published transforms
        :type base_frame_name: Transform
        :param duration: duration of publishing
        :type duration: float
        """
        max_iterations = 5
        for _ in range(max_iterations):
            service_robot.publish_transform(self.base_tf_door_hinge, "door_hinge", base_frame_name)
            service_robot.publish_transform(self.base_tf_door_bottom, "door_bottom", base_frame_name)
            service_robot.publish_transform(self.base_tf_door_handle_joint, "door_handle_joint", base_frame_name)
            service_robot.publish_transform(self.base_tf_door_handle, "door_handle", base_frame_name)
            wait(duration / max_iterations)
        
   
# TEST
test_door_parameters = {"type": "standard",
                        "opening_direction": -1,
                        "handle": {"angle": 30, "axis_offset": 825, "height": 1040, "depth": 50, "length": 90},
                        "aruco": {1: {"height": 830, "offset": 695, "id": 8, "size": 6},
                                  -1: {"height": 830, "offset": 720, "id": 15, "size": 6}}}

door = DoorGeometry(test_door_parameters)
door.locate_with_aruco(service_robot.create_transform([-100, 900, 600], [90, 0, 0]), side=1)

for i in range(10):
    door.publish("LIOc_robot_base_link", duration=3)
    base_tf_tool = service_robot.create_transform([-100 + i * 80, 900 - i * 80, 600])
    base_initial_tf_base = service_robot.create_transform([0, -60 * i, 0])
    base_initial_tf_base = service_robot.create_transform([0, 0, 0])
    door.update_angle(base_tf_tool, base_initial_tf_base)
    print("angle: " + str(door.angle))

door.publish("LIOc_robot_base_link", duration=3)



