# ROS packages
import math
import random
import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration

# Create3 packages
import irobot_create_msgs
from irobot_create_msgs.action import (
    AudioNoteSequence,
    DriveArc,
    DriveDistance,
    RotateAngle,
    Undock,
)
from irobot_create_msgs.msg import AudioNote, AudioNoteVector, DockStatus, IrOpcode

# Python packages
import os
import time

from toBool import to_bool
from randomAngle import random_angle

OPCODES = {
    "FORCE_FIELD": 161,
    "VIRTUAL_WALL": 162,
    "BUOY_GREEN": 164,
    "FF_GB": 165,  # Force Field and Green Buoy
    "BUOY_RED": 168,
    "FF_RB": 169,  # Force Field and Red Buoy
    "BUOY_BOTH": 172,
    "FF_Both": 173,  # Force Field and Both Buoys
    "EVAC_GREEN_FIELD": 244,
    "EVAC_RED_FIELD": 248,
    "EVAC_BOTH_FIELD": 252,
    "SENSOR_OMNI": 0,
    "SENSOR_DIRECTIONAL_FRONT": 1,
}

DISTANCES = {
    "STEP": 0.1,  # meters
    "SHORT": 0.2,
    "MEDIUM": 0.5,
    "LONG": 0.66,
    "ASTRONOMICAL": 1.0,
}

ANGLES = {
    "SIXTEENTH": math.pi / 8,  # radians
    "EIGHTH": math.pi / 4,
    "QUARTER": math.pi / 2,
    "THIRD": (2 * math.pi) / 3,
    "HALF": math.pi,
    "FULL": math.pi * 2,
}

HISTORY_SIZE = 10


class Roomba(Node):
    def __init__(self, ns):
        super().__init__("robot")

        self._is_docked = None
        self._is_dock_visible = None
        self._busy = False
        self._last_ir_opcode = None
        self._last_ir_sensor = None

        # Callback Groups
        cb_subscriptions = MutuallyExclusiveCallbackGroup()
        cb_actions = MutuallyExclusiveCallbackGroup()
        cb_notes = MutuallyExclusiveCallbackGroup()

        # Subscriptions
        # docking status
        self.subscription = self.create_subscription(
            DockStatus,
            "/check_dock_status",
            self.dock_status_callback,
            HISTORY_SIZE,
            callback_group=cb_subscriptions,
        )

        # ir opcodes
        self.subscription = self.create_subscription(
            IrOpcode,
            f"/{ns}/ir_opcode",
            self.ir_opcode_callback,
            QoSProfile(
                depth=HISTORY_SIZE, reliability=QoSReliabilityPolicy.BEST_EFFORT
            ),
            callback_group=cb_subscriptions,
        )

        # Actions
        # Undock
        self.undock_ac = ActionClient(
            self, Undock, f"/{ns}/undock", callback_group=cb_actions
        )
        # Drive
        self.drive_ac = ActionClient(
            self,
            DriveDistance,
            f"/{ns}/drive_distance",
            callback_group=cb_actions,
        )
        # Rotate
        self.rotate_ac = ActionClient(
            self,
            RotateAngle,
            f"/{ns}/rotate_angle",
            callback_group=cb_actions,
        )
        # Audio
        self.audio_sequence_ac = ActionClient(
            self,
            AudioNoteSequence,
            f"/{ns}/audio_note_sequence",
            callback_group=cb_notes,
        )

        # Timer Loop since we need a callback for drive
        self.timer = self.create_timer(3, self.patrol)  # seconds

    @property
    def is_docked(self) -> bool:
        """
        Returns the current dock status of the robot
        """
        return self._is_docked

    @is_docked.setter
    def is_docked(self, value):
        """
        Sets the current dock status of the robot
        """
        self._is_docked = value

    @property
    def is_dock_visible(self) -> bool:
        """
        Returns the current dock visibility of the robot
        """
        return self._is_dock_visible

    @is_dock_visible.setter
    def is_dock_visible(self, value):
        """
        Sets the current dock visibility of the robot
        """
        self._is_dock_visible = value

    @property
    def busy(self) -> bool:
        """
        Returns the current busy status of the robot
        """
        return self._busy

    @busy.setter
    def busy(self, value):
        """
        Sets the current busy status of the robot
        """
        self._busy = value

    @property
    def last_ir_opcode(self) -> int:
        """
        Returns the last ir opcode received by the robot
        """
        return self._last_ir_opcode

    @last_ir_opcode.setter
    def last_ir_opcode(self, value):
        """
        Sets the last ir opcode received by the robot
        """
        self._last_ir_opcode = value

    @property
    def last_ir_sensor(self) -> int:
        """
        Returns the last ir sensor received by the robot
        """
        return self._last_ir_sensor

    @last_ir_sensor.setter
    def last_ir_sensor(self, value):
        """
        Sets the last ir sensor received by the robot
        """
        self._last_ir_sensor = value

    # callbacks
    def dock_status_callback(self, msg):
        """
        This function will run when the /check_dock_status topic receives a message
        """
        self.is_docked = msg.is_docked
        self.is_dock_visible = msg.dock_visible

    def ir_opcode_callback(self, msg):
        """
        This function will run when the /ir_opcode topic receives a message
        """
        print(f"IR Opcode: {msg.opcode}")
        print(f"From Sensor: {msg.sensor}")
        self.last_ir_opcode = msg.opcode
        self.last_ir_sensor = msg.sensor

    def transition_notes(self):
        """
        Play a few notes to indicate a transition
        """
        print("Playing transition notes")
        self.audio_sequence_ac.wait_for_server()
        # half a second in nanoseconds
        half_sec = Duration(sec=0, nanosec=500000000)
        duration = Duration(sec=0, nanosec=0)
        audio_goal = AudioNoteSequence.Goal()
        audio_goal.iterations = 1
        audio_goal.note_sequence = AudioNoteVector(
            header=Header(frame_id="notes"),
            notes=[
                AudioNote(frequency=100, max_runtime=half_sec),
                AudioNote(frequency=50, max_runtime=half_sec),
            ],
            append=True,
        )
        self.audio_sequence_ac.send_goal_async(audio_goal)

    def found_dock_notes(self):
        """
        Play a few notes to indicate a transition
        """
        print("Playing found dock notes")
        self.audio_sequence_ac.wait_for_server()
        # half a second in nanoseconds
        half_sec = Duration(sec=0, nanosec=500000000)
        duration = Duration(sec=0, nanosec=0)
        audio_goal = AudioNoteSequence.Goal()
        audio_goal.iterations = 1
        audio_goal.note_sequence = AudioNoteVector(
            header=Header(frame_id="notes"),
            notes=[
                AudioNote(frequency=150, max_runtime=half_sec),
                AudioNote(frequency=100, max_runtime=half_sec),
                AudioNote(frequency=50, max_runtime=half_sec),
            ],
            append=True,
        )
        self.audio_sequence_ac.send_goal_async(audio_goal)

    def patrol(self):
        """
        This function will run every time the timer is triggered
        """
        if self.busy:
            print("Busy, skipping patrol...")
            return
        self.busy = True
        print("Patrolling...")
        self.transition_notes()
        print("Undock and Drive")
        self.begin_drive()
        print("Mission Complete. Returning to dock...")
        self.transition_notes()
        self.find_dock()
        self.transition_notes()
        print("Docking Complete. Lab Complete.")
        self.busy = False

    def undock(self):
        """
        Undock the robot
        """
        self.undock_ac.wait_for_server()  # Wait till its ready
        undock_goal = Undock.Goal()  # Make goal
        self.undock_ac.send_goal(undock_goal)  # Send goal blocking
        return "Undocking Complete"

    def begin_drive(self):
        """
        This function will drive the robot around
        1. Undock Action
        2. Drive forward
        3. Rotate in a random direction
        4. Drive forward
        """
        # Read current dock status
        print(f"dock_status: {self.is_docked}")
        while self.is_docked is None:
            print("docking dependency not ready, waiting...")
            print(f"dock_status: {self.is_docked}")
            time.sleep(2)

        if to_bool(self.is_docked):
            print("Undocking...")
            complete = self.undock()
            print(complete)
            # Drive
            self.drive_forward(DISTANCES["ASTRONOMICAL"])
            print("Initial undocking maneuver complete")

            self.turn(random_angle())

            # Drive
            self.drive_forward(DISTANCES["MEDIUM"])

    def find_dock(self):
        """
        Drive the robot until it finds the dock
        """
        print("Finding Dock")
        # loop until we are docked
        while not to_bool(self.is_docked):
            # Check if dock is visible
            if to_bool(self.is_dock_visible):
                self.hunt()
            else:
                print("Dock is not visible, rotating...")
                # keep track of how many times we rotate
                self.scout()
                time.sleep(0.25)

    def drive_forward(self, distance: float):
        """
        Drive forward a short distance
        """
        print(f"Driving Forward {distance} meters")
        self.drive_ac.wait_for_server()
        drive_goal = DriveDistance.Goal(distance=distance, max_translation_speed=0.25)
        self.drive_ac.send_goal(drive_goal)

    def drive_arc(self, radius: float, theta: float):
        """
        Drive in an arc. Radius is in meters, theta is in radians.
        """
        print(f"Driving in an arc with radius {radius} and angle {theta}")
        self.drive_ac.wait_for_server()
        forward = 1
        drive_goal = DriveArc.Goal(
            translate_direction=forward,
            radius=radius,
            angle=theta,
            max_translation_speed=0.25,
        )
        self.drive_ac.send_goal(drive_goal)

    def turn(self, theta):
        """
        Rotate the robot a certain angle
        """
        self.rotate_ac.wait_for_server()
        rotate_goal = RotateAngle.Goal(angle=theta, max_rotation_speed=1.0)
        self.rotate_ac.send_goal(rotate_goal)
        print("Rotation Complete")

    def scout(self):
        """
        Search in place for the dock and move occasionally if we don't find it.
        """
        rotations = 0
        # rotate until we see the dock
        while not to_bool(self.is_dock_visible):
            print("Rotating...")
            self.turn(ANGLES["SIXTEENTH"])
            rotations += 1
            time.sleep(0.1)
            # escape if we are stuck
            if rotations > 10:
                self.drive_forward(DISTANCES["SHORT"])
                break

    def hunt(self):
        """
        Drive the robot towards the dock once it has been found.
        """
        print("Hunting...")
        self.found_dock_notes()
        # Close Range Sensor
        if self.last_ir_sensor == OPCODES["SENSOR_OMNI"]:
            print("We are close to the dock. Checking op codes...")
            if self.last_ir_opcode == OPCODES["FF_Both"]:
                print("We see both buoys, driving forward a bit...")
                self.drive_forward(distance=DISTANCES["TINY"])

            elif self.last_ir_opcode == OPCODES["FF_GB"]:
                print(
                    "Force Field and Green Buoy. We are close and need to rotate in the positive direction..."
                )
                self.drive_arc(radius=DISTANCES["TINY"], theta=ANGLES["EIGHTH"])

            elif self.last_ir_opcode == OPCODES["FF_RB"]:
                print(
                    "Force Field and Red Buoy. We are close and need to rotate in the negative direction..."
                )
                self.drive_arc(radius=DISTANCES["TINY"], theta=ANGLES["EIGHTH"])

            elif self.last_ir_opcode == OPCODES["BUOY_GREEN"]:
                print(
                    "Green Buoy. We are far and need to rotate in the positive direction..."
                )
                self.drive_arc(radius=DISTANCES["SHORT"], theta=ANGLES["EIGHTH"])

            elif self.last_ir_opcode == OPCODES["BUOY_RED"]:
                print(
                    "Red Buoy. We are far and need to rotate in the negative direction..."
                )
                self.drive_arc(radius=DISTANCES["SHORT"], theta=ANGLES["EIGHTH"])

                # We are aligned if we see both buoys
            if self.last_ir_opcode == OPCODES["BUOY_BOTH"]:
                print("We see both buoys, driving forward a bit...")
                self.drive_forward(distance=DISTANCES["SHORT"])

        # Long Range Sensor
        elif self.last_ir_sensor == OPCODES["SENSOR_DIRECTIONAL_FRONT"]:
            print("We are far from the dock. Checking op codes...")
            # We are aligned if we see both buoys
            if self.last_ir_opcode == OPCODES["BUOY_BOTH"]:
                print("We see both buoys, driving forward a bit...")
                self.drive_forward(distance=DISTANCES["MEDIUM"])

            elif self.last_ir_opcode == OPCODES["BUOY_GREEN"]:
                print(
                    "Green Buoy. We are far and need to rotate in the positive direction..."
                )
                # EIGHTH turn since we are far from the dock.
                self.drive_arc(radius=DISTANCES["MEDIUM"], theta=ANGLES["EIGHTH"])

            elif self.last_ir_opcode == OPCODES["BUOY_RED"]:
                print(
                    "Red Buoy. We are far and need to rotate in the negative direction..."
                )
                # EIGHTH turn since we are far from the dock.
                self.drive_arc(radius=DISTANCES["MEDIUM"], theta=-ANGLES["EIGHTH"])

            elif self.last_ir_opcode == OPCODES["FORCE_FIELD"]:
                print("Force Field Detected, we are close....")
                # Big turn since we are close and don't see any buoys
                # choose one or -1 randomly since we don't know which direction we are facing
                self.drive_arc(
                    radius=DISTANCES["SHORT"],
                    theta=ANGLES["HALF"] * random.choice([1, -1]),
                )

            elif self.last_ir_opcode == OPCODES["FF_GB"]:
                print("Force Field and Green Buoy. Rotate in the positive direction...")
                # EIGHTH turn since we are far from the dock.
                self.drive_arc(
                    radius=DISTANCES["SHORT"],
                    theta=ANGLES["EIGHTH"],
                )

            elif self.last_ir_opcode == OPCODES["FF_RB"]:
                print("Force Field and Red Buoy. Rotate in the negative direction...")
                # EIGHTH turn since we are far from the dock.
                self.drive_arc(
                    radius=DISTANCES["SHORT"],
                    theta=-ANGLES["EIGHTH"],
                )


def main(args=None):
    # Globals
    rclpy.init(args=args)
    namespace = os.environ.get("BOWSER_NAME", "create3_05F8")
    roomba = Roomba(namespace)

    executor = MultiThreadedExecutor(2)
    executor.add_node(roomba)

    # Try/Except to shutdown "gracefully"
    try:
        executor.spin()
    except KeyboardInterrupt:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
