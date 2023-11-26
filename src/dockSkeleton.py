# ROS packages
import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration

# Create3 packages
import irobot_create_msgs
from irobot_create_msgs.action import (
    AudioNoteSequence,
    DriveDistance,
    IrOpcode,
    RotateAngle,
    Undock,
)
from irobot_create_msgs.msg import AudioNote, AudioNoteVector, DockStatus

# Python packages
import os
import time

from toBool import to_bool
from randomAngle import random_angle

OPCODES = {
    "FORCE_FIELD": 161,
    "VIRTUAL_WALL": 162,
    "BUOY_GREEN": 164,
    "BUOY_RED": 168,
    "BUOY_BOTH": 172,
    "EVAC_GREEN_FIELD": 244,
    "EVAC_RED_FIELD": 248,
    "EVAC_BOTH_FIELD": 252,
    "SENSOR_OMNI": 0,
    "SENSOR_DIRECTIONAL_FRONT": 1,
}

HISTORY_SIZE = 3


class Roomba(Node):
    def __init__(self, ns):
        super().__init__("robot")

        self._is_docked = None
        self._is_dock_visible = None
        self._busy = False
        self._last_ir_opcode = None

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
            HISTORY_SIZE,
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
        self.timer = self.create_timer(10, self.patrol)  # seconds

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
            append=False,
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
        # TODO may
        self.drive()
        print("Mission Complete. Returning to dock...")
        self.transition_notes()
        self.find_dock()
        print("Dock Found. Docking...")
        self.dock()
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

    def drive(self):
        """
        This function will drive the robot around
        1. Undock
        2. Drive forward
        3. Rotate in a random direction
        4. Drive forward
        """
        print("Driving")
        # Read current dock status
        print(f"dock_status: {self.is_docked}")
        i = 0
        while self.is_docked is None and i < 3:
            print("docking dependency not ready, waiting...")
            print(f"dock_status: {self.is_docked}")
            time.sleep(2)
            i += 1

        if to_bool(self.is_docked):
            print("Undocking...")
            complete = self.undock()
            print(complete)
            # Drive
            self.drive_ac.wait_for_server()
            drive_goal = DriveDistance.Goal()
            drive_goal.distance = 0.66  # meter
            self.drive_ac.send_goal(drive_goal)

            print("Initial undocking maneuver complete")
            self.rotate_ac.wait_for_server()
            # Rotate in a random direction between -pi and pi
            angle = random.uniform(-math.pi, math.pi)
            rotate_goal = RotateAngle.Goal()
            rotate_goal.angle = angle
            print(f"Rotating {angle} radians")
            self.rotate_ac.send_goal(rotate_goal)
            print("Rotation Complete")
            # Drive
            self.drive_ac.wait_for_server()
            drive_goal = DriveDistance.Goal()
            drive_goal.distance = 0.5

    def find_dock(self):
        """
        Drive the robot until it finds the dock
        """
        print("Finding Dock")
        # return early if we are already docked
        if to_bool(self.is_dock_visible):
            print("Dock is visible, aligning...")
            return

        # Rotate 360 degrees in place
        self.drive_ac.wait_for_server()
        raise NotImplementedError("TODO: Implement find_dock")

    def dock(self):
        """
        Dock the robot
        """
        print("Docking")
        # return early if we are already docked
        if to_bool(self.is_docked):
            print("Already docked, skipping...")
            return

        # Drive
        self.drive_ac.wait_for_server()
        drive_goal = DriveDistance.Goal()
        drive_goal.distance = 0.2


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
