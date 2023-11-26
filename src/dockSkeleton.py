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
from irobot_create_msgs.action import AudioNoteSequence, DriveDistance, Undock
from irobot_create_msgs.msg import AudioNote, AudioNoteVector, DockStatus

# Python packages
import os
import time

from toBool import to_bool


class Roomba(Node):
    def __init__(self, ns):
        super().__init__("robot")

        self._is_docked = None
        self._is_dock_visible = None
        self._busy = False

        # Callback Groups
        cb_subscriptions = MutuallyExclusiveCallbackGroup()
        cb_actions = MutuallyExclusiveCallbackGroup()

        # Subscriptions
        self.subscription = self.create_subscription(
            DockStatus,
            "/check_dock_status",
            self.listener_callback,
            10,
            callback_group=cb_subscriptions,
        )

        # Actions
        self.undock_ac = ActionClient(
            self, Undock, f"/{ns}/undock", callback_group=cb_actions
        )
        self.drive_ac = ActionClient(
            self,
            DriveDistance,
            f"/{ns}/drive_distance",
            callback_group=cb_actions,
        )
        self.audio_sequence_ac = ActionClient(
            self,
            AudioNoteSequence,
            f"/{ns}/audio_note_sequence",
            callback_group=cb_actions,
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

    def listener_callback(self, msg):
        """
        This function will run when the /check_dock_status topic receives a message
        """
        self.is_docked = msg.is_docked
        self.is_dock_visible = msg.dock_visible

    def transition_notes(self):
        """
        Play a few notes to indicate a transition
        """
        print("Playing transition notes")
        self.audio_sequence_ac.wait_for_server()
        duration = Duration(sec=1, nanosec=0)
        audio_goal = AudioNoteSequence.Goal()
        audio_goal.iterations = 1
        audio_goal.note_sequence = AudioNoteVector(
            header=Header(frame_id="notes"),
            notes=[
                AudioNote(frequency=100, max_runtime=duration),
                AudioNote(frequency=50, max_runtime=duration),
            ],
            append=False,
        )
        self.audio_sequence_ac.send_goal(audio_goal)

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
        #TODO may
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
        print("Undocking")
        self.undock_ac.wait_for_server()  # Wait till its ready
        undock_goal = Undock.Goal()  # Make goal
        self.undock_ac.send_goal(undock_goal)  # Send goal blocking

    def drive(self):
        """
        Undock and Drive the robot forward half a meter
        """
        print("Driving")
        # Read current dock status
        print(f"dock_status: {self.is_docked}")
        i = 0
        while self.is_docked is None or i > 3:
            print("docking dependency not ready, waiting...")
            print(f"dock_status: {self.is_docked}")
            time.sleep(2)
            i += 1

        if to_bool(self.is_docked):
            print("Undocking...")
            self.undock()
            print("Undocking complete...")
            # Drive
            self.drive_ac.wait_for_server()
            drive_goal = DriveDistance.Goal()
            drive_goal.distance = 0.2  # meter
            self.drive_ac.send_goal(drive_goal)

    def find_dock(self):
        """
        Drive the robot until it finds the dock
        """
        print("Finding Dock")
        # return early if we are already docked
        if to_bool(self.is_dock_visible):
            print("Dock is visible, docking...")
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
