# ROS packages
import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

# Create3 packages
import irobot_create_msgs
from irobot_create_msgs.action import DriveDistance, Undock

# Your ROS Node packages
from check_sensor import DockStatusPublisher

# Garrett packages (Easy program start)
from pynput.keyboard import KeyCode
from key_commander import KeyCommander

# Python packages
from std_msgs.msg import String
import os

# Globals
rclpy.init()
namespace = os.environ.get("BOWSER_NAME", "create3_05F8")
sensor = DockStatusPublisher(namespace)


class Roomba(Node):
    def __init__(self, namespace):
        super().__init__("robot")

        # Callback Groups
        cb_Subscripions = MutuallyExclusiveCallbackGroup()
        cb_Actions = MutuallyExclusiveCallbackGroup()

        # Subscriptions
        self.subscription = self.create_subscription(
            String,
            "/check_dock_status",
            self.listener_callback,
            10,
            callback_group=cb_Subscripions,
        )

        # Actions
        self.undock_ac = ActionClient(
            self, Undock, f"/{namespace}/undock", callback_group=cb_Actions
        )
        self.drive_ac = ActionClient(
            self,
            DriveDistance,
            f"/{namespace}/drive_distance",
            callback_group=cb_Actions,
        )
        self.dock_ac = ActionClient(
            self, irobot_create_msgs.action.Dock, f"/{namespace}/dock"
        )

    def listener_callback(self, msg):
        """
        This function will run when the subscription receives a message from the publisher
        """
        print("I got: ", msg.data)

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
        Undock and Drive the robot forward 1 meter
        """
        print("Driving")
        dock_status = sensor.poll()  # Read current dock status
        print(f"dock_status: {dock_status}")
        # Undock if docked
        if dock_status == True:
            self.undock()
            print("Undocking complete...")
            # Drive
            self.drive_ac.wait_for_server()
            drive_goal = DriveDistance.Goal()
            drive_goal.distance = 1.0  # 1 meter
            self.drive_ac.send_goal(drive_goal)

            sensor.poll()  # Read current dock status
            print(f"dock_status: {dock_status}")
        else:
            print("Not docked, driving...")
            # Drive
            self.drive_ac.wait_for_server()
            drive_goal = DriveDistance.Goal()
            drive_goal.distance = 1.0

    def dock_from_lib(self):
        """
        Attempt to dock the robot.
        If it fails, it will drive forward 0.5 meters and try again.
        """
        # see if we are already docked

        # self.dock_ac.wait_for_server()
        # dock_goal = irobot_create_msgs.action.Dock.Goal()
        # self.dock_ac.send_goal(dock_goal)

    def dock_reimplemented(self):
        """
        Attempt to dock the robot.
        If it fails, it will drive forward 0.5 meters and try again.
        """
        dock_status = sensor.poll()
        # TODO: Check if docked and return if we are
        # Docking is hard, so we will try 3 times
        while i < 3 and dock_status != "docked":
            print("Searching for Dock...")
            # TODO: Find the dock and dock
            print("Moving to dock...")
            # TODO: Check if docked and return if we are
            dock_status = sensor.poll()

            print("Failed to dock, trying again...")
            i += 1


if __name__ == "__main__":
    roomba = Roomba(namespace)
    exec = MultiThreadedExecutor(3)
    exec.add_node(roomba)
    exec.add_node(sensor)
    start = "S"
    drive_key = KeyCode(char=start)
    keycom = KeyCommander(
        [
            (drive_key, roomba.drive),
        ]
    )
    print(f"Press '{start}' to start")
    # dock_status = sensor.poll()
    # print(f"dock_status: {dock_status}")
    # roomba.drive()
    # roomba.dock()
    # Try/Except to shutdown "gracefully"
    try:
        exec.spin()
    except KeyboardInterrupt:
        rclpy.shutdown()
