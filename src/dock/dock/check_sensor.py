import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Bool

import irobot_create_msgs
from irobot_create_msgs.msg import DockStatus

import os

DEBUG = False  # set to True to print debug messages


# I tried to separate this shared functionality into its own file
# but I couldn't get it to work with colcon
# so I just copied it into both files since I'm out of time
def to_bool(value) -> bool:
    """
    Converts an unknown type to a bool. Raises an exception if it gets a string it doesn't handle.
    Case is ignored for strings. These string values are handled:
    True: 'True', "1", "TRue", "yes", "y", "t"
    False: "", "0", "faLse", "no", "n", "f"
    """
    if isinstance(value, bool):
        return value
    if not value:
        return False
    if isinstance(value, str):
        value = value.lower().strip()
        if value in ["true", "1", "yes", "y", "t"]:
            return True
        if value in ["false", "0", "no", "n", "f"]:
            return False
        raise Exception(f"Invalid value for boolean conversion: {value}")
    raise Exception(f"Invalid value for boolean conversion: {value}")


class DockStatusController(Node):
    """
    DockStatusController is a ROS2 node that subscribes to the /dock_status topic and can publish to the /check_dock_status topic
    """

    def __init__(self, ns, timer=True):
        super().__init__("publisher")

        print(f"Docking Controller created for {ns}...")

        # This is the data that will be published
        self._is_docked = None
        self._is_dock_visible = None
        # output topic
        _output = "check_dock_status"

        # This is what 'sends' the data to the 'main' node
        print(f"Creating publisher for /{_output}...")
        try:
            self.publisher_ = self.create_publisher(
                msg_type=DockStatus, topic=_output, qos_profile=10
            )
        except Exception as e:
            print("Failed to create publisher")
            print(e)
            return

        # This subscribes to the /dock_status topic
        topic = f"/{ns}/dock_status"
        print(f"Subscribing to {topic}...")
        try:
            self.subscriber_ = self.create_subscription(
                msg_type=DockStatus,
                topic=topic,
                callback=self.listener,
                qos_profile=qos_profile_sensor_data,
            )
        except Exception as e:
            print("Failed to subscribe to dock_status")
            print(e)
            return

        if timer:
            self.init_timer()

    @property
    def is_docked(self) -> bool:
        """
        This is the docking status of the robot
        """
        return self._is_docked

    @is_docked.setter
    def is_docked(self, value) -> None:
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

    def init_timer(self) -> None:
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def listener(self, msg):
        """
        This will be called everytime the subscriber recieves a new message from the topic
        """
        if DEBUG:
            print(f"Docking Subscriber recieved: {msg}")

        self.is_docked = msg.is_docked
        self.is_dock_visible = msg.dock_visible

    def publish(self):
        """
        Publish the current docking status
        """
        # return early if something is wrong
        if self.is_docked is None:
            return

        msg = DockStatus()
        if DEBUG:
            print(f"Docking Publisher sending: {msg}")

        msg.is_docked = to_bool(self.is_docked)
        msg.dock_visible = to_bool(self.is_dock_visible)
        self.publisher_.publish(msg)

    def timer_callback(self):
        """
        This will be called by the timer to check for new messages
        """
        self.publish()


def main(args=None):
    rclpy.init(args=args)
    namespace = os.environ.get("BOWSER_NAME", "create3_05F8")
    sensor = DockStatusController(namespace)
    rclpy.spin(sensor)
    sensor.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
