import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import select
import sys
import termios
import tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1),
    'O': (1, -1, 0, 0),
    'I': (1, 0, 0, 0),
    'J': (0, 1, 0, 0),
    'L': (0, -1, 0, 0),
    'U': (1, 1, 0, 0),
    '<': (-1, 0, 0, 0),
    '>': (-1, -1, 0, 0),
    'M': (-1, 1, 0, 0),
    't': (0, 0, 1, 0),
    'b': (0, 0, -1, 0),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher = self.create_publisher(Twist, '/vmegarover/cmd_vel', 10)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0

        self.timer_period = 0.1  # 100ms
        self.timer = self.create_timer(self.timer_period, self.publish_twist)

        self.get_logger().info("Keyboard Teleop Node Initialized")

    def publish_twist(self):
        twist = Twist()
        twist.linear.x = self.x * self.speed
        twist.linear.y = self.y * self.speed
        twist.linear.z = self.z * self.speed
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.th * self.turn

        self.publisher.publish(twist)

    def wait_for_subscribers(self):
        i = 0
        while rclpy.ok() and self.publisher.get_subscription_count() == 0:
            if i == 4:
                self.get_logger().info(
                    "Waiting for subscriber to connect to /vmegarover/cmd_vel"
                )
            time.sleep(0.5)
            i += 1
            i = i % 5
        if not rclpy.ok():
            raise Exception(
                "Got shutdown request before subscribers connected"
            )

    def update(self, key):
        if key in moveBindings.keys():
            self.x = moveBindings[key][0]
            self.y = moveBindings[key][1]
            self.z = moveBindings[key][2]
            self.th = moveBindings[key][3]
        elif key in speedBindings.keys():
            self.speed = self.speed * speedBindings[key][0]
            self.turn = self.turn * speedBindings[key][1]
            self.get_logger().info(f"currently:\tspeed {self.speed}\tturn {self.turn}")
        else:
            # Skip updating cmd_vel if key timeout and robot already
            # stopped.
            if (
                key == ""
                and self.x == 0.0
                and self.y == 0.0
                and self.z == 0.0
                and self.th == 0.0
            ):
                return
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            self.th = 0.0
            if key == '\x03':
                rclpy.shutdown()

def getKey(key_timeout, settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    rclpy.init()
    node = KeyboardTeleop()
    settings = termios.tcgetattr(sys.stdin)

    node.declare_parameter("speed", 0.5)
    node.declare_parameter("turn", 1.0)
    node.declare_parameter("key_timeout", 0.0)

    node.speed = node.get_parameter("speed").value
    node.turn = node.get_parameter("turn").value
    key_timeout = node.get_parameter("key_timeout").value
    if key_timeout == 0.0:
        key_timeout = None

    node.wait_for_subscribers()

    print(msg)
    while rclpy.ok():
        key = getKey(key_timeout, settings)
        node.update(key)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)