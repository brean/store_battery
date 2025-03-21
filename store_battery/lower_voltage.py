import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState

from turtlebot3_msgs.srv import Sound


# number of cells in your battery pack
CELLS = 3
# desired cell voltage for storage
# should be around 3.7.
# Because the cells might be charged unevenly you could set something
# a little bit lower (e.g. 3.65) and then use a balanced charger to
# get them charged evenly.
CELL_VOLTAGE = 3.75

PACK_VOLTAGE = CELL_VOLTAGE * CELLS

VELOCITY_TOPIC = '/cmd_vel'
BATTERY_TOPIC = '/battery_state'
SOUND_TOPIC = '/sound'
SOUND = 0  # OFF-sound


class LowerVoltage(Node):
    def __init__(self):
        super().__init__('lower_voltage')
        self.get_logger().info(f'Low voltage will be {PACK_VOLTAGE:.3f} volt.')

        self.voltage = -1
        self.last_voltage = -1
        self.rotation_speed = -1
        self.last_sound = -1

        self.sub_battery = self.create_subscription(
            BatteryState, BATTERY_TOPIC, self.upade_battery, 10)
        self.pub_vel = self.create_publisher(Twist, VELOCITY_TOPIC, 1)
        self.sound_cli = self.create_client(Sound, SOUND_TOPIC)

        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.send_velocity)

    def send_velocity(self):
        low_voltage = self.voltage < PACK_VOLTAGE
        rotation_speed = 0.0 if low_voltage else 0.1
        if low_voltage and self.voltage != -1:
            self.get_logger().info(
                f'Voltage at: {self.voltage:.3f} (below {PACK_VOLTAGE:.3f})'
                ', stopped moving')
            self.play_sound()
        if self.rotation_speed != rotation_speed:
            self.rotation_speed = rotation_speed
            twist = Twist()
            twist.angular.z = self.rotation_speed
            self.pub_vel.publish(twist)
            self.get_logger().info(f'Publishing: "{twist}"')

    def play_sound(self):
        now = time.time()
        if now - self.last_sound > 5:
            self.last_sound = now
            self.sound_cli.call(Sound.Request(value=SOUND))

    def send_stop(self):
        self.pub_vel.publish(Twist())

    def upade_battery(self, msg):
        self.voltage = float(msg.voltage)
        percentage = float(msg.percentage)
        if abs(self.voltage - self.last_voltage) > 0.045:
            self.last_voltage = self.voltage
            self.get_logger().info(
                f'Current voltage: {self.voltage:.3f} ({percentage:.1f} %)')


def main(args=None):
    rclpy.init(args=args)

    lv_node = LowerVoltage()

    rclpy.spin(lv_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lv_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
