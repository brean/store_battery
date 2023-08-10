import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState


# number of cells in your battery pack
CELLS = 3
# desired cell voltage for storage
# should be around 3.7.
# Because the cells might be charged unevenly you could set something
# a little bit lower (e.g. 3.65) and then use a balanced charger to
# get them charged evenly.
CELL_VOLTAGE = 3.7

PACK_VOLTAGE = CELL_VOLTAGE * CELLS

VELOCITY_TOPIC = '/cmd_vel'
BATTERY_TOPIC = '/battery_state'


class LowerVoltage(Node):

    def __init__(self):
        super().__init__('lower_voltage')
        self.twist = Twist()
        self.voltage = -1
        self.last_voltage = -1

        self.sub_battery = self.create_subscription(
            BatteryState, BATTERY_TOPIC, self.upade_battery, 10)
        self.pub_vel = self.create_publisher(Twist, VELOCITY_TOPIC, 1)
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.send_velocity)
        
        

    def send_velocity(self):
        rotation_speed = 0.1
        low_voltage = self.voltage < PACK_VOLTAGE
        if low_voltage:
            if low_voltage != -1 and self.twist.angular.z > 0:
                self.get_logger().info(
                    f'Voltage at: {self.voltage} (below {PACK_VOLTAGE})'
                    ', stopped moving')
            rotation_speed = 0.0
        
        self.twist.angular.z = rotation_speed
        self.pub_vel.publish(self.twist)

    def upade_battery(self, msg):
        self.voltage = msg.voltage
        if abs(self.voltage - self.last_voltage) > 0.045:
            self.last_voltage = self.voltage
            self.get_logger().info(f'Current voltage: {self.voltage}')

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