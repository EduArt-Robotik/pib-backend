import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from pib_motors.motor import motors, Motor


class MotorCurrent(Node):

    def __init__(self):
        super().__init__("motor_current")

        self.motor_current_publisher = self.create_publisher(
            DiagnosticStatus, "motor_current", 10
        )

        self.pin_to_motors = {}
        self.setup_callbacks()
        self.timer = self.create_timer(1.0, self.publish_motor_current)

        self.get_logger().info("Now Running MOTOR CURRENT")

    def setup_callbacks(self, motor_list=None):
        if motor_list is None:
            motor_list = motors

        self.pin_to_motors = {}
        connected_bricklets = set()

        for motor in motor_list:
            for bp in motor.bricklet_pins:
                if bp.is_connected():
                    key = (bp.uid, bp.pin)
                    if key not in self.pin_to_motors:
                        self.pin_to_motors[key] = []
                    if motor.name not in self.pin_to_motors[key]:
                        self.pin_to_motors[key].append(motor.name)

    def publish_motor_current(self):
        for motor in motors:
            current = motor.get_current()
            if current == Motor.NO_CURRENT:
                continue
            self.publish_diagnostic_status(motor.name, current)

    def publish_diagnostic_status(self, motor_name: str, current: int) -> None:
        if current == Motor.NO_CURRENT:
            return
        msg = DiagnosticStatus()
        msg.level = DiagnosticStatus.WARN if current >= 1500 else DiagnosticStatus.OK
        msg.name = motor_name
        keyvalue = KeyValue()
        keyvalue.key = motor_name
        keyvalue.value = str(current)
        msg.values = [keyvalue]
        self.motor_current_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotorCurrent()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
