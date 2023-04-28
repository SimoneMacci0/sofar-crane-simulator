import rclpy
from rclpy.node import Node
from simple_pid import PID
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Point

class MotorYControllerNode(Node):

    def __init__(self):
        super().__init__("motor_y_controller")

        # Position variable
        self.y_pos = 0
        # Define PID controller
        self.pid = PID(5.0, 0.05, 0.01)
        
        # Update frequency (equivalent to publisher rate)
        self.dt = 0.01
        self.pid.sample_time = self.dt

        # Position error threshold and control saturation value
        self.control_clip_value = 100
        self.threshold = 1
        
        # Position publisher 
        self.pos_publisher = self.create_publisher(Float64, '/motor_y', 10)
        self.ack_publisher = self.create_publisher(Bool, "/ack_y", 10)
        # Setpoint subscriber
        self.setpoint_subscriber = self.create_subscription(Point, "/controller_setpoint", self.set_target_point, 10)

    # Reset PID target point and start new control loop timer
    def set_target_point(self, msg):
        # Reset to clear previous errors
        self.pid.reset()
        # Set new target point 
        self.pid.setpoint = msg.y
        self.get_logger().info('New target position {0}: set'.format(self.pid.setpoint))
        # Timer for control loop callback        
        self.timer = self.create_timer(self.dt, self.control_loop_callback)

    # Control loop cycle callback
    def control_loop_callback(self):
        # Compute control based on current state value
        control = self.pid(self.y_pos)
        
        # Saturate control input if necessary (optional)
        if control > self.control_clip_value:
            control = self.control_clip_value
        elif control < -self.control_clip_value:
            control = -self.control_clip_value
        self.get_logger().info('Control input: {0}'.format(control))

        # Update position based on control
        self.y_pos += control * self.dt
        self.get_logger().info('Current position: {0}'.format(self.y_pos))

        # Publish updated motor position
        pos_msg = Float64()
        pos_msg.data = self.y_pos
        self.pos_publisher.publish(pos_msg)
        
        # Cancel control loop if position is reached
        if abs(self.y_pos - self.pid.setpoint) < self.threshold:
            self.get_logger().info('Target position {0}: reached'.format(self.pid.setpoint))
            self.timer.cancel()

            # Send ack message
            ack_msg = Bool()
            ack_msg.data = True
            self.ack_publisher.publish(ack_msg)


def main(args=None):
    rclpy.init(args=args)
    controller = MotorYControllerNode()
    rclpy.spin(controller)
    rclpy.shutdown()


if __name__ == '__main__':
    main()


