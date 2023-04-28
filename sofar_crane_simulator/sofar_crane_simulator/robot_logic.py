import rclpy
from rclpy.node import Node
import time

from std_msgs.msg import Int64, Bool


class RobotLogic(Node):

    def __init__(self):
        super().__init__("robot_logic_node")

        # bool variables to keep track of controllers' state
        self.mot_x_idle = True
        self.mot_y_idle = True

        # Stage variable
        self.next_stage = 0
        
        # next stage publisher 
        self.stage_pub = self.create_publisher(Int64, '/next_stage', 10)

        # acks subscribers
        self.ack_x_sub = self.create_subscription(Bool, "/ack_x", self.ack_x_callback, 10)
        self.ack_y_sub = self.create_subscription(Bool, "/ack_y", self.ack_y_callback, 10)

        # Sleep to allow other nodes to fully initialize before starting time
        time.sleep(2)
        self.motors_state_timer = self.create_timer(1, self.check_idle_motors)

    def ack_x_callback(self, msg: Bool):
        self.mot_x_idle = msg.data

    def ack_y_callback(self, msg: Bool):
        self.mot_y_idle = msg.data

    def check_idle_motors(self):
        # Publish next stage only if both motors are idle
        if self.mot_x_idle and self.mot_y_idle:
            # Reset idle state to avoid infinite loop
            self.mot_x_idle = False
            self.mot_y_idle = False
            # Increase internal next stage var
            self.next_stage = (self.next_stage % 5) + 1
            # Publish msg 
            next_stage_msg = Int64()
            next_stage_msg.data = self.next_stage
            self.stage_pub.publish(next_stage_msg)

   
def main(args=None):
    rclpy.init(args=args)
    robot_logic = RobotLogic()
    rclpy.spin(robot_logic)
    rclpy.shutdown()


if __name__ == '__main__':
    main()


