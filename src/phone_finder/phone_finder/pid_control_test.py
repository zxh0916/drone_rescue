import rclpy
from px4_msgs.msg import VehicleCommand
import numpy as np
from flight_control.FlightControl import FlightControlNodeBase, SimplePIDController
import math

class CustomNode(FlightControlNodeBase):
    def __init__(self, node_name, queue_size=10):
        super().__init__(node_name, queue_size)
        self.offboard_setpoint_counter_ = 0
        self.pd_control_counter = 0
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.curr_error = np.zeros(3)
        self.prev_error = np.zeros(3)
        self.target_coord = np.zeros(3)

        self.pid = SimplePIDController(0.5, 1e-4, 0.5, 3)
        
    def timer_callback(self):
        if self.offboard_setpoint_counter_ == 10:
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                1.,
                6.
            )
            self.arm()
        if self.offboard_setpoint_counter_ < 11:
            self.offboard_setpoint_counter_ += 1
        else:
            self.pd_control_counter += 1
        stage = self.pd_control_counter // 30
        if stage < 5:
            self.set_position(0, 0, 5)
        else:
            r = 40
            ang = self.pd_control_counter / 30
            self.pid_control_velocity(np.array([r*math.sin(ang), r*math.cos(ang), 5], dtype=float))
        # elif stage < 10:
        #     self.pid_control_velocity(np.array([5, 5, 5], dtype=float))
        # elif stage < 15:
        #     self.pid_control_velocity(np.array([5, -5, 5], dtype=float))
        # elif stage < 20:
        #     self.pid_control_velocity(np.array([-5, -5, 5], dtype=float))
        # elif stage < 25:
        #     self.pid_control_velocity(np.array([-5, 5, 5], dtype=float))
        # elif stage < 30:
        #     self.pid_control_velocity(np.array([5, 5, 5], dtype=float))


    def pid_control_velocity(self, target_coord):
        control = self.pid(self.coord, target_coord)
        control = [float(control[i]) for i in range(len(control))]
        self.set_velocity(*control)


def main(args=None):
    print("Starting offboard control node...")
    rclpy.init(args=args) # 初始化rclpy
    node = CustomNode("simple_pd_control")  # 新建一个节点
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy

if __name__ == '__main__':
    main()
