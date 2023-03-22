import rclpy
from px4_msgs.msg import VehicleCommand
import math
from flight_control.FlightControl import FlightControlNodeBase
class DrawCircle(FlightControlNodeBase):
    def __init__(self, node_name, queue_size=10):
        super().__init__(node_name, queue_size)
        self.offboard_setpoint_counter_ = 0
        self.trajectory_counter = 0
        self.timer = self.create_timer(0.1, self.timer_callback)
        
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
            self.trajectory_counter += 1
        stage = self.trajectory_counter // 30
        if stage < 5:
            self.set_position(0, 0, -5)
        else:
            radius = 5
            self.set_pos_vel(
                None, None, -5,
                radius * math.sin(self.trajectory_counter / 30),
                radius * math.cos(self.trajectory_counter / 30),
                None
            )

def main(args=None):
    print("Starting offboard control node...")
    rclpy.init(args=args) # 初始化rclpy
    node = DrawCircle("draw_circle")  # 新建一个节点
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy

if __name__ == '__main__':
    main()
