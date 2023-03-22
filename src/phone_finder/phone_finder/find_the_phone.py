import rclpy
from phone_finder.solver import VirtualPhone, Locator
from px4_msgs.msg import VehicleCommand
from flight_control.FlightControl import FlightControlNodeBase
import math
import numpy as np

class FindThePhone(FlightControlNodeBase):
    def __init__(self, node_name, queue_size=10):
        super().__init__(node_name, queue_size)
        self.offboard_setpoint_counter_ = 0
        self.pos_ctrl_timer = self.create_timer(0.1, self.pos_ctrl_cb)
        self.trajectory_counter = 0
        self.home = {'return': True, 'coord': [0, 0, -5]}
        self.destination = self.home['coord']
        self.locate_timer = self.create_timer(2, self.locate_cb)
        self.locator = Locator(50)
        self.detect_phone_timer = self.create_timer(2, self.detect_phone_cb)
        self.detect_phone_counter = 0
        self.max_n_samples = 100
        self.phone = [VirtualPhone(1, 10, -15, -5),
                      VirtualPhone(2, -300, 200, -20),
                      VirtualPhone(3, 500, -100, -30),]

    def pos_ctrl_cb(self):
        if self.offboard_setpoint_counter_ == 10:
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 6.)
            self.arm()
        if self.offboard_setpoint_counter_ < 11:
            self.offboard_setpoint_counter_ += 1
        self.trajectory_counter += 1
        if self.home['return']:
            self.set_position(
                x=self.home['coord'][0],
                y=self.home['coord'][1],
                z=self.home['coord'][2]
            )
        else:
            radius = 5
            x, y, _ = self.destination
            self.set_position(
                x=x + radius * math.sin(self.trajectory_counter / 30),
                y=y + radius * math.cos(self.trajectory_counter / 30),
                z=self.home['coord'][-1]
            )

    def locate_cb(self):
        IDs = [phone.id for phone in self.phone]
        dists = [phone(self.coord) for phone in self.phone]
        self.locator.collect(self.coord, IDs, dists)
        success_dict, new_found = self.locator.locate()
        if len(new_found) != 0:
            for id in new_found:
                x, y, z = self.locator.coord[id]()
                self.get_logger().warn(f"Found phone {id}: x={x:.3f}, y={y:.3f}, z={z:.3f}")

    def decide_next_phone(self):
        """
        获取距离当前位置最近的手机的id
        """
        min_dist = float('inf')
        nearest = None
        for phone in self.phone:
            id = phone.id
            mean_dist = float(self.locator.dist[id].apply(np.array).mean())
            if mean_dist < min_dist and not self.locator.found[id]:
                min_dist = mean_dist
                nearest = id
        return nearest


    def detect_phone_cb(self):
        """
        从home起飞后，重复：飞至距离最近手机处，绕圈飞行，直到位置收敛
        """
        self.detect_phone_counter += 1
        if self.detect_phone_counter < 10:
            return
        elif self.detect_phone_counter < 25:
            self.home['return'] = False
            return
        else:
            self.current_phone = self.decide_next_phone()
            if self.current_phone is None:
                self.home['return'] = True
            elif self.locator.coord[self.current_phone].data is not None:
                x, y, z = self.locator.coord[self.current_phone].mean()
                self.destination = [x, y, -5]
                self.get_logger().info(f"Current phone: {self.current_phone}, x={x:.3f}, y={y:.3f}, z={z:.3f}")

def main(args=None):
    print("Starting offboard control node...")
    rclpy.init(args=args) # 初始化rclpy
    node = FindThePhone("phone_detector")  # 新建一个节点
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy

if __name__ == '__main__':
    main()
