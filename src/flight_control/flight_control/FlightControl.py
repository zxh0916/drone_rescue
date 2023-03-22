import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import Timesync
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleControlMode
from px4_msgs.msg import VehicleOdometry
from builtin_interfaces.msg import Time
import math
import numpy as np

class SimplePIDController:
    def __init__(
        self, 
        kp: float, 
        ki: float, 
        kd: float, 
        dim: int
    ):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dim = dim
        self.reset()
    def __call__(self, output, input):
        assert(isinstance(output, (np.ndarray, list)))
        assert(isinstance(input, (np.ndarray, list)))
        if isinstance(input, list):
            input = np.array(input, dtype=float)
        if isinstance(output, list):
            output = np.array(output, dtype=float)
        assert(len(output) == self.dim)
        assert(len(input) == self.dim)
        self.curr_error = input - output
        self.control += self.kp * (self.curr_error - self.prev_error)
        self.control += self.ki * self.curr_error
        self.control += self.kd * (self.curr_error - 2*self.prev_error + self.prev_prev_error)
        self.prev_prev_error = self.prev_error[:]
        self.prev_error = self.curr_error[:]
        return self.control
    
    def reset(self):
        self.curr_error = np.zeros(self.dim, dtype=float)
        self.prev_error = np.zeros(self.dim, dtype=float)
        self.prev_prev_error = np.zeros(self.dim, dtype=float)
        self.acc_error = np.zeros(self.dim, dtype=float)
        self.control = np.zeros(self.dim, dtype=float)


class FlightControlNodeBase(Node):
    def __init__(self, node_name, queue_size=10):
        super().__init__(node_name)
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode,
            "fmu/offboard_control_mode/in",
            queue_size
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            "fmu/vehicle_command/in",
            queue_size
        )
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            "fmu/trajectory_setpoint/in",
            queue_size
        )
        self.vehicle_odometry_sub = self.create_subscription(
            VehicleOdometry,
            "fmu/vehicle_odometry/out",
            self.vehicle_odom_cb,
            queue_size
        )
        self.coord = np.zeros(3, dtype=float)
        self.q = np.zeros(4, dtype=float)
        self.velocity = np.zeros(3, dtype=float)

    def vehicle_odom_cb(self, msg):
        self.coord[0], self.coord[1], self.coord[2] = msg.x, msg.y, msg.z
        self.velocity[0], self.velocity[1], self.velocity[2] = msg.vx, msg.vy, msg.vz
        for i in range(4):
            self.q[i] = msg.q[i]

    def publish_vehicle_command(self, command, param1=.0, param2=.0):
        msg = VehicleCommand()
        time = self.get_clock().now().to_msg()
        msg.timestamp = int(time.nanosec * 1e-6 + time.sec * 1e3)
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)

    def publish_offboard_control_mode(self, pos, vel, acc):
        msg = OffboardControlMode()
        time = self.get_clock().now().to_msg()
        msg.timestamp = int(time.nanosec * 1e-6 + time.sec * 1e3)
        msg.position = pos
        msg.velocity = vel
        msg.acceleration = acc
        msg.attitude = False
        msg.body_rate = False
        self.offboard_control_mode_pub.publish(msg)
        

    def set_position(self, x=None, y=None, z=None):
        msg = TrajectorySetpoint()
        time = self.get_clock().now().to_msg()
        msg.timestamp = int(time.nanosec * 1e-6 + time.sec * 1e3)
        self.publish_offboard_control_mode(True, False, False)
        msg.x = float(x) if x is not None else float('nan')
        msg.y = float(y) if y is not None else float('nan')
        msg.z = float(z) if z is not None else float('nan')
        self.trajectory_setpoint_pub.publish(msg)

    def set_velocity(self, vx=None, vy=None, vz=None):
        msg = TrajectorySetpoint()
        time = self.get_clock().now().to_msg()
        msg.timestamp = int(time.nanosec * 1e-6 + time.sec * 1e3)
        self.publish_offboard_control_mode(False, True, False)
        msg.x = msg.y = msg.z = float('nan')
        msg.vx = float(vx) if vx is not None else float('nan')
        msg.vy = float(vy) if vy is not None else float('nan')
        msg.vz = float(vz) if vz is not None else float('nan')
        self.trajectory_setpoint_pub.publish(msg)

    def set_pos_vel(self, x=None, y=None, z=None, vx=None, vy=None, vz=None):
        msg = TrajectorySetpoint()
        time = self.get_clock().now().to_msg()
        msg.timestamp = int(time.nanosec * 1e-6 + time.sec * 1e3)
        self.publish_offboard_control_mode(True, True, False)
        msg.x = float(x) if x is not None else float('nan')
        msg.y = float(y) if y is not None else float('nan')
        msg.z = float(z) if z is not None else float('nan')
        msg.vx = float(vx) if vx is not None else float('nan')
        msg.vy = float(vy) if vy is not None else float('nan')
        msg.vz = float(vz) if vz is not None else float('nan')
        self.trajectory_setpoint_pub.publish(msg)

    def arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1.0
        )
        self.get_logger().info("Arm command send")

    def disarm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            0.0
        )
        self.get_logger().info("Disarm command send")
