import time
import rclpy
from rclpy.node import Node
import math
from tier4_external_api_msgs.srv import Engage
from autoware_auto_control_msgs.msg import AckermannControlCommand
from tier4_control_msgs.msg import GateMode
from autoware_auto_vehicle_msgs.msg import VelocityReport
from autoware_auto_vehicle_msgs.msg import GearCommand
from rclpy.qos import qos_profile_services_default
from geometry_msgs.msg import Twist

class MnualVehicleController(Node):

    def __init__(self):
        super().__init__('manual_vehicle_controller')
        timer_period = 1.0
        self.target_vel = 0.0
        self.target_angle = 0.0
        self.vel_add = 0.0
        self.angle_add = 0.0
        
        self.MAX_VEL = 30
        self.MIN_VEL = -30
        
        # Initializing Default Count and Speed
        self.button_cnt = 0
        self.status ="stop"

        # onVelocity and accel
        self.previous_velocity_ = 0
        self.prev_stamp_ = None
        self.current_acceleration_ = 0.01
        self.current_velocity_ = 0.0
        self.current_a_des = 0.0
        self.dt = 0.1
        
        self.stop_time = 5.0
        self.stop_pub_num = int (self.stop_time / self.dt)
        self.min_acc = -1.0
        self.max_acc = 1.0

        # Publish gate mode
        self.pub_mannual_cmd_ = self.create_publisher(AckermannControlCommand, "/external/selected/control_cmd", 1)
        self.pub_gate_mode_ = self.create_publisher(GateMode, "/control/gate_mode_cmd", 1)
        self.pub_gear_cmd_ = self.create_publisher(GearCommand, "/external/selected/gear_cmd", 1)

        #Send engage survice
        self.srv_client_engage_ = self.create_client(Engage, "/api/autoware/set/engage")

        self.scale_factor = -0.25
        self.gear_cmd = GearCommand()

        #self.create_subscription(VelocityReport ,"/vehicle/status/velocity_status", self.onVelocity, 10)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        

    def initialize(self):
        # Set GateMode as External
        self.gate_cmd = GateMode()
        self.gate_cmd.data = GateMode.EXTERNAL
        self.pub_gate_mode_.publish(self.gate_cmd)

        # Request Engage
        req = Engage.Request()
        req.engage = True
        while not self.srv_client_engage_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.srv_client_engage_.call_async(req)

    def finalize(self):
        # Set GateMode as External
        self.gate_cmd = GateMode()
        self.gate_cmd.data = GateMode.AUTO
        self.pub_gate_mode_.publish(self.gate_cmd)

    def publish_manual_control(self, vel: float, angle: float, t: int):
        self.initialize()

        ## publish drive cmd
        pub_num = int(t / self.dt)
        self.get_logger().info("pub")
        self._set_ackermann_topic(vel, angle)
        self.pub_gear_cmd_.publish(self.gear_cmd)
        for i in range(pub_num):
            self.pub_mannual_cmd_.publish(self.ackermann_topic)
            time.sleep(self.dt)

        ## publish stop cmd
        self._set_ackermann_topic(0.0, 0.0)
        self.pub_gear_cmd_.publish(self.gear_cmd)
        for i in range(self.stop_pub_num):
            self.pub_mannual_cmd_.publish(self.ackermann_topic)
            time.sleep(self.dt)
        self.current_velocity_ = 0.0
        self.finalize()

    def timer_callback(self):
        self.get_logger().info("Call manual controller")
        self.target_vel = 5.0
        self.publish_manual_control(self.target_vel, 30 ,15)
        #print("Set Vel {:.2f}".format(self.target_vel))

    #def _set_ackermann_topic(self, target_acc:float, target_angle :float):
    def _set_ackermann_topic(self, target_vel:float, target_angle :float):
        target_vel = max(min(target_vel, self.MAX_VEL), self.MIN_VEL)        
        
         ## Generate ackermann topic
        self.ackermann_topic = AckermannControlCommand()
        self.ackermann_topic.stamp = self.get_clock().now().to_msg()
        ### Vehicle angle (rad)
        self.ackermann_topic.lateral.steering_tire_angle =  self.scale_factor * math.pi * (target_angle/ 180.0)
        ### Vehicle speed  (km/h) -> (m/s)
        self.ackermann_topic.longitudinal.speed = target_vel / 3.6

        k = -0.01
        self.current_velocity_ = self.current_velocity_ + self.current_a_des * self.dt
        target_acc = k * (self.current_velocity_ - target_vel) + self.current_acceleration_
        target_acc = min(max(target_acc, self.min_acc), self.max_acc)
        self.ackermann_topic.longitudinal.acceleration = target_acc

        eps = 0.001
        if (self.ackermann_topic.longitudinal.speed > eps):
            self.gear_cmd.command = GearCommand.DRIVE
            self.get_logger().info("DRIVE")
        elif (self.ackermann_topic.longitudinal.speed < -eps) :
            self.gear_cmd.command = GearCommand.REVERSE
            self.ackermann_topic.longitudinal.acceleration *= -1.0
            self.get_logger().info("REVERSE")
        else:
            self.gear_cmd.command = GearCommand.PARK
            self.get_logger().info("Park mode")

    # a_t = k * a_{t-1} + (1-k) * a_{t}
    def _lowpassFilter(self, current_value: float, prev_value: float, cutoff: float, dt: float) -> float:
        tau = 1.0 / (2.0 * math.pi * cutoff + 1.0e-5)
        k = tau / (dt + tau)
        return k * prev_value + (1.0 - k) * current_value

def main(args=None):
    print('Hi from MnualVehicleController')
    rclpy.init(args=args)
    
    controller_class = MnualVehicleController()
    rclpy.spin(controller_class)

    controller_class.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
