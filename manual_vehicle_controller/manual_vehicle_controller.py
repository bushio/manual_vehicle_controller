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
        timer_period = 0.1
        self.target_vel = 0.0
        self.target_angle = 0.0
        self.vel_add = 0.0
        self.angle_add = 0.0
        
        self.MAX_VEL = 30
        self.MIN_VEL = 0
        
        # Initializing Default Count and Speed
        self.button_cnt = 0

        # onVelocity and accel
        self.previous_velocity_ = 0
        self.prev_stamp_ = None
        self.current_acceleration_ = 0.01
        self.current_velocity_ = 0.0

        # Publish gate mode
        self.pub_mannual_cmd_ = self.create_publisher(AckermannControlCommand, "/external/selected/control_cmd", 1)
        self.pub_gate_mode_ = self.create_publisher(GateMode, "/control/gate_mode_cmd", 1)
        self.pub_gear_cmd_ = self.create_publisher(GearCommand, "/external/selected/gear_cmd", 1)

        #Send engage survice
        self.srv_client_engage_ = self.create_client(Engage, "/api/autoware/set/engage")

        self.scale_factor = -0.25
        self.gear_cmd = GearCommand()

        self.create_subscription(VelocityReport ,"/vehicle/status/velocity_status", self.onVelocity, 1)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def publish_manual_control(self, vel: float, angle: float, t: int):
        # Set GateMode as External
        self.gate_cmd = GateMode()
        self.gate_cmd.data = GateMode.EXTERNAL
        self.pub_gate_mode_.publish(self.gate_cmd)
        
        # Request Engage
        req = Engage.Request()
        req.engage = True
        self.future = self.srv_client_engage_.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        
        ## Change gear mode
        self.pub_gear_cmd_.publish(self.gear_cmd)
        sleep_time = 0.1
        pub_num = int(t / sleep_time)
        print(pub_num)
        
        ## publish drive cmd
        for i in range(3):
            time.sleep(sleep_time)
            self._set_ackermann_topic(vel, angle)
            self.pub_mannual_cmd_.publish(self.ackermann_topic)
        
        ## publish stop cmd
        for i in range(5):
            self._set_ackermann_topic(0, 0)
            self.pub_mannual_cmd_.publish(self.ackermann_topic)

        # Set GateMode as External
        self.gate_cmd = GateMode()
        self.gate_cmd.data = GateMode.AUTO
        self.pub_gate_mode_.publish(self.gate_cmd)

    def onVelocity(self, msg: VelocityReport):
        """
        Set current velocity
        """
        self.current_velocity_ = msg.longitudinal_velocity
        cutoff = 0.0
        dt = 0.0
        acc = 0.01
        if (self.previous_velocity_):
            cutoff = 10.0
            dt = 1.0 / 10.0
            acc = (self.current_velocity_ - self.previous_velocity_) / dt
        if (self.current_acceleration_):
            self.current_acceleration_ = acc
        else:
            self.current_acceleration_ = self._lowpassFilter(acc, self.current_acceleration_, cutoff, dt)

        self.previous_velocity_ = msg.longitudinal_velocity
        self.prev_stamp_ = msg.header.stamp

    def timer_callback(self):
        self.get_logger().info("Call manual controller")
        self.target_vel = 5.0
        self.publish_manual_control(-1, 0 ,10)
        #print("Set Vel {:.2f}".format(self.target_vel))

    def _set_ackermann_topic(self, target_vel:float, target_angle :float):
        target_vel = max(min(target_vel, self.MAX_VEL), self.MIN_VEL)        
        
         ## Generate ackermann topic
        self.ackermann_topic = AckermannControlCommand()
        self.ackermann_topic.stamp = self.get_clock().now().to_msg()
        ### Vehicle angle (rad)
        self.ackermann_topic.lateral.steering_tire_angle =  self.scale_factor * math.pi * (target_angle/ 180.0)
        ### Vehicle speed  (km/h) -> (m/s)
        self.ackermann_topic.longitudinal.speed = target_vel / 3.6

        if (self.current_acceleration_):
            k = -0.5
            v = self.current_velocity_
            v_des = self.ackermann_topic.longitudinal.speed
            a_des = k * (v - v_des) + self.current_acceleration_

            # clamp(a_des, -1.0, 1.0)
            low = -1.0
            high = 1.0
            if a_des < low:
                a_des = low
            elif a_des > high:
                a_des = high
            self.ackermann_topic.longitudinal.acceleration = a_des
    
            eps = 0.001
            if (self.ackermann_topic.longitudinal.speed > eps):
                self.gear_cmd.command = GearCommand.DRIVE
            elif (self.ackermann_topic.longitudinal.speed < -eps) :
                self.gear_cmd.command = GearCommand.REVERSE
                self.ackermann_topic.longitudinal.acceleration *= -1.0
            else:
                self.gear_cmd.command = GearCommand.PARK

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
