from MPC import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf.transformations as tf

import math
import timeit 
from dynamic_reconfigure import server
from multi_car.cfg import car_param_Config  
from multi_car.msg import ContorlRef
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped

class PID_t:
    def __init__(self, kp, ki, kd, output_max, output_min, ts):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_max = output_max
        self.output_min = output_min
        self.ts = ts

        self.ref = 0.0
        self.fdb = 0.0
        self.error = 0.0
        self.last_error = 0.0
        self.total_error = 0.0
        self.output = 0.0
        self.p_output = 0.0
        self.d_output = 0.0
        self.i_output = 0.0

    def pid_calculate(self):
        self.error = self.ref - self.fdb
        self.p_output = self.kp * self.error
        self.d_output = self.kd * (self.error - self.last_error) / self.ts

        if not (self.error > 0 and self.total_error > self.output_max / self.ki) and not (
            self.error < 0 and self.total_error < self.output_min / self.ki
        ):
            self.total_error += self.error * self.ts

        self.i_output = self.ki * self.total_error
        self.output = self.p_output + self.i_output + self.d_output

        if self.output > self.output_max:
            self.output = self.output_max
        elif self.output < self.output_min:
            self.output = self.output_min

        self.last_error = self.error

    def set_param(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.total_error = 0.0

class LPF():
    def __init__(self, ts, cutoff_freq):
        self.ts = ts
        self.cutoff_freq = cutoff_freq
        self.last_output = 0.0
    def filter(self, input):
        output = (self.cutoff_freq * self.ts * input + self.last_output) / (self.cutoff_freq * self.ts + 1)
        self.last_output = output
        return output
    def set_param(self, cutoff_freq):
        self.cutoff_freq = cutoff_freq
        self.last_output = 0.0


def param_callback(config, level):
    print(config)
    # global vel_pid
    global omega_pid
    # vel_pid.set_param(config["vel_kp"], config["vel_ki"], config["vel_kd"])
    omega_pid.set_param(config["omega_kp"], config["omega_ki"], config["omega_kd"])

    # global vel_lpf
    global omega_lpf
    # vel_lpf.set_param(config["cutoff_freq"])
    omega_lpf.set_param(config["cutoff_freq"])
    # vel_pid.ref = config["vel"]
    omega_pid.ref = config["omega"]
    return config

class CircleTraj():
    def __init__(self, init_pos, omega, radius):
        self.omega = omega
        self.radius = radius
        self.theta_bias = np.arctan2(init_pos[1], init_pos[0])
    def get_pos_vel(self, t):
        theta = self.omega * t + self.theta_bias
        x = self.radius * np.cos(theta)
        y = self.radius * np.sin(theta)
        vx = -self.radius * self.omega * np.sin(theta)
        vy = self.radius * self.omega * np.cos(theta)
        return np.array([x, y]), np.array([vx, vy])

real_pos = PoseStamped()
init_flag0 = False
def RealPosCallback(data):
    global init_flag0
    init_flag0 = True
    global real_pos
    real_pos = data

real_vel = TwistStamped()
init_flag2 = False
def RealVelCallback(data):
    global init_flag2
    init_flag2 = True
    global real_vel
    real_vel = data

def QuaternionToTheta(quaternion):
    # global init_theta
    # if(init_theta == None):
    #     init_theta = tf.euler_from_quaternion(quaternion, axes='sxyz')[2]
    theta = tf.euler_from_quaternion(quaternion, axes='sxyz')[2]
    # while((theta-init_theta)>0.5):
    #     theta -= np.pi*2
    # while((theta-init_theta)<-0.5):
    #     theta += np.pi*2
    # init_theta = theta
    return theta

ref = ContorlRef()
init_flag1 = False
def ref_callback(data):
    global init_flag1
    init_flag1 = True
    global ref
    ref = data

def GetThetaBias():
    global real_pos, cmdvel_pub, rate
    init_pos = np.array([real_pos.pose.position.x, real_pos.pose.position.y])
    cmd_vel_msg = Twist()
    init_theta = QuaternionToTheta([real_pos.pose.orientation.x, real_pos.pose.orientation.y, real_pos.pose.orientation.z, real_pos.pose.orientation.w])
    cmd_vel_msg.linear.x = 0.2
    cmd_vel_msg.linear.y = 0.0
    cmd_vel_msg.angular.z = 0.0
    for i in range(100):
        cmdvel_pub.publish(cmd_vel_msg)
        rate.sleep()
    cmd_vel_msg.linear.x = 0.0
    cmdvel_pub.publish(cmd_vel_msg)
    rate.sleep()
    delta = np.array([real_pos.pose.position.x, real_pos.pose.position.y]) - init_pos
    theta_bias = np.arctan2(delta[1], delta[0]) - init_theta
    print("theta_bias is {}".format(theta_bias))
    return theta_bias
if __name__ == '__main__':
    rospy.init_node('mpc_circle', anonymous=True)
    ts = 0.02
    
    controller = diff_car_controller("circle_car_mpc")

    vel_pid = PID_t(1.0, 2.0, 0.2, 0.4, -0.4, ts)
    # omega_pid = PID_t(0.0, 1.0, 0.0, 0.5, -0.5, ts)
    vel_lpf = LPF(ts, 10)
    # omega_lpf = LPF(ts, 10)



    # rospy.Subscriber("/car0/real_pose", PoseStamped, RealPosCallback, queue_size=1)
    # rospy.Subscriber("/car0/real_vel", TwistStamped, RealVelCallback, queue_size=1)
    # cmdvel_pub = rospy.Publisher('/car0/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber("/vrpn_client_node/car0/pose", PoseStamped, RealPosCallback, queue_size=1)
    rospy.Subscriber("/vrpn_client_node/car0/twist", TwistStamped, RealVelCallback, queue_size=1)
    cmdvel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    rate = rospy.Rate(int(1/ts))
    rospy.Subscriber("/control_ref0", ContorlRef, ref_callback, queue_size=1)
    # myserver = server.Server(car_param_Config, param_callback)

    while(init_flag0 == False or init_flag2 == False):
        rate.sleep()

    theta_bias = GetThetaBias()

    while(init_flag1 == False):
        rate.sleep()

    # Set initial state
    yref = controller.yref
    yref_e = controller.yref_e
    x0 = controller.x0

    while not rospy.is_shutdown():
        start = timeit.default_timer()
        for i in range(controller.N+1):
            if(i < controller.N):
                yref[0:2] = np.array([ref.pos_x[i], ref.pos_y[i]])
                controller.solver.set(i, "yref", yref)
            else:
                yref_e[0:2] = np.array([ref.pos_x[i], ref.pos_y[i]])
                controller.solver.set(i, "yref", yref_e)

        quaternion = [real_pos.pose.orientation.x, real_pos.pose.orientation.y, real_pos.pose.orientation.z, real_pos.pose.orientation.w]
        theta = QuaternionToTheta(quaternion) + theta_bias 
        x0[0] = real_pos.pose.position.x
        x0[1] = real_pos.pose.position.y
        x0[2] = theta

        direction = cos(theta)*real_vel.twist.linear.x + sin(theta)*real_vel.twist.linear.y
        if(direction>0):
            x0[3] = sqrt(real_vel.twist.linear.x**2+real_vel.twist.linear.y**2)
        else:
            x0[3] = -sqrt(real_vel.twist.linear.x**2+real_vel.twist.linear.y**2)
        x0[4] = real_vel.twist.angular.z * 100
        # x0[4] = real_vel.twist.angular.z

        for j in range(10):
            u = controller.solver.solve_for_x0(x0)
            residuals = controller.solver.get_residuals()
            if max(residuals)<1e-6:
                break
        # print("SQP_RTI iterations:\n", residuals)
        x1 = controller.solver.get(1, "x")

        cmd_vel = Twist()

        vel_pid.fdb = vel_lpf.filter(x0[3])
        vel_pid.ref = x0[3]+u[0]*0.2
        vel_pid.pid_calculate()
        cmd_vel.linear.x = vel_pid.output
        cmd_vel.angular.z = x0[4]+u[1]*0.2


        # cmd_vel.linear.x = x0[3]+u[0]*0.1
        # cmd_vel.angular.z = x0[4]+u[1]*0.1

        cmdvel_pub.publish(cmd_vel)
        print("x0:",x0)
        print("x1:",x1)
        # x0 = x1

        
        # print(u)
        time_record = timeit.default_timer() - start
        print("estimation time is {}".format(time_record))

        rate.sleep()