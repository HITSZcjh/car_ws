from MPC import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf.transformations as tf

import math
import timeit 
from dynamic_reconfigure import server
from multi_car.cfg import car_param_Config
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


class CircleTraj():
    def __init__(self, init_pos, omega):
        self.omega = omega
        self.radius = np.sqrt(init_pos[0]**2 + init_pos[1]**2)
        self.theta_bias = np.arctan2(init_pos[1], init_pos[0])
    def get_pos_vel(self, t):
        theta = self.omega * t + self.theta_bias
        x = self.radius * np.cos(theta)
        y = self.radius * np.sin(theta)
        vx = -self.radius * self.omega * np.sin(theta)
        vy = self.radius * self.omega * np.cos(theta)
        return np.array([x, y]), np.array([vx, vy])

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

car_odom = Odometry()
init_flag = False
def OdometryCallback(data):
    global init_flag
    init_flag = True
    global car_odom
    car_odom = data

init_theta = None
def QuaternionToTheta(quaternion):
    global init_theta
    if(init_theta == None):
        init_theta = tf.euler_from_quaternion(quaternion, axes='sxyz')[2]
    theta = tf.euler_from_quaternion(quaternion, axes='sxyz')[2]
    while((theta-init_theta)>0.5):
        theta -= np.pi*2
    while((theta-init_theta)<-0.5):
        theta += np.pi*2
    init_theta = theta
    return theta

def param_callback(config, level):
    # global vel_pid
    # global omega_pid
    # vel_pid.set_param(config["vel_kp"], config["vel_ki"], config["vel_kd"])
    # omega_pid.set_param(config["omega_kp"], config["omega_ki"], config["omega_kd"])

    # global vel_lpf
    # global omega_lpf
    # vel_lpf.set_param(config["cutoff_freq"])
    # omega_lpf.set_param(config["cutoff_freq"])
    # vel_pid.ref = config["vel"]
    # omega_pid.ref = config["omega"]
    return config

if __name__ == '__main__':
    rospy.init_node('mpc_circle', anonymous=True)
    ts = 0.01
    
    controller = diff_car_controller("circle_car")
    vel_pid = PID_t(0.0, 5.0, 0.1, 1.0, -1.0, ts)
    omega_pid = PID_t(0.0, 10.0, 0.01, 1.0, -1.0, ts)

    # vel_lpf = LPF(ts, 10)
    # omega_lpf = LPF(ts, 10)

    rospy.Subscriber("car2/odom", Odometry, OdometryCallback, queue_size=10)
    cmdvel_pub = rospy.Publisher('car2/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(int(1/ts))

    myserver = server.Server(car_param_Config, param_callback)

    while(init_flag == False):
        rate.sleep()

    # Set initial state
    traj = CircleTraj(omega=0.75, init_pos=[car_odom.pose.pose.position.x, car_odom.pose.pose.position.y])
    t_init = rospy.Time.now().to_sec()
    yref = controller.yref
    yref_e = controller.yref_e
    x0 = controller.x0

    while not rospy.is_shutdown():
        start = timeit.default_timer()
        t = rospy.Time.now().to_sec() - t_init
        for i in range(controller.N+1):
            if(i < controller.N):
                yref[0:2], vel = traj.get_pos_vel(t + i * controller.dT)
                yref[2] = np.arctan2(vel[1], vel[0])
                yref[3] = np.linalg.norm(vel)
                controller.solver.set(i, "yref", yref)
            else:
                yref_e[0:2], vel = traj.get_pos_vel(t + i * controller.dT)
                yref_e[2] = np.arctan2(vel[1], vel[0])
                yref_e[3] = np.linalg.norm(vel)
                controller.solver.set(i, "yref", yref_e)

        quaternion = [car_odom.pose.pose.orientation.x, car_odom.pose.pose.orientation.y, car_odom.pose.pose.orientation.z, car_odom.pose.pose.orientation.w]
        theta = QuaternionToTheta(quaternion)  
        x0[0] = car_odom.pose.pose.position.x
        x0[1] = car_odom.pose.pose.position.y
        x0[2] = theta

        x0[3] = car_odom.twist.twist.linear.x
        x0[4] = car_odom.twist.twist.angular.z

        # x0[3] = vel_lpf.filter(car_odom.twist.twist.linear.x)
        # x0[4] = omega_lpf.filter(car_odom.twist.twist.angular.z)

        u = controller.solver.solve_for_x0(x0)
        x1 = controller.solver.get(1, "x")

        vel_pid.ref = x1[3]
        vel_pid.fdb = x0[3]
        vel_pid.pid_calculate()

        omega_pid.ref = x1[4]
        omega_pid.fdb = x0[4]
        omega_pid.pid_calculate()

        cmd_vel = Twist()
        # cmd_vel.linear.y = x0[3]
        # cmd_vel.linear.z = omega_lpf.filter(x0[4])
        cmd_vel.linear.x = vel_pid.output
        cmd_vel.angular.z = omega_pid.output

        # cmd_vel.linear.x = x1[3]
        # cmd_vel.angular.z = x1[4]

        cmdvel_pub.publish(cmd_vel)

        print(x0)
        print(u)
        # time_record = timeit.default_timer() - start
        # print("estimation time is {}".format(time_record))

        # x0 = x1

        rate.sleep()