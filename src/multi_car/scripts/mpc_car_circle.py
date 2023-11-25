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

def param_callback(config, level):
    return config

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

    # rospy.Subscriber("/car0/real_pose", PoseStamped, RealPosCallback, queue_size=1)
    # rospy.Subscriber("/car0/real_vel", TwistStamped, RealVelCallback, queue_size=1)
    # cmdvel_pub = rospy.Publisher('/car0/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber("/vrpn_client_node/car0/pose", PoseStamped, RealPosCallback, queue_size=1)
    rospy.Subscriber("/vrpn_client_node/car0/twist", TwistStamped, RealVelCallback, queue_size=1)
    cmdvel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    rate = rospy.Rate(int(1/ts))
    rospy.Subscriber("/control_ref0", ContorlRef, ref_callback, queue_size=1)
    myserver = server.Server(car_param_Config, param_callback)

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
        x0[3] = sqrt(real_vel.twist.linear.x**2+real_vel.twist.linear.y**2)
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
        cmd_vel.linear.x = x0[3]+u[0]*0.4
        cmd_vel.angular.z = x0[4]+u[1]*0.4

        cmdvel_pub.publish(cmd_vel)
        print("x0:",x0)
        print("x1:",x1)
        x0 = x1

        
        # print(u)
        time_record = timeit.default_timer() - start
        print("estimation time is {}".format(time_record))

        rate.sleep()