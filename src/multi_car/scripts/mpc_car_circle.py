from MPC import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf.transformations as tf

import math
import timeit 
from dynamic_reconfigure import server
from multi_car.cfg import car_param_Config  
from multi_car.msg import ContorlRef
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

car_odom = Odometry()
init_flag0 = False
def OdometryCallback(data):
    global init_flag0
    init_flag0 = True
    global car_odom
    car_odom = data

init_theta = None
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

if __name__ == '__main__':
    rospy.init_node('mpc_circle', anonymous=True)
    ts = 0.02
    
    controller = diff_car_controller("circle_car_mpc")

    rospy.Subscriber("car0/odom", Odometry, OdometryCallback, queue_size=10)
    cmdvel_pub = rospy.Publisher('car0/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(int(1/ts))
    rospy.Subscriber("/control_ref0", ContorlRef, ref_callback, queue_size=1)
    myserver = server.Server(car_param_Config, param_callback)

    while(init_flag0 == False or init_flag1 == False):
        rate.sleep()

    # Set initial state
    traj = CircleTraj(omega=1.0, init_pos=[car_odom.pose.pose.position.x, car_odom.pose.pose.position.y], radius=1.0)
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
                yref[0:2] = np.array([ref.pos_x[i], ref.pos_y[i]])
                controller.solver.set(i, "yref", yref)
            else:
                yref_e[0:2], vel = traj.get_pos_vel(t + i * controller.dT)
                yref_e[0:2] = np.array([ref.pos_x[i], ref.pos_y[i]])
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

        for j in range(10):
            u = controller.solver.solve_for_x0(x0)
            residuals = controller.solver.get_residuals()
            if max(residuals)<1e-6:
                break
        print("SQP_RTI iterations:\n", residuals)
        x1 = controller.solver.get(1, "x")

        cmd_vel = Twist()
        cmd_vel.linear.x = x0[3]+u[0]*0.05
        cmd_vel.angular.z = x0[4]+u[1]*0.05

        cmdvel_pub.publish(cmd_vel)

        x0 = x1

        print(x0)
        # print(u)
        time_record = timeit.default_timer() - start
        print("estimation time is {}".format(time_record))

        rate.sleep()