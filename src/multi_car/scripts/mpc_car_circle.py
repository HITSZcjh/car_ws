from MPC import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf.transformations as tf


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
    
car_odom = Odometry()
init_flag = False
def OdometryCallback(data):
    global init_flag
    init_flag = True
    global car_odom
    car_odom = data

def QuaternionToTheta(quaternion):
    euler = tf.euler_from_quaternion(quaternion, axes='sxyz')
    return euler[2]

if __name__ == '__main__':
    rospy.init_node('mpc_circle', anonymous=True)
    controller = diff_car_controller("circle_car")
    rospy.Subscriber("car2/odom", Odometry, OdometryCallback)
    cmdvel_pub = rospy.Publisher('car2/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(50)

    while(init_flag == False):
        rate.sleep()

    # Set initial state
    traj = CircleTraj(omega=0.5, init_pos=[car_odom.pose.pose.position.x, car_odom.pose.pose.position.y])
    t_init = rospy.Time.now().to_sec()
    yref = controller.yref
    yref_e = controller.yref_e
    x0 = controller.x0

    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec() - t_init
        for i in range(controller.N+1):
            if(i < controller.N):
                yref[0:2], vel = traj.get_pos_vel(t + i * controller.dT)
                yref[2] = np.arctan2(vel[1], vel[0])
                # yref[3] = np.linalg.norm(vel)
                controller.solver.set(i, "yref", yref)
            else:
                yref_e[0:2], vel = traj.get_pos_vel(t + i * controller.dT)
                yref_e[2] = np.arctan2(vel[1], vel[0])
                # yref_e[3] = np.linalg.norm(vel)
                controller.solver.set(i, "yref", yref_e)

        quaternion = [car_odom.pose.pose.orientation.x, car_odom.pose.pose.orientation.y, car_odom.pose.pose.orientation.z, car_odom.pose.pose.orientation.w]
        theta = QuaternionToTheta(quaternion)       
        x0[0] = car_odom.pose.pose.position.x
        x0[1] = car_odom.pose.pose.position.y
        x0[2] = theta
        x0[3] = car_odom.twist.twist.linear.x
        x0[4] = car_odom.twist.twist.angular.z

        u = controller.solver.solve_for_x0(x0)
        
        cmd_vel = Twist()
        cmd_vel.linear.x = u[0]
        cmd_vel.angular.z = u[1]
        cmdvel_pub.publish(cmd_vel)
        
        print(x0)

        # x1 = controller.solver.get(1, "x")
        # while(x1[2]>np.pi):
        #     x1[2] -= np.pi*2
        # while(x1[2]<-np.pi):
        #     x1[2] += np.pi*2
        # x0 = x1

        rate.sleep()