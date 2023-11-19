from PolynomialTrajectory import PolyTrajectory
from acados_template import AcadosSim, AcadosSimSolver, AcadosModel, AcadosOcp, AcadosOcpSolver

# import numpy as np
from casadi import *
import timeit
import math
import scipy.linalg

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import rospy
from matplotlib import pyplot as plt
from visualization_msgs.msg import MarkerArray, Marker

import sys
# python强制打印整个数组
np.set_printoptions(threshold=sys.maxsize)


class MyIntegrator(object):
    def __init__(self, order=3, param=None, tf=1, name=None):
        self.xparam = param[0]
        self.yparam = param[1]
        self.zparam = param[2]

        model = AcadosModel()
        t = SX.sym("t")
        x = SX.sym("x")
        y = SX.sym("y")
        z = SX.sym("z")
        l = SX.sym("l")

        x_diff = 0
        y_diff = 0
        z_diff = 0
        for i in range(order):
            x_diff += self.xparam[i+1]*t**i*(i+1)
            y_diff += self.yparam[i+1]*t**i*(i+1)
            z_diff += self.zparam[i+1]*t**i*(i+1)

        model.x = vertcat(t, x, y, z, l)
        model.f_expl_expr = vertcat(
            1,
            x_diff,
            y_diff,
            z_diff,
            sqrt(x_diff**2+y_diff**2+z_diff**2)
        )

        x_dot = SX.sym("x_dot", 5)
        model.xdot = x_dot
        model.u = SX.sym('u', 0, 0)  # [] / None doesnt work
        model.p = []
        model.name = name

        sim = AcadosSim()
        sim.model = model

        sim.solver_options.T = tf
        sim.solver_options.integrator_type = "ERK"
        sim.solver_options.num_stages = 3
        sim.solver_options.num_steps = 3
        sim.solver_options.newton_iter = 3  # for implicit integrator
        sim.solver_options.collocation_type = "GAUSS_RADAU_IIA"
        sim.code_export_directory(os.path.dirname(os.path.realpath(__file__))+"/")
        self.acados_integrator = AcadosSimSolver(sim)
        self.acados_integrator.set("x", np.array(
            [0, self.xparam[0], self.yparam[0], self.zparam[0], 0]))

    def integrate(self, t):
        self.acados_integrator.set("T", t)
        status = self.acados_integrator.solve()
        if (status != 0):
            print("solve_error!")
            exit()
        return self.acados_integrator.get("x")


class ArcPolynomial(object):
    def __init__(self, order=3, N=4, k=2, l_=0.1, point=None, name=None):
        self.order = order
        self.N = N - 1
        self.l_ = l_

        ocp = AcadosOcp()
        model = AcadosModel()
        sigma = SX.sym("sigma", (self.order+1)*3*self.N, 1)
        model.x = sigma
        model.disc_dyn_expr = sigma
        model.u = SX.sym('u', 0, 0)  # [] / None doesnt work
        model.p = []

        model.name = name
        ocp.model = model

        ocp.dims.N = 1
        ocp.solver_options.tf = 1

        self.b = np.zeros((self.order-k+1))
        for i in range((self.order-k+1)):
            self.b[i] = math.factorial(k+i)/math.factorial(i)

        Q_temp = np.zeros((self.order-k+1, self.order-k+1))
        for i in range((self.order-k+1)):
            for j in range((self.order-k+1)):
                Q_temp[i][j] = self.b[i]*self.b[j]*self.l_**(i+j+1)/(i+j+1)
        Q_temp = scipy.linalg.block_diag(np.zeros((k, k)), Q_temp)
        Q_temp = scipy.linalg.block_diag(Q_temp, Q_temp, Q_temp)
        Q = Q_temp

        ocp.cost.cost_type = 'EXTERNAL'
        for i in range(self.N-1):
            Q = scipy.linalg.block_diag(Q, Q_temp)
        ocp.model.cost_expr_ext_cost = sigma.T @ Q @ sigma

        self.A = np.zeros((0, 0))
        position_s0 = np.zeros((3, (self.order+1)*3))
        position_s0[0][0] = 1
        position_s0[1][self.order+1] = 1
        position_s0[2][(self.order+1)*2] = 1

        position_sl = np.zeros((3, (self.order+1)*3))
        for i in range(3):
            for j in range(self.order+1):
                position_sl[i][j+(self.order+1)*i] = self.l_**j

        diff_t0 = np.zeros((1*3, (self.order+1)*3))
        for i in range(3):
            diff_t0[i][1+i*(self.order+1)] = 1

        diff_tf = np.zeros((1*3, (self.order+1)*3))
        for i in range(3):
            for j in range(1):
                temp = 0
                for k in range(j+1, self.order+1):
                    diff_tf[j+i][k+i*(self.order+1)] = math.factorial(
                        k)/math.factorial(k-j-1)*self.l_**temp
                    temp += 1

        for i in range(self.N):
            zero1 = np.zeros((3, i*(self.order+1)*3))
            zero2 = np.zeros((3, (self.N-1-i)*(self.order+1)*3))
            if (i == 0):
                self.A = np.hstack((zero1, position_s0, zero2))
                self.A = np.vstack(
                    (self.A, np.hstack((zero1, position_sl, zero2))))
            else:
                self.A = np.vstack(
                    (self.A, np.hstack((zero1, position_s0, zero2))))
                self.A = np.vstack(
                    (self.A, np.hstack((zero1, position_sl, zero2))))

        for i in range(self.N-1):
            zero1 = np.zeros((i*(self.order+1)*3))
            zero3 = np.zeros(((self.N-2-i)*(self.order+1)*3))
            for j in range(1*3):
                self.A = np.vstack(
                    (self.A, np.hstack((zero1, diff_tf[j], -diff_t0[j], zero3))))

        self.b = np.zeros(self.A.shape[0])
        self.b[0] = point[0][0]
        self.b[1] = point[0][1]
        self.b[2] = point[0][2]
        for i in range(1, self.N):
            for j in range(3):
                self.b[(2*i-1)*3+j] = point[i][j]
                self.b[(2*i)*3+j] = point[i][j]
        self.b[(self.N*2-1)*3] = point[self.N][0]
        self.b[(self.N*2-1)*3+1] = point[self.N][1]
        self.b[(self.N*2-1)*3+2] = point[self.N][2]

        x_diff = sigma[1]
        y_diff = sigma[1+(self.order+1)]
        z_diff = sigma[1+(self.order+1)*2]
        constraint1 = (x_diff**2 + y_diff ** 2 + z_diff**2)

        x_diff = 0
        y_diff = 0
        z_diff = 0
        for j in range(self.order):
            x_diff += (j+1)*sigma[j+1]*self.l_**j
            y_diff += (j+1)*sigma[j+1+(self.order+1)]*self.l_**j
            z_diff += (j+1)*sigma[j+1+(self.order+1)*2]*self.l_**j
        constraint2 = (x_diff**2 + y_diff ** 2 + z_diff**2)
        for i in range(1, self.N):
            x_diff = 0
            y_diff = 0
            z_diff = 0
            for j in range(self.order):
                x_diff += (j+1)*sigma[i*(self.order+1)*3+j+1]*self.l_**j
                y_diff += (j+1)*sigma[i*(self.order+1)
                                      * 3+j+1+(self.order+1)]*self.l_**j
                z_diff += (j+1)*sigma[i*(self.order+1)
                                      * 3+j+1+(self.order+1)*2]*self.l_**j
            constraint2 = vertcat(
                constraint2, (x_diff**2 + y_diff ** 2 + z_diff**2))
        
        constraint2 = vertcat(constraint2,constraint1)

        ocp.model.con_h_expr = vertcat(self.A @ sigma, constraint2)

        h = np.hstack((self.b,np.ones((self.N+1))))
        ocp.constraints.lh = h
        ocp.constraints.uh = h

        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM' 
        ocp.solver_options.hessian_approx = 'EXACT'
        ocp.solver_options.integrator_type = 'DISCRETE'

        ocp.solver_options.nlp_solver_type = 'SQP' # SQP_RTI, SQP
        ocp.solver_options.levenberg_marquardt = 1e-1
        ocp.solver_options.qp_solver_iter_max = 5000
        ocp.solver_options.regularize_method = 'MIRROR'
        ocp.solver_options.eps_sufficient_descent = 1e-1
        ocp.solver_options.qp_tol = 1e-6
        ocp.solver_options.nlp_solver_max_iter = 5000

        ocp.code_export_directory(os.path.dirname(os.path.realpath(__file__))+"/"+model.name)
        json_file = os.path.dirname(os.path.realpath(__file__))+model.name+'_acados_ocp.json'
        self.solver = AcadosOcpSolver(ocp, json_file=json_file)
        self.solver.set(0,"x",np.ones(((self.order+1)*3*self.N)))


    def solve(self):
        start = timeit.default_timer()
        status = self.solver.solve()
        time_record = timeit.default_timer() - start
        print("estimation time is {}".format(time_record))
        if (status != 0):
            print("solve_error!")
            exit()
        self.solver.print_statistics()
        self.param = self.solver.get(0, 'x')



class ArcTrajectory(object):
    def __init__(self, order=3, param=None, N=5, tf=1, m=10):
        self.order = order

        self.xparam = param[0]
        self.yparam = param[1]
        self.zparam = param[2]

        self.N = N
        self.tf = tf
        self.m = m
        self.len_integrate = []
        for i in range(self.N):
            temp_param = np.vstack((self.xparam[i*(self.order+1):(i+1)*(self.order+1)], self.yparam[i*(
                self.order+1):(i+1)*(self.order+1)], self.zparam[i*(self.order+1):(i+1)*(self.order+1)]))
            name = "len_integrate_" + str(i)
            temp_intgrator = MyIntegrator(
                order=order, param=temp_param, tf=tf, name=name)
            self.len_integrate.append(temp_intgrator)

        self.L = []
        for i in range(self.N):
            self.L.append(self.len_integrate[i].integrate(tf)[4])
        self.L = np.array(self.L)
        self.l_ = self.L/self.m

    def generate_points(self):
        toleration = 1e-5
        self.points = []
        delta_tf = self.tf/self.m
        for i in range(self.N):
            self.points.append(self.len_integrate[i].integrate(0)[1:4])
            tl = 0
            th = delta_tf
            tmid = delta_tf
            for j in range(self.m-1):
                len_now = self.len_integrate[i].integrate(th)[4]
                len_target = (j+1)*self.l_[i]
                while (len_now < len_target):
                    th += delta_tf
                    len_now = self.len_integrate[i].integrate(th)[4]
                tmid = (tl+th)/2.0
                len_now = self.len_integrate[i].integrate(tmid)[4]
                while (math.fabs(len_now-len_target) > toleration):
                    if (len_now > len_target):
                        th = tmid
                        tmid = (tl+th)/2.0
                        len_now = self.len_integrate[i].integrate(tmid)[4]
                    elif (len_now < len_target):
                        tl = tmid
                        tmid = (tl+th)/2.0
                        len_now = self.len_integrate[i].integrate(tmid)[4]
                self.points.append(self.len_integrate[i].integrate(tmid)[1:4])
        self.points.append(self.len_integrate[self.N-1].integrate(self.tf)[1:4])

    def solve(self, order, k):
        self.arc_order = order
        self.solver = []
        for i in range(self.N):
            name = "Arcsolver_" + str(i)
            point_temp = self.points[i*self.m:(i+1)*self.m+1]
            self.solver.append(ArcPolynomial(order=order, N=self.m+1, k=k, l_=self.l_[
                               i], point=point_temp, name=name))
            self.solver[i].solve()

    def get_trajectory(self, ds):
        y = np.zeros((0,3))
        for i in range(self.N):
            sample_num = int(self.l_[i]/ds)
            s = np.linspace(0,self.l_[i]-ds,sample_num)
            for j in range(self.m):
                temp = np.zeros((3,s.shape[0]))
                for k in range(self.arc_order+1):
                    temp[0]+=self.solver[i].param[k+j*(self.arc_order+1)*3]*s**k
                    temp[1]+=self.solver[i].param[k+j*(self.arc_order+1)*3+(self.arc_order+1)]*s**k
                    temp[2]+=self.solver[i].param[k+j*(self.arc_order+1)*3+(self.arc_order+1)*2]*s**k
                temp = temp.T
                y = np.vstack((y,temp))
        totalL = np.sum(self.L)
        s = np.linspace(0,totalL,y.shape[0])
        return s,y
    
    def get_param(self):
        param = self.solver[0].param
        for i in range(1,self.N):
            param = np.vstack((param,self.solver[i].param))
        param = np.reshape(param,(-1,(self.arc_order+1)*3))

        
        l = []
        temp = 0
        for i in range(self.N):
            for j in range(self.m):
                temp+=self.l_[i]
                l.append(temp)
        l = np.array(l)

        return param,l


    def test(self):
        # for i in range(self.N):
        #     for j in range(self.m):
        #         temp_param = np.vstack((self.solver[i].param[j*(self.arc_order+1)*3:j*(self.arc_order+1)*3+(self.arc_order+1)],
        #                                 self.solver[i].param[j*(self.arc_order+1)*3+(self.arc_order+1):j*(self.arc_order+1)*3+(self.arc_order+1)*2],
        #                                 self.solver[i].param[j*(self.arc_order+1)*3+(self.arc_order+1)*2:j*(self.arc_order+1)*3+(self.arc_order+1)*3]))
        #         temp_intgrator = MyIntegrator(order=self.arc_order, param=temp_param, tf=tf, name="test")
        #         temp = np.linspace(0,self.l_[i],1000)
        #         y = []
        #         for k in range(1000):
        #             y.append(temp_intgrator.integrate(temp[k])[4])
        #         plt.subplot(211)
        #         plt.plot(temp,y)
        #         plt.subplot(212)
        #         plt.plot(temp,temp-y)
        #         plt.show()
        ds = 0.01
        for i in range(self.N):
            sample_num = int(self.l_[i]/ds)
            s = np.linspace(0,self.l_[i]-ds,sample_num)
            for j in range(self.m):
                temp = np.zeros((3,s.shape[0]))
                for k in range(0,self.arc_order):
                    temp[0]+=self.solver[i].param[k+1+j*(self.arc_order+1)*3]*s**k*(k+1)
                    temp[1]+=self.solver[i].param[k+1+j*(self.arc_order+1)*3+(self.arc_order+1)]*s**k*(k+1)
                    temp[2]+=self.solver[i].param[k+1+j*(self.arc_order+1)*3+(self.arc_order+1)*2]*s**k*(k+1)
                y = temp[0]**2+temp[1]**2+temp[2]**2
                plt.plot(s,y)
                plt.show()
if __name__ == "__main__":

    point = np.array([[0,0,0.1],[0,10,3],[-10,0,3],[0,-10,3],[0,10,3],[0,-10,3]])
    point = point.T
    order = 7
    tf = 10
    ts = 0.02

    trax = PolyTrajectory(
        N=point[0].shape[0], point=point[0], order=order, k=1, tf=tf, name="trax")
    trax.solve()
    t, x = trax.get_trajectory(Ts=ts)

    tray = PolyTrajectory(
        N=point[1].shape[0], point=point[1], order=order, k=1, tf=tf, name="tray")
    tray.solve()
    t, y = tray.get_trajectory(Ts=ts)

    traz = PolyTrajectory(
        N=point[2].shape[0], point=point[2], order=order, k=1, tf=tf, name="traz")
    traz.solve()
    t, z = traz.get_trajectory(Ts=ts)

    param = np.vstack((trax.param, tray.param, traz.param))

    arc = ArcTrajectory(order=order, param=param, N=trax.N, tf=tf,m=7)
    arc.generate_points()
    arc.solve(order=3,k=2)
    param,l = arc.get_param()
    s,arctrajectory = arc.get_trajectory(ds=0.01)
    points = np.array(arc.points)


    rospy.init_node("ArcTrajectory_node", anonymous=True)

    marker_array = MarkerArray()

    for i in range(points.shape[0]):
        # marker.header.stamp = rospy.Time.now()
        marker = Marker()

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        # marker.lifetime = rospy.Duration()
        marker.header.frame_id = "world"
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        marker.pose.position.x = points[i][0]
        marker.pose.position.y = points[i][1]
        marker.pose.position.z = points[i][2]
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        marker_array.markers.append(marker)

    path2 = Path()
    path2.header.frame_id = "world"
    path2_pub = rospy.Publisher("path2", Path, queue_size=1)

    for i in range(x.shape[0]):
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = x[i]
        pose.pose.position.y = y[i]
        pose.pose.position.z = z[i]
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 1
        path2.poses.append(pose)

    # for i in range(int(x.shape[0]/2)):
    #     # marker.header.stamp = rospy.Time.now()
    #     marker = Marker()

    #     marker.type = Marker.SPHERE
    #     marker.action = Marker.ADD
    #     # marker.lifetime = rospy.Duration()
    #     marker.header.frame_id = "world"
    #     marker.scale.x = 0.05
    #     marker.scale.y = 0.05
    #     marker.scale.z = 0.05
    #     marker.color.a = 1.0
    #     marker.color.r = 1.0
    #     marker.color.g = 0.0
    #     marker.color.b = 0.0

    #     marker.pose.position.x = x[2*i]
    #     marker.pose.position.y = y[2*i]
    #     marker.pose.position.z = z[2*i]
    #     marker.pose.orientation.x = 0
    #     marker.pose.orientation.y = 0
    #     marker.pose.orientation.z = 0
    #     marker.pose.orientation.w = 1
    #     marker_array.markers.append(marker)

    id = 0
    for m in marker_array.markers:
        m.id = id
        id += 1

    rate = rospy.Rate(1/ts)
    path_pub = rospy.Publisher("ArcPath", Path, queue_size=15)
    vis_pub = rospy.Publisher("desire_marker_array", MarkerArray)
    path1 = Path()

    # for i in range(t.shape[0]):
    #     path1.header.stamp = rospy.Time.now()
    #     path1.header.frame_id = "world"
    #     current_point = PoseStamped()
    #     current_point.header.frame_id = "world"
    #     current_point.pose.position.x = x[i]
    #     current_point.pose.position.y = y[i]
    #     current_point.pose.position.z = z[i]

    #     current_point.pose.orientation.x = 0
    #     current_point.pose.orientation.y = 0
    #     current_point.pose.orientation.z = 0
    #     current_point.pose.orientation.w = 1

    #     path1.poses.append(current_point)
    #     vis_pub.publish(marker_array)
    #     path_pub.publish(path1)
    #     rate.sleep()

    for i in range(arctrajectory.shape[0]):

        path1.header.frame_id = "world"
        current_point = PoseStamped()
        current_point.header.frame_id = "world"
        current_point.pose.position.x = arctrajectory[i][0]
        current_point.pose.position.y = arctrajectory[i][1]
        current_point.pose.position.z = arctrajectory[i][2]
        
        current_point.pose.orientation.x = 0
        current_point.pose.orientation.y = 0
        current_point.pose.orientation.z = 0
        current_point.pose.orientation.w = 1

        path1.poses.append(current_point)
    while(1):
        path1.header.stamp = rospy.Time.now()
        path2.header.stamp = rospy.Time.now()
        vis_pub.publish(marker_array)
        path_pub.publish(path1)
        path2_pub.publish(path2)
        rate.sleep()
