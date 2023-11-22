#!/home/jiao/python_env/acados/bin/python
import numpy as np
import casadi as ca
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver, AcadosSim, AcadosSimSolver
import rospy
import os
from PolynomialTrajectory import PolyTrajectory
from ArcTrajectory import ArcTrajectory
from visualization_msgs.msg import Marker, MarkerArray
from math import *
import matplotlib.pyplot as plt
import timeit
from matplotlib.animation import FuncAnimation
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf.transformations as tf


def GetArcInfo(arc_order, arc_num, arc_length, arc_param, s, prefix=""):
    l1 = ca.SX.sym(prefix+"l1")
    l2 = ca.SX.sym(prefix+"l2")
    input = ca.SX.sym("s")
    MyGate = ca.Function(prefix+"MyGate", [input, l1, l2], [
        ca.rectangle(1/(l2-l1)*(input-(l1+l2)/2))])
    temp = 0
    for i in range(arc_order+1):
        temp += arc_param[0, i] * s ** i
    px_desired = MyGate(s, 0, arc_length[0])*temp
    temp = 0
    for i in range(arc_order+1):
        temp += arc_param[0, i+arc_order+1] * s ** i
    py_desired = MyGate(s, 0, arc_length[0])*temp

    for i in range(1, arc_num):
        temp = 0
        for j in range(arc_order+1):
            temp += arc_param[i, j] * (s-arc_length[i-1]) ** j
        px_desired += MyGate(s, arc_length[i-1], arc_length[i])*temp
        temp = 0
        for j in range(arc_order+1):
            temp += arc_param[i, j+arc_order+1] * \
                (s-arc_length[i-1]) ** j
        py_desired += MyGate(s, arc_length[i-1], arc_length[i])*temp

    # 补充最后一段到正无穷
    temp = 0
    for j in range(arc_order+1):
        temp += arc_param[arc_num-1, j] * (arc_length[arc_num-1]-arc_length[arc_num-2]) ** j
    px_desired += MyGate(s, arc_length[arc_num-1], 1e6)*temp
    temp = 0
    for j in range(arc_order+1):
        temp += arc_param[arc_num-1, j+arc_order+1] * (arc_length[arc_num-1]-arc_length[arc_num-2]) ** j
    py_desired += MyGate(s, arc_length[arc_num-1], 1e6)*temp

    temp = 0
    for i in range(arc_order):
        temp += arc_param[0, i+1] * s ** i*(i+1)
    diff_x = MyGate(s, 0, arc_length[0])*temp
    temp = 0
    for i in range(arc_order):
        temp += arc_param[0, i+1+arc_order+1] * s ** i*(i+1)
    diff_y = MyGate(s, 0, arc_length[0])*temp

    for i in range(1, arc_num):
        temp = 0
        for j in range(arc_order):
            temp += arc_param[i, j+1] * (s-arc_length[i-1]) ** j*(j+1)
        diff_x += MyGate(s, arc_length[i-1], arc_length[i])*temp
        temp = 0
        for j in range(arc_order):
            temp += arc_param[i, j+1+arc_order+1] * \
                (s-arc_length[i-1]) ** j*(j+1)
        diff_y += MyGate(s, arc_length[i-1], arc_length[i])*temp

    # 补充最后一段到正无穷
    temp = 0
    for j in range(arc_order):
        temp += arc_param[arc_num-1, j+1] * (arc_length[arc_num-1]-arc_length[arc_num-2]) ** j * (j+1)
    diff_x += MyGate(s, arc_length[arc_num-1], 1e6)*temp
    temp = 0
    for j in range(arc_order):
        temp += arc_param[arc_num-1, j+1+arc_order+1] * (arc_length[arc_num-1]-arc_length[arc_num-2]) ** j * (j+1)
    diff_y += MyGate(s, arc_length[arc_num-1], 1e6)*temp

    return px_desired, py_desired, diff_x, diff_y


class Multi_MPCC(object):
    def __init__(self, arc_order, arc_num, name: str = "multi_car", sim_dt = 0.01) -> None:

        self.nx = 0
        self.ny = 0
        self.nu = 0

        self.v_max = 1.0
        self.v_min = -1.0
        self.omega_max = 1.0
        self.omega_min = -1.0

        car0_p = ca.SX.sym('car0_p', 2)
        car0_theta = ca.SX.sym('car0_theta', 1)
        car0_v = ca.SX.sym('car0_v', 1)
        car0_d_v = ca.SX.sym('car0_d_v', 1)
        car0_omega = ca.SX.sym('car0_omega', 1)
        car0_d_omega = ca.SX.sym('car0_d_omega', 1)
        car0_s = ca.SX.sym('car0_s', 1)
        car0_v_s = ca.SX.sym('car0_v_s', 1)
        car0_a_s = ca.SX.sym('car0_a_s', 1)
        car0_state = ca.vertcat(
            car0_p, car0_theta, car0_v, car0_omega, car0_s, car0_v_s)
        car0_u = ca.vertcat(car0_d_v, car0_d_omega, car0_a_s)
        car0_f_expl = ca.vertcat(car0_v*ca.cos(car0_theta),
                                 car0_v*ca.sin(car0_theta),
                                 car0_omega,
                                 car0_d_v,
                                 car0_d_omega,
                                 car0_v_s, 
                                 car0_a_s)
        self.nx += car0_state.size()[0]
        self.nu += car0_u.size()[0]

        car1_p = ca.SX.sym('car1_p', 2)
        car1_theta = ca.SX.sym('car1_theta', 1)
        car1_v = ca.SX.sym('car1_v', 1)
        car1_d_v = ca.SX.sym('car1_d_v', 1)
        car1_omega = ca.SX.sym('car1_omega', 1)
        car1_d_omega = ca.SX.sym('car1_d_omega', 1)
        car1_s = ca.SX.sym('car1_s', 1)
        car1_v_s = ca.SX.sym('car1_v_s', 1)
        car1_a_s = ca.SX.sym('car1_a_s', 1)
        car1_state = ca.vertcat(
            car1_p, car1_theta, car1_v, car1_omega, car1_s, car1_v_s)
        car1_u = ca.vertcat(car1_d_v, car1_d_omega, car1_a_s)
        car1_f_expl = ca.vertcat(car1_v*ca.cos(car1_theta),
                                 car1_v*ca.sin(car1_theta),
                                 car1_omega,
                                 car1_d_v,
                                 car1_d_omega,
                                 car1_v_s, 
                                 car1_a_s)
        self.nx += car1_state.size()[0]
        self.nu += car1_u.size()[0]

        model = AcadosModel()
        model.u = ca.vertcat(car0_u, car1_u)
        model.x = ca.vertcat(car0_state, car1_state)
        model.xdot = ca.SX.sym('x_dot', self.nx, 1)
        model.f_expl_expr = ca.vertcat(car0_f_expl, car1_f_expl)
        model.name = name

        car0_arc_param = ca.SX.sym('car0_arc_param', arc_num, (arc_order+1)*3)

        car0_arc_length = ca.SX.sym('car0_arc_length', arc_num, 1)
        car0_px_desired, car0_py_desired, car0_diff_x, car0_diff_y = GetArcInfo(
            arc_order, arc_num, car0_arc_length, car0_arc_param, car0_s, prefix="car0_")

        car1_arc_param = ca.SX.sym('car1_arc_param', arc_num, (arc_order+1)*3)
        car1_arc_length = ca.SX.sym('car1_arc_length', arc_num, 1)
        car1_px_desired, car1_py_desired, car1_diff_x, car1_diff_y = GetArcInfo(
            arc_order, arc_num, car1_arc_length, car1_arc_param, car1_s, prefix="car1_")

        car0_err = ca.vertcat(car0_px_desired-car0_p[0], car0_py_desired-car0_p[1])
        car0_err_l = ca.dot(car0_err, ca.vertcat(car0_diff_x, car0_diff_y))
        car0_err_l_2 = car0_err_l**2
        car0_err_c_2 = car0_err.T @ car0_err - car0_err_l_2
        car1_err = ca.vertcat(car1_px_desired-car1_p[0], car1_py_desired-car1_p[1])
        car1_err_l = ca.dot(car1_err, ca.vertcat(car1_diff_x, car1_diff_y))
        car1_err_l_2 = car1_err_l**2
        car1_err_c_2 = car1_err.T @ car1_err - car1_err_l_2

        obstacle_pos = ca.SX.sym('obstacle_pos', 2, 1)
        car0_obs_err = (
            car0_p[0:2]-obstacle_pos).T @ (car0_p[0:2]-obstacle_pos)
        car1_obs_err = (
            car1_p[0:2]-obstacle_pos).T @ (car1_p[0:2]-obstacle_pos)
        car0_car1_distance = (
            car0_p[0:2]-car1_p[0:2]).T @ (car0_p[0:2]-car1_p[0:2])
        
        def fun1(x):
            A = 50.0048886846377

            B = 1.33041641228442

            C = 0.295627839582315

            D = -9.8828764268101
            return (A-D)/(1+ca.power(x/C,B))+D
        
        def pos_err(x):
            l1 = ca.SX.sym("l1")
            l2 = ca.SX.sym("l2")
            input = ca.SX.sym("x")
            MyGate = ca.Function("MyGate", [input, l1, l2], [
                                 ca.rectangle(1/(l2-l1)*(input-(l1+l2)/2))])
            return MyGate(x, 0, 1.0**2) * fun1(x)

        w0 = ca.SX.sym('w0', 8, 1)
        w1 = ca.SX.sym('w1', 8, 1)

        model.p = ca.vertcat(ca.reshape(car0_arc_param.T, (-1, 1)), car0_arc_length,
                             ca.reshape(car1_arc_param.T, (-1, 1)), car1_arc_length, w0, w1, obstacle_pos)

        self.N = 20
        self.dt = 0.2
        ocp = AcadosOcp()
        ocp.model = model
        ocp.dims.N = self.N
        ocp.parameter_values = np.zeros((model.p.size()[0], 1))
        ocp.solver_options.tf = self.N*self.dt

        ocp.cost.cost_type = 'EXTERNAL'
        # w0 = [0.5, 0.5, 1.0, 0.05, 1.0]
        # w1 = [0.5, 0.5, 1.0, 0.05, 1.0]

        # w0 = [1.0, 10.0, 0.1, 0.05, 0.1, 0.01, 0.005, 0.005]
        # w1 = [1.0, 10.0, 0.1, 0.05, 0.1, 0.01, 0.005, 0.005]
        ocp.model.cost_expr_ext_cost = \
            w0[0]*car0_err_l_2 + w1[0] * car1_err_l_2 \
            + w0[1]*car0_err_c_2 + w1[1]*car1_err_c_2 \
            + w0[2]*pos_err(car0_obs_err) + w1[2]*pos_err(car1_obs_err)\
            - w0[3]*car0_v_s - w1[3]*car1_v_s \
            + w0[4]*pos_err(car0_car1_distance)\
            + w0[5]*car0_d_v**2 + w1[5]*car1_d_v**2 \
            + w0[6]*car0_d_omega**2 + w1[6]*car1_d_omega**2 \
            + w0[7]*car0_a_s**2 + w1[7]*car1_a_s**2

        self.print_fun0 = ca.Function('print_fun0', [model.x, model.u, model.p], 
                                      [w0[0]*car0_err_l_2, w0[1]*car0_err_c_2, w0[2]*pos_err(car0_obs_err), - w0[3]*car0_v_s,  
                                     w0[4]*pos_err(car0_car1_distance), w0[5]*car0_d_v**2, w0[6]*car0_d_omega**2, w0[7]*car0_a_s**2])
        self.print_fun1 = ca.Function('print_fun1', [model.x, model.u, model.p], 
                                      [w1[0]*car1_err_l_2, w1[1]*car1_err_c_2, w1[2]*pos_err(car1_obs_err), - w1[3]*car1_v_s,  
                                       w1[4]*pos_err(car0_car1_distance), w1[5]*car1_d_v**2, w1[6]*car1_d_omega**2, w1[7]*car1_a_s**2])

        self.get_param = ca.Function('get_param', [model.x, model.u, model.p], [car0_diff_x**2+car0_diff_y**2])

        # car0_px, car0_py, car0_theta, 'car0_v, car0_omega, car0_s, car0_v_s'
        ocp.constraints.idxbx = np.array([3, 4, 5, 6, 
                                          10, 11, 12, 13])
        ocp.constraints.lbx = np.array([self.v_min, self.omega_min, 0, 0,
                                        self.v_min, self.omega_min, 0, 0,])
        self.ubx = np.array([self.v_max, self.omega_max, 0, self.v_max,
                            self.v_max, self.omega_max, 0, self.v_max])
        ocp.constraints.ubx = self.ubx

        ocp.constraints.idxbx_e = np.array([3, 4, 5, 6, 
                                          10, 11, 12, 13])
        ocp.constraints.lbx_e = np.array([self.v_min, self.omega_min, 0, 0,
                                        self.v_min, self.omega_min, 0, 0,])
        ocp.constraints.ubx_e = np.array([self.v_max, self.omega_max, 0, self.v_max,
                                        self.v_max, self.omega_max, 0, self.v_max])

        self.x0 = np.array([-3, 0, 0, 0, 0, 1e-6, 0, 
                            3, 0, np.pi, 0, 0, 1e-6, 0])
        ocp.constraints.x0 = self.x0

        # car0_d_v, car0_d_omega, car0_a_s
        ocp.constraints.idxbu = np.arange(self.nu)
        ocp.constraints.lbu = np.array([-3.0, -3.0, -3.0,
                                        -3.0, -3.0, -3.0])
        ocp.constraints.ubu = np.array([3.0, 3.0, 3.0,
                                        3.0, 3.0, 3.0])

        ocp.solver_options.hessian_approx = 'EXACT'

        ocp.solver_options.regularize_method = 'MIRROR'  # MIRROR,CONVEXIFY
        # 'PARTIAL_CONDENSING_HPIPM''FULL_CONDENSING_HPIPM'
        ocp.solver_options.qp_solver = 'FULL_CONDENSING_HPIPM'
        ocp.solver_options.integrator_type = 'ERK'
        ocp.solver_options.nlp_solver_type = 'SQP_RTI'  # SQP_RTI, SQP
        ocp.solver_options.nlp_solver_max_iter = 100
        ocp.solver_options.qp_solver_iter_max = 50
        ocp.code_export_directory = os.path.dirname(os.path.realpath(__file__))+"/c_generated_code/ocp_"+model.name
        json_file = os.path.dirname(os.path.realpath(__file__))+"/json_files/ocp_"+model.name+'_acados_ocp.json'
        self.solver = AcadosOcpSolver(ocp, json_file=json_file)

        self.sim_dt = sim_dt
        sim = AcadosSim()
        sim.model = model
        sim.parameter_values = np.zeros((model.p.size()[0], 1))
        sim.solver_options.T = self.sim_dt
        sim.solver_options.integrator_type = "ERK"
        sim.solver_options.num_stages = 3
        sim.solver_options.num_steps = 3
        sim.solver_options.newton_iter = 3  # for implicit integrator
        sim.solver_options.collocation_type = "GAUSS_RADAU_IIA"
        sim.code_export_directory = os.path.dirname(os.path.realpath(__file__))+"/c_generated_code/sim_"+model.name
        json_file = os.path.dirname(os.path.realpath(__file__))+"/json_files/sim_"+model.name+'_acados_ocp.json'
        self.integrator = AcadosSimSolver(sim, json_file=json_file)


def show_arc_traj(points):
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

    id = 0
    for m in marker_array.markers:
        m.id = id
        id += 1
    return marker_array

def save_param():
    point_car0 = np.array([[-3, 0, 0], [0, 0, 0], [0, -3, 0]]).T
    point_car1 = np.array([[3, 0, 0], [0, 0, 0], [0, 3, 0]]).T

    order = 5
    k = 3
    tf = 2

    trax_car0 = PolyTrajectory(
        N=point_car0[0].shape[0], point=point_car0[0], order=order, k=k, tf=tf, name="trax_car0")
    trax_car0.solve()
    tray_car0 = PolyTrajectory(
        N=point_car0[1].shape[0], point=point_car0[1], order=order, k=k, tf=tf, name="tray_car0")
    tray_car0.solve()
    traz_car0 = PolyTrajectory(
        N=point_car0[2].shape[0], point=point_car0[2], order=order, k=k, tf=tf, name="traz_car0")
    traz_car0.solve()

    trax_car1 = PolyTrajectory(
        N=point_car1[0].shape[0], point=point_car1[0], order=order, k=k, tf=tf, name="trax_car1")
    trax_car1.solve()
    tray_car1 = PolyTrajectory(
        N=point_car1[1].shape[0], point=point_car1[1], order=order, k=k, tf=tf, name="tray_car1")
    tray_car1.solve()
    traz_car1 = PolyTrajectory(
        N=point_car1[2].shape[0], point=point_car1[2], order=order, k=k, tf=tf, name="traz_car1")
    traz_car1.solve()

    arc_order = 3
    arc_k = 2
    sample_point_num = 5

    param_car0 = np.vstack((trax_car0.param, tray_car0.param, traz_car0.param))
    arc_car0 = ArcTrajectory(order=order, param=param_car0,
                             N=trax_car0.N, tf=tf, m=sample_point_num, prefix="car0_")
    arc_car0.generate_points()
    arc_car0.solve(order=arc_order, k=arc_k)
    param_car0, lengths_car0 = arc_car0.get_param()
    points_car0 = np.array(arc_car0.points)

    param_car1 = np.vstack((trax_car1.param, tray_car1.param, traz_car1.param))
    arc_car1 = ArcTrajectory(order=order, param=param_car1,
                             N=trax_car1.N, tf=tf, m=sample_point_num, prefix="car1_")
    arc_car1.generate_points()
    arc_car1.solve(order=arc_order, k=arc_k)
    param_car1, lengths_car1 = arc_car1.get_param()
    points_car1 = np.array(arc_car1.points)

    marker_array_car0 = show_arc_traj(points_car0)
    marker_array_car1 = show_arc_traj(points_car1)

    np.save(os.path.dirname(os.path.realpath(__file__)) +
            "/lengths_car0.npy", lengths_car0)
    np.save(os.path.dirname(os.path.realpath(__file__)) +
            "/lengths_car1.npy", lengths_car1)
    np.save(os.path.dirname(os.path.realpath(__file__)) +
            "/param_car0.npy", param_car0)
    np.save(os.path.dirname(os.path.realpath(__file__)) +
            "/param_car1.npy", param_car1)
    pub_car0 = rospy.Publisher('car0', MarkerArray, queue_size=10)
    pub_car1 = rospy.Publisher('car1', MarkerArray, queue_size=10)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        pub_car0.publish(marker_array_car0)
        pub_car1.publish(marker_array_car1)
        rate.sleep()
    exit()

def load_param():
    arc_order = 3
    lengths_car0 = np.load(os.path.dirname(
        os.path.realpath(__file__))+"/lengths_car0.npy")
    lengths_car1 = np.load(os.path.dirname(
        os.path.realpath(__file__))+"/lengths_car1.npy")
    param_car0 = np.load(os.path.dirname(
        os.path.realpath(__file__)) + "/param_car0.npy")
    param_car1 = np.load(os.path.dirname(
        os.path.realpath(__file__)) + "/param_car1.npy")  
    return arc_order, lengths_car0, lengths_car1, param_car0, param_car1

init_flag = [False, False, False]
obs_odom = Odometry()
def ObsOdometryCallback(data):
    global init_flag
    init_flag[0] = True
    global obs_odom
    obs_odom = data

car0_odom = Odometry()
def Car0OdometryCallback(data):
    global init_flag
    init_flag[1] = True
    global car0_odom
    car0_odom = data

car1_odom = Odometry()
def Car1OdometryCallback(data):
    global init_flag
    init_flag[2] = True
    global car1_odom
    car1_odom = data

if __name__ == '__main__':
    rospy.init_node('mpcc_node', anonymous=True)

    json_files_path = os.path.dirname(os.path.realpath(__file__))+"/json_files"
    if not (os.path.exists(json_files_path)):
        os.makedirs(json_files_path)

    # save_param()

    arc_order, lengths_car0, lengths_car1, param_car0, param_car1 = load_param()
    w0 = np.array([10.0, 10.0, 0.1, 0.8, 0.1, 0.1, 0.1, 0.1])
    w1 = np.array([10.0, 10.0, 0.1, 0.8, 0.1, 0.1, 0.1, 0.1])
    p = np.vstack((np.reshape(param_car0, (-1, 1)), np.reshape(lengths_car0, (-1, 1)),
                  np.reshape(param_car1, (-1, 1)), np.reshape(lengths_car1, (-1, 1)), 
                  np.reshape(w0, (-1, 1)), np.reshape(w1, (-1, 1)), np.array([[0.0], [0.0]])))
    

    ts = 0.02
    mpcc = Multi_MPCC(arc_order=arc_order,
                      arc_num=lengths_car0.shape[0], name="multi_car", sim_dt=ts)
    rospy.Subscriber("obs/odom", Odometry, ObsOdometryCallback, queue_size=1)
    rospy.Subscriber("car0/odom", Odometry, Car0OdometryCallback, queue_size=1)
    rospy.Subscriber("car1/odom", Odometry, Car1OdometryCallback, queue_size=1)
    car0_cmdvel_pub = rospy.Publisher('car0/cmd_vel', Twist, queue_size=1)
    car1_cmdvel_pub = rospy.Publisher('car1/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(int(1/ts))
    while(init_flag[0] == False or init_flag[1] == False or init_flag[2] == False):
        rate.sleep()

    obs_omega = 1.0
    obs_radius = 1.0

    x0 = mpcc.x0

    ubx = mpcc.ubx
    ubx[2] = lengths_car0[-1] + 1e6
    ubx[6] = lengths_car1[-1] + 1e6
    for i in range(1, mpcc.N+1):
        mpcc.solver.constraints_set(i, "ubx", ubx)
    
    car0_state = []
    car0_loss = []
    car1_state = []
    car1_loss = []
    obs = []
    for k in range(600):
        start = timeit.default_timer()

        car0_state.append(x0[0:7].copy())
        car1_state.append(x0[7:14].copy())

        # car0_px, car0_py, car0_theta, 'car0_v, car0_omega, car0_s, car0_v_s'
        x0[0] = car0_odom.pose.pose.position.x
        x0[1] = car0_odom.pose.pose.position.y
        x0[2] = tf.euler_from_quaternion(
            [car0_odom.pose.pose.orientation.x, car0_odom.pose.pose.orientation.y, car0_odom.pose.pose.orientation.z, car0_odom.pose.pose.orientation.w]
            , axes='sxyz')[2]
        x0[3] = car0_odom.twist.twist.linear.x
        x0[4] = car0_odom.twist.twist.angular.z
        x0[7] = car1_odom.pose.pose.position.x
        x0[8] = car1_odom.pose.pose.position.y
        x0[9] = tf.euler_from_quaternion(
            [car1_odom.pose.pose.orientation.x, car1_odom.pose.pose.orientation.y, car1_odom.pose.pose.orientation.z, car1_odom.pose.pose.orientation.w]
            , axes='sxyz')[2]
        x0[10] = car1_odom.twist.twist.linear.x
        x0[11] = car1_odom.twist.twist.angular.z

        obs_theta = np.arctan2(obs_odom.pose.pose.position.y, obs_odom.pose.pose.position.x)
        for i in range(0, mpcc.N+1):
            t = i*mpcc.dt
            p[-2:] = np.array([[obs_radius*cos(obs_omega*t+obs_theta)], [obs_radius*sin(obs_omega*t+obs_theta)]])
            if(i == 0):
                obs.append(p[-2:].copy())                
            mpcc.solver.set(i, "p", p)

        u = None
        for j in range(8):
            u = mpcc.solver.solve_for_x0(x0)
            residuals = mpcc.solver.get_residuals()
            if max(residuals)<1e-6:
                break
        
        x_sim = mpcc.integrator.simulate(x=x0, u=u, p=p)
        x0[5] = x_sim[5]
        x0[6] = x_sim[6]
        x0[12] = x_sim[12]
        x0[13] = x_sim[13]

        car0_cmdvel = Twist()
        car0_cmdvel.linear.x = x_sim[3]
        car0_cmdvel.angular.z = x_sim[4]
        car0_cmdvel_pub.publish(car0_cmdvel)

        car1_cmdvel = Twist()
        car1_cmdvel.linear.x = x_sim[10]
        car1_cmdvel.angular.z = x_sim[11]
        car1_cmdvel_pub.publish(car1_cmdvel)

        loss = []
        for i in range(0, mpcc.N):
            x_temp = mpcc.solver.get(i, 'x')
            u_temp = mpcc.solver.get(i, 'u')
            loss1 = np.array([ca.DM.toarray(elem) for elem in mpcc.print_fun0(x_temp, u_temp, p)]).flatten()
            loss2 = np.array([ca.DM.toarray(elem) for elem in mpcc.print_fun1(x_temp, u_temp, p)]).flatten()
            loss.append(np.hstack([loss1, loss2]))

        loss = np.array(loss)
        loss = np.sum(loss, axis=0)
        car0_loss.append(loss[0:8].copy())
        car1_loss.append(loss[8:16].copy())
        def fun2(x):
            A = 10.4636917174441

            B = 0.831338857699215

            C = 0.0402405243617193

            D = -0.00572514396595922

            return (A-D)/(1+np.power(x/C,B))+D
        
        w0[0] = w0[1] = fun2(loss[2]+loss[4])
        if(w0[0]<0.1):
            w0[0] = 0.1
        if(w0[1]<0.1):
            w0[1] = 0.1

        w1[0] = w1[1] = fun2(loss[10]+loss[12])
        if(w1[0]<0.1):
            w1[0] = 0.1
        if(w1[1]<0.1):
            w1[1] = 0.1

        p = np.vstack((np.reshape(param_car0, (-1, 1)), np.reshape(lengths_car0, (-1, 1)),
                np.reshape(param_car1, (-1, 1)), np.reshape(lengths_car1, (-1, 1)), 
                np.reshape(w0, (-1, 1)), np.reshape(w1, (-1, 1)), np.array([[0.0], [0.0]])))

        time_record = timeit.default_timer() - start
        print(k, w0[0:2], w1[0:2], j, "estimation time is {}".format(time_record))
        rate.sleep()

    car0_state = np.array(car0_state)
    car1_state = np.array(car1_state)
    car0_loss = np.array(car0_loss)
    car1_loss = np.array(car1_loss)
    obs = np.array(obs).reshape((-1, 2))

    distance_ca0_car1 = np.sqrt((car0_state[:, 0]-car1_state[:, 0])**2+(car0_state[:, 1]-car1_state[:, 1])**2)
    distance_car0_obs = np.sqrt((car0_state[:, 0]-obs[:, 0])**2+(car0_state[:, 1]-obs[:, 1])**2)
    distance_car1_obs = np.sqrt((car1_state[:, 0]-obs[:, 0])**2+(car1_state[:, 1]-obs[:, 1])**2)

    fig, axs = plt.subplots(2, 9, figsize=(18, 4))
    for i in range(7):
        axs[0, i].plot(car0_state[:, i])
        axs[1, i].plot(car1_state[:, i])
    axs[0, 7].plot(distance_car0_obs)
    axs[1, 7].plot(distance_car1_obs)
    axs[0, 8].plot(distance_ca0_car1)
    axs[1, 8].plot(distance_ca0_car1)

    fig, axs = plt.subplots(2, 8, figsize=(16, 4))
    for i in range(8):
        axs[0, i].plot(car0_loss[:, i])
        axs[1, i].plot(car1_loss[:, i])

    fig_combined, ax_combined = plt.subplots(figsize=(10, 8))
    ax_combined.set_xlim(-5, 5)
    ax_combined.set_ylim(-5, 5)
    ax_combined.legend()
    car0_line, = ax_combined.plot([], [], label='Car 0 State')
    car1_line, = ax_combined.plot([], [], label='Car 1 State')
    obs_line, = ax_combined.plot([], [], label='Obstacle')
    def update(frame):
        car0_line.set_data(car0_state[:frame, 0], car0_state[:frame, 1])
        car1_line.set_data(car1_state[:frame, 0], car1_state[:frame, 1])
        obs_line.set_data(obs[:frame, 0], obs[:frame, 1])
        return car0_line, car1_line, obs_line
    ani = FuncAnimation(fig_combined, update, frames=len(car0_state), interval=50, blit=True)


    plt.tight_layout()
    plt.show()