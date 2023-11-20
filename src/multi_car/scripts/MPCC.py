#!/home/jiao/python_env/acados/bin/python
import numpy as np
import casadi as ca
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
import rospy
import os
from PolynomialTrajectory import PolyTrajectory
from ArcTrajectory import ArcTrajectory
from visualization_msgs.msg import Marker, MarkerArray
from math import *
import matplotlib.pyplot as plt


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
    return px_desired, py_desired, diff_x, diff_y


class Multi_MPCC(object):
    def __init__(self, arc_order, arc_num, name: str = "multi_car") -> None:

        self.nx = 0
        self.ny = 0
        self.nu = 0

        time_constant = 0.2

        car0_p = ca.SX.sym('car0_p', 2)
        car0_theta = ca.SX.sym('car0_theta', 1)
        car0_v_real = ca.SX.sym('car0_v_real', 1)
        car0_v_target = ca.SX.sym('car0_v_target', 1)
        car0_omega_real = ca.SX.sym('car0_omega_real', 1)
        car0_omega_target = ca.SX.sym('car0_omega_target', 1)
        car0_s = ca.SX.sym('car0_s', 1)
        car0_ds = ca.SX.sym('car0_ds', 1)
        car0_state = ca.vertcat(
            car0_p, car0_theta, car0_v_real, car0_omega_real, car0_s)
        car0_u = ca.vertcat(car0_v_target, car0_omega_target, car0_ds)
        car0_f_expl = ca.vertcat(car0_v_real*ca.cos(car0_theta),
                                 car0_v_real*ca.sin(car0_theta),
                                 car0_omega_real,
                                 1/time_constant*(car0_v_target-car0_v_real),
                                 1/time_constant *
                                 (car0_omega_target-car0_omega_real),
                                 car0_ds)
        self.nx += car0_state.size()[0]
        self.nu += car0_u.size()[0]

        car1_p = ca.SX.sym('car1_p', 2)
        car1_theta = ca.SX.sym('car1_theta', 1)
        car1_v_real = ca.SX.sym('car1_v_real', 1)
        car1_v_target = ca.SX.sym('car1_v_target', 1)
        car1_omega_real = ca.SX.sym('car1_omega_real', 1)
        car1_omega_target = ca.SX.sym('car1_omega_target', 1)
        car1_s = ca.SX.sym('car1_s', 1)
        car1_ds = ca.SX.sym('car1_ds', 1)
        car1_state = ca.vertcat(
            car1_p, car1_theta, car1_v_real, car1_omega_real, car1_s)
        car1_u = ca.vertcat(car1_v_target, car1_omega_target, car1_ds)
        car1_f_expl = ca.vertcat(car1_v_real*ca.cos(car1_theta),
                                 car1_v_real*ca.sin(car1_theta),
                                 car1_omega_real,
                                 1/time_constant*(car1_v_target-car1_v_real),
                                 1/time_constant *
                                 (car1_omega_target-car1_omega_real),
                                 car1_ds)
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

        car0_err = ca.vertcat(
            car0_px_desired-car0_p[0], car0_py_desired-car0_p[1])
        car0_err_l = ca.dot(car0_err, ca.vertcat(car0_diff_x, car0_diff_y))
        car0_err_l_2 = car0_err_l**2
        car0_err_c_2 = car0_err.T @ car0_err - car0_err_l_2
        car1_err = ca.vertcat(
            car1_px_desired-car1_p[0], car1_py_desired-car1_p[1])
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
        model.p = ca.vertcat(ca.reshape(car0_arc_param.T, (-1, 1)), car0_arc_length,
                             ca.reshape(car1_arc_param.T, (-1, 1)), car1_arc_length, obstacle_pos)

        self.N = 10
        self.dt = 0.1
        ocp = AcadosOcp()
        ocp.model = model
        ocp.dims.N = self.N
        ocp.parameter_values = np.zeros((model.p.size()[0], 1))
        ocp.solver_options.tf = self.N*self.dt

        def pos_err(x):
            l1 = ca.SX.sym("l1")
            l2 = ca.SX.sym("l2")
            input = ca.SX.sym("x")
            MyGate = ca.Function("MyGate", [input, l1, l2], [
                                 ca.rectangle(1/(l2-l1)*(input-(l1+l2)/2))])
            return MyGate(x, 0, 1.5**2) * (-44.4*x+100)

        ocp.cost.cost_type = 'EXTERNAL'
        # w0 = [0.5, 0.5, 1.0, 0.05, 1.0]
        # w1 = [0.5, 0.5, 1.0, 0.05, 1.0]
        w0 = [0.5, 0.5, 0.1, 0.005, 0.0]
        w1 = [0.5, 0.5, 0.1, 0.005, 0.0]
        ocp.model.cost_expr_ext_cost = w0[0]*car0_err_l_2 + w1[0] * \
            car1_err_l_2 + w0[1]*car0_err_c_2 + w1[1]*car1_err_c_2 + \
            w0[2]*pos_err(car0_obs_err) + w1[2]*pos_err(car1_obs_err) - w0[3]*car0_ds - w1[3]*car1_ds \
            + w0[4]*pos_err(car0_car1_distance)

        self.print_fun0 = ca.Function('print_fun0', [model.x, model.u, model.p], [
                                     w0[0]*car0_err_l_2, w0[1]*car0_err_c_2, w0[2]*pos_err(car0_obs_err), - w0[3]*car0_ds,  w0[4]*pos_err(car0_car1_distance)])
        self.print_fun1 = ca.Function('print_fun1', [model.x, model.u, model.p], [
                                     w1[0]*car1_err_l_2, w1[1]*car1_err_c_2, w1[2]*pos_err(car1_obs_err), - w1[3]*car1_ds,  w1[4]*pos_err(car0_car1_distance)])

        self.get_param = ca.Function('get_param', [model.x, model.u, model.p], [car0_diff_x**2+car0_diff_y**2])
        # car0_p, car0_theta, car0_v_real, car0_omega_real, car0_s
        ocp.constraints.idxbx = np.array([5, 11])
        ocp.constraints.lbx = np.array([0, 0])
        ocp.constraints.ubx = np.array([0, 0])

        ocp.constraints.idxbx_e = np.array([5, 11])
        ocp.constraints.lbx_e = np.array([0, 0])
        ocp.constraints.ubx_e = np.array([0, 0])

        self.x0 = np.array([-3, 0, 0, 0, 0, 1e-6,
                            3, 0, np.pi, 0, 0, 1e-6])
        ocp.constraints.x0 = self.x0

        # car0_v_target, car0_omega_target, car0_ds
        ocp.constraints.idxbu = np.arange(self.nu)
        ocp.constraints.lbu = np.array([-0.5, -1, 0, -0.5, -1, 0])
        ocp.constraints.ubu = np.array([0.5, 1, 0.5, 0.5, 1, 0.5])

        ocp.solver_options.hessian_approx = 'EXACT'

        ocp.solver_options.regularize_method = 'MIRROR'  # MIRROR,CONVEXIFY
        # 'PARTIAL_CONDENSING_HPIPM''FULL_CONDENSING_HPIPM'
        ocp.solver_options.qp_solver = 'FULL_CONDENSING_HPIPM'
        ocp.solver_options.integrator_type = 'ERK'
        ocp.solver_options.nlp_solver_type = 'SQP'  # SQP_RTI, SQP
        ocp.solver_options.nlp_solver_max_iter = 100
        ocp.solver_options.qp_solver_iter_max = 100
        ocp.code_export_directory = os.path.dirname(
            os.path.realpath(__file__))+"/c_generated_code/"+model.name
        json_file = os.path.dirname(os.path.realpath(
            __file__))+"/json_files/"+model.name+'_acados_ocp.json'
        self.solver = AcadosOcpSolver(ocp, json_file=json_file)


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


def get_param():
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

    p = np.vstack((np.reshape(param_car0, (-1, 1)), np.reshape(lengths_car0, (-1, 1)),
                  np.reshape(param_car1, (-1, 1)), np.reshape(lengths_car1, (-1, 1)), np.array([[0.0], [0.0]])))
    np.save(os.path.dirname(os.path.realpath(__file__))+"/p.npy", p)
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


if __name__ == '__main__':
    rospy.init_node('mpcc_node', anonymous=True)
    json_files_path = os.path.dirname(os.path.realpath(__file__))+"/json_files"
    if not (os.path.exists(json_files_path)):
        os.makedirs(json_files_path)

    # get_param()

    arc_order = 3
    p = np.load(os.path.dirname(os.path.realpath(__file__))+"/p.npy")
    lengths_car0 = np.load(os.path.dirname(
        os.path.realpath(__file__))+"/lengths_car0.npy")
    lengths_car1 = np.load(os.path.dirname(
        os.path.realpath(__file__))+"/lengths_car1.npy")
    

    mpcc = Multi_MPCC(arc_order=arc_order,
                      arc_num=lengths_car0.shape[0], name="multi_car")
    param_car0 = np.load(os.path.dirname(
        os.path.realpath(__file__)) + "/param_car0.npy")
    param_car1 = np.load(os.path.dirname(
        os.path.realpath(__file__)) + "/param_car1.npy")

    omega = 0.5
    radius = 1.0
    x0 = mpcc.x0

    for i in range(1, mpcc.N+1):
        mpcc.solver.constraints_set(i, "ubx", np.array(
            [lengths_car0[-1], lengths_car1[-1]]))

    x_values = []
    y_values = []
    time_now = rospy.Time.now().to_sec()
    for i in range(300):
        loss = []
        time_now+=mpcc.dt
        for i in range(0, mpcc.N+1):
            t = time_now + i*mpcc.dt
            p[-2:] = np.array([[radius*cos(omega*t)], [radius*sin(omega*t)]])
            # p[-2:] = np.array([[0],[0]])
            mpcc.solver.set(i, "p", p)
        x_values.append(x0[0])
        y_values.append(x0[1])

        # print(mpcc.get_param(x0, np.zeros((mpcc.nu, 1)), p))

        u = mpcc.solver.solve_for_x0(x0)
        x1 = mpcc.solver.get(1, 'x')
        print(x0)
        for i in range(0, mpcc.N):
            x_temp = mpcc.solver.get(i, 'x')
            u_temp = mpcc.solver.get(i, 'u')
            loss1 = np.array([ca.DM.toarray(elem) for elem in mpcc.print_fun0(x_temp, u_temp, p)]).flatten()
            loss2 = np.array([ca.DM.toarray(elem) for elem in mpcc.print_fun1(x_temp, u_temp, p)]).flatten()
            loss.append(np.hstack([loss1, loss2]))
        loss = np.array(loss)
        loss = np.sum(loss, axis=0)
        print(loss)
        x0 = x1
    plt.plot(x_values, y_values)
    plt.show()
