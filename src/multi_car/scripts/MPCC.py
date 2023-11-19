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


def GetArcInfo(arc_order, arc_num, arc_length, arc_param, car0_s):
    l1 = ca.SX.sym("l1")
    l2 = ca.SX.sym("l2")
    input = ca.SX.sym("s")
    MyGate = ca.Function("MyGate", [input, l1, l2], [
        ca.rectangle(1/(l2-l1)*(input-(l1+l2)/2))])
    temp = 0
    for i in range(arc_order+1):
        temp += arc_param[0,i] * car0_s ** i
    px_desired = MyGate(car0_s, 0, arc_length[0])*temp
    temp = 0
    for i in range(arc_order+1):
        temp += arc_param[0,i+arc_order+1] * car0_s ** i
    py_desired = MyGate(car0_s, 0, arc_length[0])*temp

    for i in range(1, arc_num):
        temp = 0
        for j in range(arc_order+1):
            temp += arc_param[i,j] * (car0_s-arc_length[i-1]) ** j
        px_desired += MyGate(car0_s, arc_length[i-1], arc_length[i])*temp
        temp = 0
        for j in range(arc_order+1):
            temp += arc_param[i,j+arc_order+1] * \
                (car0_s-arc_length[i-1]) ** j
        py_desired += MyGate(car0_s, arc_length[i-1], arc_length[i])*temp

    temp = 0
    for i in range(arc_order):
        temp += arc_param[0,i+1] * car0_s ** i*(i+1)
    diff_x = MyGate(car0_s, 0, arc_length[0])*temp
    temp = 0
    for i in range(arc_order):
        temp += arc_param[0,i+1+arc_order+1] * car0_s ** i*(i+1)
    diff_y = MyGate(car0_s, 0, arc_length[0])*temp

    for i in range(1, arc_num):
        temp = 0
        for j in range(arc_order):
            temp += arc_param[i,j+1] * (car0_s-arc_length[i-1]) ** j*(j+1)
        diff_x += MyGate(car0_s, arc_length[i-1], arc_length[i])*temp
        temp = 0
        for j in range(arc_order):
            temp += arc_param[i,j+1+arc_order+1] * \
                (car0_s-arc_length[i-1]) ** j*(j+1)
        diff_y += MyGate(car0_s, arc_length[i-1], arc_length[i])*temp
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
            arc_order, arc_num, car0_arc_length, car0_arc_param, car0_s)

        car1_arc_param = ca.SX.sym('car1_arc_param', arc_num, (arc_order+1)*3)
        car1_arc_length = ca.SX.sym('car1_arc_length', arc_num, 1)
        car1_px_desired, car1_py_desired, car1_diff_x, car1_diff_y = GetArcInfo(
            arc_order, arc_num, car1_arc_length, car1_arc_param, car1_s)

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
        model.p = ca.vertcat(ca.reshape(car0_arc_param, (-1, 1)), car0_arc_length,
                             ca.reshape(car1_arc_param, (-1, 1)), car1_arc_length, obstacle_pos)
        
        self.N = 10
        self.dt = 0.1
        ocp = AcadosOcp()
        ocp.model = model
        ocp.dims.N = 10
        ocp.parameter_values = np.zeros((model.p.size()[0], 1))
        ocp.solver_options.tf = self.N*self.dt

        ocp.cost.cost_type = 'EXTERNAL'
        w = [1.0, 1.0, 1.0]
        ocp.model.cost_expr_ext_cost = w[0]*car0_err_l_2 + w[1]*car0_err_c_2 + w[0] * \
            car1_err_l_2 + w[1]*car1_err_c_2 + \
            w[2]*car0_obs_err + w[2]*car1_obs_err

        ocp.constraints.idxbx = np.array([4, 9])
        ocp.constraints.lbx = np.array([0, 0])
        ocp.constraints.ubx = np.array([0,0])
        # car0_p, car0_theta, car0_v_real, car0_omega_real, car0_s
        self.x0 = np.array([-3, 0, 0, 0, 0, 0,
                            3, 0, np.pi, 0, 0, 0])
        ocp.constraints.x0 = self.x0
        ocp.constraints.idxbu = np.arange(self.nu)
        ocp.constraints.lbu = np.array([-3, -2, 0, -3, -2, 0])
        ocp.constraints.ubu = np.array([3, 2, 3, 3, 2, 3])

        ocp.solver_options.hessian_approx = 'EXACT'

        ocp.solver_options.regularize_method = 'CONVEXIFY'  # MIRROR,CONVEXIFY
        # 'PARTIAL_CONDENSING_HPIPM''FULL_CONDENSING_HPIPM'
        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        ocp.solver_options.integrator_type = 'ERK'
        ocp.solver_options.nlp_solver_type = 'SQP'  # SQP_RTI, SQP
        ocp.solver_options.qp_solver_iter_max = 50
        ocp.code_export_directory(os.path.dirname(os.path.realpath(__file__))+"/"+model.name)
        json_file = os.path.dirname(os.path.realpath(__file__))+model.name+'_acados_ocp.json'
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


if __name__ == '__main__':
    rospy.init_node('mpcc', anonymous=True)

    point_car0 = np.array([[-3,0,0],[0,0,0],[0,-3,0]]).T
    point_car1 = np.array([[3,0,0],[0,0,0],[0,3,0]]).T

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
    arc_car0 = ArcTrajectory(order=order, param=param_car0, N=trax_car0.N, tf=tf, m=sample_point_num)
    arc_car0.generate_points()
    arc_car0.solve(order=arc_order,k=arc_k)
    param_car0,lengths_car0 = arc_car0.get_param()
    points_car0 = np.array(arc_car0.points)

    param_car1 = np.vstack((trax_car1.param, tray_car1.param, traz_car1.param))
    arc_car1 = ArcTrajectory(order=order, param=param_car1, N=trax_car1.N, tf=tf, m=sample_point_num)
    arc_car1.generate_points()
    arc_car1.solve(order=arc_order,k=arc_k)
    param_car1,lengths_car1 = arc_car1.get_param()
    points_car1 = np.array(arc_car1.points)

    marker_array_car0 = show_arc_traj(points_car0)
    marker_array_car1 = show_arc_traj(points_car1)

    p = np.vstack((np.reshape(param_car0,(-1,1)), np.reshape(lengths_car0,(-1,1)), np.reshape(param_car1,(-1,1)), np.reshape(lengths_car1,(-1,1)), np.array([[0.0],[0.0]])))
    np.save(os.path.dirname(os.path.realpath(__file__))+"/p.npy",p)
    np.save(os.path.dirname(os.path.realpath(__file__))+"/lengths_car0.npy",lengths_car0)
    np.save(os.path.dirname(os.path.realpath(__file__))+"/lengths_car1.npy",lengths_car1)
    print(param_car0.shape)
    exit()
    
    arc_order = 3
    p = np.load("p.npy")
    lengths_car0 = np.load("lengths_car0.npy")
    lengths_car1 = np.load("lengths_car1.npy")
    mpcc = Multi_MPCC(arc_order=arc_order, arc_num=lengths_car0.shape[0], name="multi_car")

    omega = 0.5
    radius = 1.0
    x0 = mpcc.x0
    while (not rospy.is_shutdown()):
        time_now = rospy.Time.now().to_sec()
        for i in range(mpcc.N+1):
            t = time_now + i*mpcc.dt
            p[-2:] = np.array([[radius*cos(omega*t)], [radius*sin(omega*t)]])
            mpcc.solver.set(i, "p", p)
            mpcc.solver.constraints_set(i,"idxbx", np.array([4,9]))
            mpcc.solver.constraints_set(i, "ubx", np.array([lengths_car0[-1],lengths_car1[-1]]))
        mpcc.solver.solve_for_x0(x0)
        x1 = mpcc.solver.get(1, 'x')
        print(x0)
        x0 = x1
        print(rospy.Time.now().to_sec()-time_now)
