#!/home/jiao/python_env/acados/bin/python
import numpy as np
import casadi as ca
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
import rospy


def GetArcInfo(arc_order, arc_num, arc_length, arc_param, car0_s):
    l1 = ca.SX.sym("l1")
    l2 = ca.SX.sym("l2")
    input = ca.SX.sym("s")
    MyGate = ca.Function("MyGate", [input, l1, l2], [
                        ca.rectangle(1/(l2-l1)*(input-(l1+l2)/2))])
    temp = 0
    for i in range(arc_order+1):
        temp += arc_param[0][i] * car0_s ** i
    px_desired = MyGate(car0_s, 0, arc_length[0])*temp
    temp = 0
    for i in range(arc_order+1):
        temp += arc_param[0][i+arc_order+1] * car0_s ** i
    py_desired = MyGate(car0_s, 0, arc_length[0])*temp

    for i in range(1, arc_num):
        temp = 0
        for j in range(arc_order+1):
            temp += arc_param[i][j] * (car0_s-arc_length[i-1]) ** j
        px_desired += MyGate(car0_s, arc_length[i-1], arc_length[i])*temp
        temp = 0
        for j in range(arc_order+1):
            temp += arc_param[i][j+arc_order+1] * \
                (car0_s-arc_length[i-1]) ** j
        py_desired += MyGate(car0_s, arc_length[i-1], arc_length[i])*temp

    temp = 0
    for i in range(arc_order):
        temp += arc_param[0][i+1] * car0_s ** i*(i+1)
    diff_x = MyGate(car0_s, 0, arc_length[0])*temp
    temp = 0
    for i in range(arc_order):
        temp += arc_param[0][i+1+arc_order+1] * car0_s ** i*(i+1)
    diff_y = MyGate(car0_s, 0, arc_length[0])*temp

    for i in range(1, arc_num):
        temp = 0
        for j in range(arc_order):
            temp += arc_param[i][j+1] * (car0_s-arc_length[i-1]) ** j*(j+1)
        diff_x += MyGate(car0_s, arc_length[i-1], arc_length[i])*temp
        temp = 0
        for j in range(arc_order):
            temp += arc_param[i][j+1+arc_order+1] * \
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
                                 1/time_constant*(car0_omega_target-car0_omega_real,
                                                  car0_ds))
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
                                 1/time_constant*(car1_omega_target-car1_omega_real,
                                                  car1_ds))
        self.nx += car1_state.size()[0]
        self.nu += car1_u.size()[0]

        model = AcadosModel()
        model.u = ca.vertcat(car0_u, car1_u)
        model.x = ca.vertcat(car0_state, car1_state)
        model.xdot = ca.SX.sym('x_dot', self.nx, 1)
        model.f_expl_expr = ca.vertcat(car0_f_expl, car1_f_expl)
        model.name = name

        car0_arc_param = ca.SX.sym('car0_arc_param', arc_num, (arc_order+1)*2)
        car0_arc_length = ca.SX.sym('car0_arc_length', arc_num, 1)
        car0_px_desired, car0_py_desired, car0_diff_x, car0_diff_y = GetArcInfo(arc_order, arc_num, car0_arc_length, car0_arc_param, car0_s)

        car1_arc_param = ca.SX.sym('car1_arc_param', arc_num, (arc_order+1)*2)
        car1_arc_length = ca.SX.sym('car1_arc_length', arc_num, 1)
        car1_px_desired, car1_py_desired, car1_diff_x, car1_diff_y = GetArcInfo(arc_order, arc_num, car1_arc_length, car1_arc_param, car1_s)
        
        car0_err = ca.vertcat(car0_px_desired-car0_p[0], car0_py_desired-car0_p[1])
        car0_err_l = ca.dot(car0_err, ca.vertcat(car0_diff_x, car0_diff_y))
        car0_err_l_2 = car0_err_l**2
        car0_err_c_2 = car0_err.T @ car0_err - car0_err_l_2
        car1_err = ca.vertcat(car1_px_desired-car1_p[0], car1_py_desired-car1_p[1])
        car1_err_l = ca.dot(car1_err, ca.vertcat(car1_diff_x, car1_diff_y))
        car1_err_l_2 = car1_err_l**2
        car1_err_c_2 = car1_err.T @ car1_err - car1_err_l_2

        obstacle_pos = ca.SX.sym('obstacle_pos', 2, 1)
        car0_obs_err = (car0_p[0:2]-obstacle_pos).T @ (car0_p[0:2]-obstacle_pos)
        car1_obs_err = (car1_p[0:2]-obstacle_pos).T @ (car1_p[0:2]-obstacle_pos)
        model.p = ca.vertcat(ca.reshape(car0_arc_param,(-1,1)), car0_arc_length, ca.reshape(car1_arc_param), car1_arc_length, obstacle_pos)

        N = 10
        dt = 0.1
        ocp = AcadosOcp()
        ocp.model = model
        ocp.dims.N = 10
        ocp.solver_options.tf = N*dt
        ocp.cost.cost_type = 'EXTERNAL'
        w = [1.0, 1.0, 1.0]
        ocp.model.cost_expr_ext_cost = w[0]*car0_err_l_2 + w[1]*car0_err_c_2 + w[0]*car1_err_l_2 + w[1]*car1_err_c_2 + w[2]*car0_obs_err + w[2]*car1_obs_err

        ocp.constraints.idxbx = np.array([4, 9])
        ocp.constraints.lbx = np.array([0,0])
        ocp.constraints.ubx = np.array([car0_arc_length[-1], car1_arc_length[-1]])

        self.x0 = np.zeros((self.nx))
        ocp.constraints.x0 = self.x0
        ocp.constraints.idxbu = np.arange(self.nu)
        ocp.constraints.lbu = np.array([-3, -2, 0, -3, -2, 0])
        ocp.constraints.ubu = np.array([3, 2, 3, 3, 2, 3])