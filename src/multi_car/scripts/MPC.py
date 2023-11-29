#!/home/jiao/python_env/acados/bin/python
import numpy as np
import casadi as ca
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
import rospy
import os
from math import *
import matplotlib.pyplot as plt
import scipy


class diff_car_controller(object):
    def __init__(self, name: str = "diff_car_controller") -> None:
        p = ca.SX.sym('p', 2)
        theta = ca.SX.sym('theta')
        v = ca.SX.sym('v')
        omega = ca.SX.sym('omega')

        d_v = ca.SX.sym('d_v')
        d_omega = ca.SX.sym('d_omega')

        state = ca.vertcat(p, theta, v, omega)
        u = ca.vertcat(d_v, d_omega)
        f_expr = ca.vertcat(v * ca.cos(theta),
                            v * ca.sin(theta),
                            omega,
                            d_v,
                            d_omega)

        self.nx = state.size()[0]
        self.nu = u.size()[0]
        self.ny = self.nx+self.nu
        model = AcadosModel()
        model.x = state
        model.u = u
        model.xdot = ca.SX.sym('x_dot', self.nx, 1)
        model.f_expl_expr = f_expr
        model.name = name
        model.p = []

        self.N = 40
        self.dT = 0.2
        ocp = AcadosOcp()
        ocp.model = model
        ocp.dims.N = self.N
        ocp.solver_options.tf = self.N * self.dT

        ocp.cost.cost_type = 'LINEAR_LS'
        ocp.cost.cost_type_e = 'LINEAR_LS'

        # px, py, theta, v, omega
        Q = np.diag([1.0, 1.0, 0.0, 0.0, 0.0])
        # d_v, d_omega
        # R = np.diag([0.04, 0.2])
        R = np.diag([0.001, 0.002])
        ocp.cost.W = scipy.linalg.block_diag(Q, R)
        ocp.cost.W_e = Q
        ocp.cost.Vx = np.zeros((self.ny, self.nx))
        ocp.cost.Vx[:self.nx, :self.nx] = np.eye(self.nx)
        ocp.cost.Vx_e = np.eye(self.nx)
        ocp.cost.Vu = np.zeros((self.ny, self.nu))
        ocp.cost.Vu[-self.nu:, -self.nu:] = np.eye(self.nu)

        self.yref = np.zeros(self.ny)
        self.yref_e = np.zeros(self.nx)
        ocp.cost.yref = self.yref
        ocp.cost.yref_e = self.yref_e

        self.x0 = np.zeros((self.nx))
        ocp.constraints.x0 = self.x0

        # px, py, theta, v, omega
        ocp.constraints.idxbx = np.array([3, 4])
        ocp.constraints.lbx = np.array([-0.5, -0.7])
        ocp.constraints.ubx = np.array([0.5, 0.7])

        # d_v, d_omega
        ocp.constraints.idxbu = np.arange(0, self.nu)
        ocp.constraints.lbu = np.array([-1.0, -1.0])
        ocp.constraints.ubu = np.array([1.0, 1.0])

        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        # 'PARTIAL_CONDENSING_HPIPM''FULL_CONDENSING_HPIPM'
        ocp.solver_options.qp_solver = 'FULL_CONDENSING_HPIPM'
        ocp.solver_options.integrator_type = 'ERK'
        ocp.solver_options.nlp_solver_type = 'SQP_RTI'  # SQP_RTI, SQP
        ocp.solver_options.nlp_solver_max_iter = 100
        ocp.solver_options.qp_solver_iter_max = 100

        ocp.code_export_directory = os.path.dirname(
            os.path.realpath(__file__))+"/c_generated_code/"+model.name
        json_file = os.path.dirname(os.path.realpath(
            __file__))+"/json_files/"+model.name+'_acados_ocp.json'
        self.solver = AcadosOcpSolver(ocp, json_file=json_file)

if __name__ == '__main__':
    controller = diff_car_controller()
    x0 = 0.1*np.ones_like(controller.x0)

    yref = 0.0*np.ones_like(controller.yref)
    controller.solver.set(1, "yref", controller.yref)
    controller.solver.solve_for_x0(x0)
    controller.solver.solve_for_x0(x0)
    print(x0)
    print(controller.solver.get(1, "x"))
    print(controller.solver.get(0, "u"))