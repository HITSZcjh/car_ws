#!/home/jiao/python_env/acados/bin/python
import numpy as np
import casadi as ca
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
import rospy
import os
from math import *
import matplotlib.pyplot as plt

class diff_car_controller(object):
    def __init__(self, name:str="diff_car_controller") -> None:
        p = ca.SX.sym('p', 2)
        theta = ca.SX.sym('theta')
        v_real = ca.SX.sym('v_real')
        omega_real = ca.SX.sym('omega_real')

        v_target = ca.SX.sym('v_target')
        omega_target = ca.SX.sym('omega_target')

        time_constant = 0.2
        state = ca.vertcat(p, theta, v_real, omega_real)
        u = ca.vertcat(v_target, omega_target)
        f_expr = ca.vertcat(v_real * ca.cos(theta), 
                            v_real * ca.sin(theta), 
                            omega_real, 
                            1/time_constant * (v_target - v_real), 
                            1/time_constant * (omega_target - omega_real))
        
        self.nx = state.size()[0]
        self.nu = u.size()[0]
        model = AcadosModel()
        model.x = state
        model.u = u
        model.xdot = ca.SX.sym('x_dot', self.nx, 1)
        model.f_expl_expr = f_expr
        model.name = name
        model.p = []

        self.N = 40
        self.dT = 0.02
        ocp = AcadosOcp()
        ocp.model = model
        ocp.dims.N = self.N
        ocp.solver_options.tf = self.N * self.dT

        ocp.cost.cost_type = 'LINEAR_LS'
        ocp.cost.cost_type_e = 'LINEAR_LS'
        
        # p, theta, v_real, omega_real
        