#!/home/jiao/python_env/acados/bin/python
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
import numpy as np
from casadi import *
from matplotlib import pyplot as plt
from itertools import product
import scipy.linalg
from matplotlib import pyplot as plt
import math
import timeit


class PolyTrajectory(object):
    def __init__(self, order=7, N=6, k=4, tf=1, point=None,name=None):
        self.order = order
        self.N = N - 1
        self.tf = tf
        ocp = AcadosOcp()
        model = AcadosModel()
        x = SX.sym("x", (self.order+1)*self.N, 1)
        model.disc_dyn_expr = x
        model.x = x
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
                Q_temp[i][j] = self.b[i]*self.b[j]*self.tf**(i+j+1)/(i+j+1)
        Q_temp = scipy.linalg.block_diag(np.zeros((k, k)), Q_temp)
        Q = Q_temp
        ocp.cost.cost_type_e = 'EXTERNAL'
        for i in range(self.N-1):
            Q = scipy.linalg.block_diag(Q, Q_temp)
        ocp.model.cost_expr_ext_cost_e = model.x.T @ Q @ model.x

        self.A = np.zeros((0, 0))
        position_t0 = np.zeros((self.order+1))
        position_t0[0] = 1
        position_tf = np.zeros((self.order+1))
        for i in range(self.order+1):
            position_tf[i] = self.tf**i

        diff_t0 = np.zeros((self.order, self.order+1))
        for i in range(self.order):
            diff_t0[i][i+1] = math.factorial(i+1)

        diff_tf = np.zeros((self.order, self.order+1))
        for i in range(self.order):
            temp = 0
            for j in range(i+1, self.order+1):
                diff_tf[i][j] = math.factorial(
                    j)/math.factorial(j-i-1)*self.tf**temp
                temp += 1

        for i in range(self.N):
            zero1 = np.zeros((i*(self.order+1)))
            zero2 = np.zeros(((self.N-1-i)*(self.order+1)))
            if (i == 0):
                self.A = np.hstack((zero1, position_t0, zero2))
                self.A = np.vstack((self.A, np.hstack((zero1, position_tf, zero2))))
            else:
                self.A = np.vstack((self.A, np.hstack((zero1, position_t0, zero2))))
                self.A = np.vstack((self.A, np.hstack((zero1, position_tf, zero2))))

        for i in range(self.N-1):
            zero1 = np.zeros((i*(self.order+1)))
            zero3 = np.zeros(((self.N-2-i)*(self.order+1)))
            for j in range(self.order):
                self.A = np.vstack(
                    (self.A, np.hstack((zero1, diff_tf[j], -diff_t0[j], zero3))))

        self.b = np.zeros(self.A.shape[0])
        self.b[0] = point[0]
        for i in range(1,self.N):
            self.b[2*i-1] = point[i]
            self.b[2*i] = point[i]
        self.b[self.N*2-1] = point[self.N]
        
        ocp.model.con_h_expr = self.A @ x
        ocp.constraints.lh = self.b
        ocp.constraints.uh = self.b

        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        ocp.solver_options.hessian_approx = 'EXACT'
        ocp.solver_options.integrator_type = 'DISCRETE'
        # ocp.solver_options.regularize_method = 'CONVEXIFY'
        ocp.solver_options.tol = 1e-6
        ocp.solver_options.nlp_solver_type = 'SQP'
        ocp.solver_options.qp_solver_iter_max = 50
        ocp.solver_options.print_level = 0
        ocp.code_export_directory = os.path.dirname(os.path.realpath(__file__))+"/c_generated_code/"+model.name
        json_file = os.path.dirname(os.path.realpath(__file__))+"/json_files/"+model.name+'_acados_ocp.json'
        self.solver = AcadosOcpSolver(ocp, json_file=json_file)
        
    def set_point(self, point):
        self.b = np.zeros(self.A.shape[0])
        self.b[0] = point[0]
        for i in range(1,self.N):
            self.b[2*i-1] = point[i]
            self.b[2*i] = point[i]
        self.b[self.N*2-1] = point[self.N]

        self.solver.constraints_set(0,"lh",self.b)
        self.solver.constraints_set(0,"uh",self.b)
    
    def solve(self):
        start = timeit.default_timer()
        status = self.solver.solve()
        time_record = timeit.default_timer() - start
        print("estimation time is {}".format(time_record))
        if(status!=0):
            print("solve_error!")
            exit()
        self.solver.print_statistics()
        self.param = self.solver.get(0, 'x')

    def get_trajectory(self,Ts):
        sample_num = int(self.tf/Ts)
        t = np.linspace(0, self.tf-Ts, sample_num)
        y = np.zeros((0))
        for i in range(self.N):
            temp = np.array(0)
            for j in range((self.order+1)):
                temp = temp + self.param[j+i*(self.order+1)]*t**j
            y = np.hstack((y,temp))
        t = np.linspace(0, self.N*self.tf, self.N*sample_num)
        return t,y

    def draw(self):
        sample_num = 1000
        t = np.linspace(0, self.tf, sample_num)
        y = np.zeros((0))
        for i in range(self.N):
            temp = np.array(0)
            for j in range((self.order+1)):
                temp = temp + self.param[j+i*(self.order+1)]*t**j
            y = np.hstack((y,temp))
        t = np.linspace(0, self.N*self.tf, self.N*sample_num)
        plt.plot(t, y)
        plt.show()
        
if __name__ == '__main__':

    tra1 = PolyTrajectory(N=6, point=[0,3,1,5,3,2],order=7,k=4,tf=15,name="mytest")
    tra1.solve()
    tra1.draw()
    tra1.set_point(point=[0,3,1,5,1,2])
    tra1.solve()
    tra1.draw()
