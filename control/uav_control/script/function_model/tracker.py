import casadi as ca
import numpy as np
from acados_template import AcadosOcp, AcadosOcpSolver

# import tf
import os, sys
BASEPATH = os.path.abspath(__file__).split('function_model/', 1)[0]+'function_model/'
sys.path += [BASEPATH]
from os import system
from quadrotor_control import QuadrotorSimpleModel,QuadrotorModel

# calculate yaw error according to q
def yaw_q_error(yaw_d, qw, qz):
    c_yaw = ca.cos(yaw_d / 2)
    s_yaw = ca.sin(yaw_d / 2)
    sqrt_q = ca.sqrt(qw * qw + qz * qz)
    delta_yaw_c = qw - c_yaw * sqrt_q
    delta_yaw_s = qz - s_yaw * sqrt_q
    return delta_yaw_c * delta_yaw_c + delta_yaw_s * delta_yaw_s

def p_cost(v, Th):
    # 1
    c = v.T@v
    return c

def u_cost(U, diag_Q):
    ee = U - ca.DM([9.81, 0, 0, 0])
    c = ee.T@diag_Q@ee
    return c

# pos yaw
# pos yaw
class TrackerMPC():
    def __init__(self, quad:QuadrotorSimpleModel):
        # load uav model / mpc horizon / state equation  
        self._quad = quad
        self._Herizon = 5
        self._step=0.1
        self._ddynamics = []
        for n in range(self._Herizon):
            self._ddynamics += [self._quad.ddynamics(self._step)]
        
        # calculate param define
        self._X_dim = self._ddynamics[0].size1_in(0)
        self._U_dim = self._ddynamics[0].size1_in(1)
        self._X_lb = self._quad._X_lb
        self._X_ub = self._quad._X_ub
        self._U_lb = self._quad._U_lb
        self._U_ub = self._quad._U_ub
        
        self._Xs = ca.SX.sym('Xs', self._X_dim, self._Herizon)
        self._Us = ca.SX.sym('Us', self._U_dim, self._Herizon)
        
        self._X_init = ca.SX.sym("X_init", self._X_dim)
        self._Trj_p = ca.SX.sym("Trj_p", 3, self._Herizon)
        self._Trj_yaw = ca.SX.sym("Trj_yaw", self._Herizon)
        
        self._opt_option = {
            'verbose': False,
            'ipopt.tol': 1e-2,
            'ipopt.acceptable_tol': 1e-2,
            'ipopt.max_iter': 25,
            'ipopt.warm_start_init_point': 'yes',
            'ipopt.print_level': 0,
        }
        
        # nlp param init / 0
        self._nlp_x_x = []
        self._nlp_lbx_x = []
        self._nlp_ubx_x = []
        
        self._nlp_x_u = []
        self._nlp_lbx_u = []
        self._nlp_ubx_u = []

        self._nlp_g_dyn = []
        self._nlp_lbg_dyn = []
        self._nlp_ubg_dyn = []

        self._nlp_p_xinit = []
        self._nlp_p_Trj_p = []
        self._nlp_p_Trj_yaw = []

        self._nlp_obj_dyn = 0
        self._nlp_obj_trjp = 0
        self._nlp_obj_trjyaw = 0
        self._nlp_obj_u = 0
        
        self._nlp_x_x += [ self._Xs[:, 0] ]
        self._nlp_lbx_x += self._X_lb
        self._nlp_ubx_x += self._X_ub
        self._nlp_x_u += [ self._Us[:, 0] ]
        self._nlp_lbx_u += self._U_lb
        self._nlp_ubx_u += self._U_ub
        
        dd_dyn = self._Xs[:,0]-self._ddynamics[0]( self._X_init, self._Us[:,0] )
        self._nlp_g_dyn += [ dd_dyn ]
        self._nlp_obj_dyn += dd_dyn.T@dd_dyn
        self._nlp_lbg_dyn += [ -0.0 for _ in range(self._X_dim) ]
        self._nlp_ubg_dyn += [  0.0 for _ in range(self._X_dim) ]
        
        self._nlp_obj_trjp += p_cost(self._Xs[:3,0]-self._Trj_p[:,0], 0.5)
        self._nlp_obj_trjyaw += yaw_q_error(self._Trj_yaw[0], self._Xs[6, 0], self._Xs[9, 0])
        self._nlp_obj_u += u_cost(self._Us[:, 0], ca.diag([1, 1, 1, 0]))
        
        # nlp param add (x/g/p) / 1 to horizon
        for i in range(1,self._Herizon):
            self._nlp_x_x += [ self._Xs[:, i] ]
            self._nlp_lbx_x += self._X_lb
            self._nlp_ubx_x += self._X_ub
            self._nlp_x_u += [ self._Us[:, i] ]
            self._nlp_lbx_u += self._U_lb
            self._nlp_ubx_u += self._U_ub
            
            dd_dyn = self._Xs[:,i]-self._ddynamics[i]( self._Xs[:,i-1], self._Us[:,i] )
            self._nlp_g_dyn += [ dd_dyn ]
            self._nlp_obj_dyn += dd_dyn.T@dd_dyn
            self._nlp_lbg_dyn += [ -0.0 for _ in range(self._X_dim) ]
            self._nlp_ubg_dyn += [  0.0 for _ in range(self._X_dim) ]
            
            self._nlp_obj_trjp += p_cost(self._Xs[:3,i]-self._Trj_p[:,i], 0.5)
            self._nlp_obj_trjyaw += yaw_q_error(self._Trj_yaw[i], self._Xs[6, i], self._Xs[9, 0])
            self._nlp_obj_u += u_cost(self._Us[:, i], ca.diag([1, 1, 1, 0]))

        self._nlp_p_xinit += [self._X_init]
        for i in range(self._Herizon):
            self._nlp_p_Trj_p += [self._Trj_p[:,i]]
            self._nlp_p_Trj_yaw += [self._Trj_yaw[i]]
    
    # reset qw
    def reset_xut(self):
        self._xu0 = np.zeros((self._X_dim+self._U_dim)*self._Herizon)
        for i in range(self._Herizon):
            self._xu0[i*self._X_dim+6] = 1
    
    # load nlp solver file
    def load_so(self, so_path):
        self._opt_solver = ca.nlpsol("opt", "ipopt", so_path, self._opt_option)
        self.reset_xut()
    
    # define nlp solver
    def define_opt(self):
        nlp_dect = {
            'f': 1*self._nlp_obj_trjp + 0.01*self._nlp_obj_u + 0.1*self._nlp_obj_trjyaw,
            'x': ca.vertcat(*(self._nlp_x_x + self._nlp_x_u)),
            'p': ca.vertcat(*(self._nlp_p_xinit + self._nlp_p_Trj_p + self._nlp_p_Trj_yaw)),
            'g': ca.vertcat(*(self._nlp_g_dyn)),
        }
        self._opt_solver = ca.nlpsol('opt', 'ipopt', nlp_dect, self._opt_option)
        
        self.reset_xut()
        
    # solve nlp problem
    def solve(self, xinit, Trjp, Trjyaw):
        p = np.zeros(self._X_dim + 3*self._Herizon + 1*self._Herizon) #状态/期望位置/期望偏航
        #load param
        p[:self._X_dim] = xinit
        p[self._X_dim:self._X_dim+3*self._Herizon] = Trjp
        p[self._X_dim+3*self._Herizon:self._X_dim+4*self._Herizon] = Trjyaw
        
        res = self._opt_solver(
            x0=self._xu0,
            lbx=(self._nlp_lbx_x+self._nlp_lbx_u),
            ubx=(self._nlp_ubx_x+self._nlp_ubx_u),
            lbg=(self._nlp_lbg_dyn),
            ubg=(self._nlp_ubg_dyn),
            p=p
        )
        
        self._xu0 = res['x'].full().flatten()
        
        return res
    




class TrackerMPC_FULL():
    def __init__(self, quad:QuadrotorModel):
        self._quad = quad
        self._Herizon = 10
        self._step=0.16
        self._ddynamics = []
        for n in range(self._Herizon):
            self._ddynamics += [self._quad.ddynamics(self._step)]
        
        self._X_dim = self._ddynamics[0].size1_in(0)
        self._U_dim = self._ddynamics[0].size1_in(1)
        self._X_lb = self._quad._X_lb
        self._X_ub = self._quad._X_ub
        self._U_lb = self._quad._U_lb
        self._U_ub = self._quad._U_ub
        
        self._Xs = ca.SX.sym('Xs', self._X_dim, self._Herizon)
        self._Us = ca.SX.sym('Us', self._U_dim, self._Herizon)
        
        self._X_init = ca.SX.sym("X_init", self._X_dim)
        self._Trj_p = ca.SX.sym("Trj_p", 3, self._Herizon)
        self._Trj_yaw = ca.SX.sym("Trj_yaw", self._Herizon)
        # ca.set_option('threads', 4)
        self._opt_option = {
            'verbose': False,
            'ipopt.tol': 1e-2,
            'ipopt.acceptable_tol': 1e-1,
            'ipopt.acceptable_obj_change_tol': 1e-2,            # 设置目标函数的变化容差
            'ipopt.max_iter': 3,
            'ipopt.warm_start_init_point': 'yes',
            'ipopt.print_level': 0,
            'print_time':0,
            'ipopt.max_cpu_time': 0.03,  # 控制单线程运算时间
            'ipopt.max_wall_time': 0.05  # 包括资源调度和多线程开销的总时间
            # 'ipopt.hessian_approximation': 'limited-memory',  # 使用有限记忆近似 Hessian
            # 'ipopt.limited_memory_max_history': 4,
        }
        
        self._nlp_x_x = []
        self._nlp_lbx_x = []
        self._nlp_ubx_x = []
        
        self._nlp_x_u = []
        self._nlp_lbx_u = []
        self._nlp_ubx_u = []

        self._nlp_g_dyn = []
        self._nlp_lbg_dyn = []
        self._nlp_ubg_dyn = []

        self._nlp_p_xinit = []
        self._nlp_p_Trj_p = []
        # self._nlp_p_Trj_v = []
        self._nlp_p_Trj_yaw = []
        
        self._nlp_obj_dyn = 0
        self._nlp_obj_trjp = 0
        # self._nlp_obj_trjv = 0
        self._nlp_obj_trjyaw = 0
        self._nlp_obj_u = 0
        self._nlp_obj_u_con = 0

        self._nlp_x_x += [ self._Xs[:, 0] ]
        self._nlp_lbx_x += self._X_lb
        self._nlp_ubx_x += self._X_ub
        self._nlp_x_u += [ self._Us[:, 0] ]
        self._nlp_lbx_u += self._U_lb
        self._nlp_ubx_u += self._U_ub
        
        dd_dyn = self._Xs[:,0]-self._ddynamics[0]( self._X_init, self._Us[:,0] )
        self._nlp_g_dyn += [ dd_dyn ]
        self._nlp_obj_dyn += dd_dyn.T@dd_dyn
        self._nlp_lbg_dyn += [ -0.0 for _ in range(self._X_dim) ]
        self._nlp_ubg_dyn += [  0.0 for _ in range(self._X_dim) ]
        
        self._nlp_obj_trjp += p_cost(self._Xs[:3,0]-self._Trj_p[:,0], 0.5)
        self._nlp_obj_trjyaw += yaw_q_error(self._Trj_yaw[0], self._Xs[6, 0], self._Xs[9, 0])
        # self._nlp_obj_u += self._Us[:,0].T@self._Us[:,0]
        self._nlp_obj_u += self._Xs[10:13,0].T@self._Xs[10:13,0] #U惩罚函数是角速度模长平方
        
        for i in range(1,self._Herizon):
            self._nlp_x_x += [ self._Xs[:, i] ]
            self._nlp_lbx_x += self._X_lb
            self._nlp_ubx_x += self._X_ub
            self._nlp_x_u += [ self._Us[:, i] ]
            self._nlp_lbx_u += self._U_lb
            self._nlp_ubx_u += self._U_ub
            
            dd_dyn = self._Xs[:,i]-self._ddynamics[i]( self._Xs[:,i-1], self._Us[:,i] )
            self._nlp_g_dyn += [ dd_dyn ]
            self._nlp_obj_dyn += dd_dyn.T@dd_dyn
            self._nlp_lbg_dyn += [ -0.0 for _ in range(self._X_dim) ]
            self._nlp_ubg_dyn += [  0.0 for _ in range(self._X_dim) ]
            
            self._nlp_obj_trjp += p_cost(self._Xs[:3,i]-self._Trj_p[:,i], 0.5)
            self._nlp_obj_trjyaw += yaw_q_error(self._Trj_yaw[i], self._Xs[6, i], self._Xs[9, 0])
            # self._nlp_obj_u += self._Us[:,i].T@self._Us[:,i]
            self._nlp_obj_u += self._Xs[10:13,i].T@self._Xs[10:13,i] 

            # 惩罚输入变化
            self._nlp_obj_u_con += (self._Us[:, i] - self._Us[:, i-1]).T@ (self._Us[:, i] - self._Us[:, i-1])

        self._nlp_p_xinit += [self._X_init]
        for i in range(self._Herizon):
            self._nlp_p_Trj_p += [self._Trj_p[:,i]]
            self._nlp_p_Trj_yaw += [self._Trj_yaw[i]]
    
    def reset_xut(self):
        self._xu0 = np.zeros((self._X_dim+self._U_dim)*self._Herizon)
        for i in range(self._Herizon):
            self._xu0[i*self._X_dim+6] = 1

    def load_so(self, so_path):
        self._opt_solver = ca.nlpsol("opt", "ipopt", so_path, self._opt_option)
        self.reset_xut()
    
    def define_opt(self):
        nlp_dect = {
            #'f': 3*self._nlp_obj_trjp +0.000*self._nlp_obj_u +0.1*self._nlp_obj_u_con+ 0.0*self._nlp_obj_trjyaw,
            'f': 5*self._nlp_obj_trjp +0.3*self._nlp_obj_u_con, #完全没有yaw
            # 'f': 3*self._nlp_obj_trjp ,
            'x': ca.vertcat(*(self._nlp_x_x+self._nlp_x_u)),
            'p': ca.vertcat(*(self._nlp_p_xinit + self._nlp_p_Trj_p + self._nlp_p_Trj_yaw)),
            'g': ca.vertcat(*(self._nlp_g_dyn)),
        }
        self._opt_solver = ca.nlpsol('opt', 'ipopt', nlp_dect, self._opt_option)
        
        self.reset_xut()
        
     # solve nlp problem
    def solve(self, xinit, Trjp, Trjyaw):
        p = np.zeros(self._X_dim+3*self._Herizon+ 1*self._Herizon)
        #load param
        p[:self._X_dim] = xinit
        p[self._X_dim:self._X_dim+3*self._Herizon] = Trjp
        p[self._X_dim+3*self._Herizon:self._X_dim+4*self._Herizon] = Trjyaw

        res = self._opt_solver(
            x0=self._xu0,
            lbx=(self._nlp_lbx_x+self._nlp_lbx_u),
            ubx=(self._nlp_ubx_x+self._nlp_ubx_u),
            lbg=(self._nlp_lbg_dyn),
            ubg=(self._nlp_ubg_dyn),
            p=p
        )
        
        self._xu0 = res['x'].full().flatten()
        
        return res


class TrackerMPC_FULL_AC():
    def __init__(self, quad:QuadrotorModel):
        self._quad = quad
        self._Herizon = 10
        self._step=0.1

        self._dynamics = self._quad.dynamics()
        self._X_dim = self._dynamics.size1_in(0)
        self._U_dim = self._dynamics.size1_in(1)
        # print(  self._X_dim ,  self._U_dim )

        self._X_lb = self._quad._X_lb
        self._X_ub = self._quad._X_ub
        self._U_lb = self._quad._U_lb
        self._U_ub = self._quad._U_ub
        # 初始化 ACADOS OCP
        self._ocp = AcadosOcp()
        self.define_ocp()
        
    def solve(self, x_init, Trjp, Trjyaw, wind_local):
        # print(x_init)
        if x_init is None:
            x_init = [0, 0, 0]  + [0, 0, 0] + [1, 0, 0, 0]+[0, 0, 0]
        # x_init = [0, 0, 0]  + [0, 0, 0] + [1, 0, 0, 0]+[0, 0, 0]
        # print(x_init)
        x_init = np.concatenate([x_init, wind_local])
        # print(x_init)
        x_init = np.stack(x_init)
                # Set initial condition, equality constraint
        self.solver.set(0, 'lbx', x_init)
        self.solver.set(0, 'ubx', x_init)


        #set_reference_trajectory
        for j in range(self._Herizon):
            ref = Trjp[j*3: j*3+3]
            # Convert yaw to quaternion
            q_yaw = np.array([
                np.cos(Trjyaw[j] / 2),
                0,
                0,
                np.sin(Trjyaw[j] / 2)
            ])
            if self.prev_q is not None:
            # 如果和前一个四元数的点积小于0，取反
                if np.dot(q_yaw, self.prev_q) < 0:
                    q_yaw = -q_yaw
            self.prev_q = q_yaw

            ref = np.concatenate((ref, np.zeros(3),q_yaw, np.zeros(3),np.zeros(4), np.zeros(3)))
            # ref[6]=1
            self.solver.set(j, "yref", ref)
        # the last MPC node has only a state reference but no input reference
        ref = Trjp[(self._Herizon-1)*3:(self._Herizon-1)*3+3]
        ref = np.concatenate((ref, np.zeros(3),q_yaw, np.zeros(3), np.zeros(3)))
        # ref[6]=1
        self.solver.set(self._Herizon, "yref",ref)


        # Solve OCP
        self.solver.solve()
        # Get u
        w_opt_acados = np.ndarray((self._Herizon, 4))
        x_opt_acados = np.ndarray((self._Herizon + 1, len(x_init)))
        x_opt_acados[0, :] = self.solver.get(0, "x")
        for i in range(self._Herizon):
            w_opt_acados[i, :] = self.solver.get(i, "u")
            x_opt_acados[i + 1, :] = self.solver.get(i + 1, "x")

        w_opt_acados = np.reshape(w_opt_acados, (-1))
        return w_opt_acados,x_opt_acados
        
        
    def define_ocp(self):
        self.prev_q=None
        # ocp.acados_include_path = acados_source_path + '/include'
        # ocp.acados_lib_path = acados_source_path + '/lib'
        # 动力学
        x = ca.SX.sym('x', self._X_dim)
        u = ca.SX.sym('u', self._U_dim)
        x_dot = ca.SX.sym('x_dot', x.size1())
        f_impl = x_dot -self._dynamics(x, u)
        # 设置 OCP 模型
        self._ocp.model.name = 'quadrotor_mpc'
        self._ocp.model.x = x
        self._ocp.model.u = u
        self._ocp.model.xdot = x_dot  
        self._ocp.model.f_expl_expr =  self._dynamics(x, u)
        self._ocp.model.f_impl_expr = f_impl

        # 时间步长和预测步长
        self._ocp.dims.N = self._Herizon
        self._ocp.solver_options.tf = self._Herizon * self._step

        # 设置状态和输入约束
        self._ocp.constraints.constr_type = 'BGH'
        print(np.array(self._X_lb))
        # self._ocp.constraints.lbx = np.array(self._X_lb[3:6]+self._X_lb[10:13])
        # self._ocp.constraints.ubx = np.array(self._X_ub[3:6]+self._X_ub[10:13])
        # self._ocp.constraints.idxbx = np.array([3,4,5,10,11,12])
        # self._ocp.constraints.lbx = np.array(self._X_lb[3:13])
        # self._ocp.constraints.ubx = np.array(self._X_ub[3:13])
        # self._ocp.constraints.idxbx = np.array([3,4,5,6,7,8,9,10,11,12])

        self._ocp.constraints.lbx = np.array(self._X_lb[6:13])
        self._ocp.constraints.ubx = np.array(self._X_ub[6:13])
        self._ocp.constraints.idxbx = np.array([6,7,8,9,10,11,12])
        
        self._ocp.constraints.lbu = np.array(self._U_lb)
        self._ocp.constraints.ubu = np.array(self._U_ub)
        self._ocp.constraints.idxbu = np.array([0, 1, 2, 3])


        # 设置目标函数
        self._ocp.cost.cost_type = 'LINEAR_LS'
        self._ocp.cost.cost_type_e = 'LINEAR_LS'
        nx=self._X_dim
        nu=self._U_dim
        ny = nx + nu
        # Weighted squared error loss function q = (p_xyz, v_xyz, ,qwxyz, w_xyz,wind), r = (u1, u2, u3, u4)
        q_cost = np.array([15, 15, 15, 1e-5, 1e-5, 1e-5, 6.5, 1e-5, 1e-5, 6.5, 1.55, 1.55, 1.55, 1e-5,1e-5,1e-5])
        r_cost = np.array([1e-5, 1e-5, 1e-5, 1e-5])
        self._ocp.cost.W = np.diag(np.concatenate((q_cost, r_cost)))
        q_cost_e = np.array([10, 10, 10, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5,1e-5,1e-5])
        self._ocp.cost.W_e = np.diag(q_cost_e)
        self._ocp.cost.Vx = np.zeros((ny, nx))
        self._ocp.cost.Vx[:nx, :nx] = np.eye(nx)
        self._ocp.cost.Vu = np.zeros((ny, nu))
        self._ocp.cost.Vu[-nu:, -nu:] = np.eye(nu)
        self._ocp.cost.Vx_e = np.eye(nx)

       
        # Initial reference trajectory (will be overwritten)
        x_ref = np.zeros(nx)
        self._ocp.cost.yref = np.concatenate((x_ref, np.array([0.0, 0.0, 0.0, 0.0])))
        self._ocp.cost.yref_e = x_ref
        # Initial state (will be overwritten)
        self._ocp.constraints.x0 = x_ref

        # 配置求解器
        self._ocp.solver_options.qp_solver = 'FULL_CONDENSING_HPIPM'
        self._ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        self._ocp.solver_options.integrator_type = 'ERK'
        self._ocp.solver_options.print_level = 0
        self._ocp.solver_options.nlp_solver_type = 'SQP_RTI'

         # compile acados ocp
        json_file = os.path.join('./'+self._ocp.model.name+'_acados_ocp.json')
        self.solver = AcadosOcpSolver(self._ocp, json_file=json_file)


class TrackerMPC_AC():
    def __init__(self, quad:QuadrotorSimpleModel, solver_id='default'):
        self._quad = quad
        self._Herizon = 15
        self._step=0.1
        self._solver_id = ''.join(ch if ch.isalnum() or ch == '_' else '_' for ch in solver_id)

        self._dynamics = self._quad.dynamics()
        self._X_dim = self._dynamics.size1_in(0)
        self._U_dim = self._dynamics.size1_in(1)
        # print(  self._X_dim ,  self._U_dim )

        self._X_lb = self._quad._X_lb
        self._X_ub = self._quad._X_ub
        self._U_lb = self._quad._U_lb
        self._U_ub = self._quad._U_ub
        # 初始化 ACADOS OCP
        self._ocp = AcadosOcp()
        self.define_ocp()
        
    def solve(self, x_init, Trjp, Trjyaw):
        # print(x_init)
        if x_init is None:
            x_init = [0, 0, 0]  + [0, 0, 0] + [1, 0, 0, 0]
        # x_init = [0, 0, 0]  + [0, 0, 0] + [1, 0, 0, 0]+[0, 0, 0]
        # print(x_init)

        x_init = np.stack(x_init)
                # Set initial condition, equality constraint
        self.solver.set(0, 'lbx', x_init)
        self.solver.set(0, 'ubx', x_init)

        sqrt_q = float(np.sqrt(x_init[6] * x_init[6] + x_init[9] * x_init[9]))
        reference_quaternion = np.asarray(x_init[6:10], dtype=float)
        reference_positions = np.asarray(Trjp, dtype=float).reshape(
            self._Herizon, 3
        )
        # The planner message contains sampled positions rather than explicit
        # velocities.  Using zero velocity at every moving reference point
        # removes damping from the position loop and caused large altitude
        # oscillations during waypoint transitions.  Recover the trajectory
        # velocity with a forward difference; once the sampled trajectory
        # reaches its endpoint this naturally becomes zero.
        reference_velocities = np.zeros_like(reference_positions)
        if self._Herizon > 1:
            reference_velocities[:-1] = (
                reference_positions[1:] - reference_positions[:-1]
            ) / self._step
            reference_velocities[-1] = reference_velocities[-2]
        #set_reference_trajectory
        for j in range(self._Herizon):
            ref = reference_positions[j]
            # Convert yaw to quaternion
            q_yaw = np.array([
                np.cos(Trjyaw[j] / 2),
                0,
                0,
                np.sin(Trjyaw[j] / 2)
            ]) * sqrt_q
            # Quaternion signs are equivalent geometrically but not under the
            # LINEAR_LS component cost below.  Anchor the first reference to
            # the measured attitude, then keep the complete horizon on a
            # continuous quaternion hemisphere.
            if np.dot(q_yaw, reference_quaternion) < 0.0:
                q_yaw = -q_yaw
            reference_quaternion = q_yaw
            self.prev_q = q_yaw
            # if q_yaw[0] < 0:
            #     q_yaw = -q_yaw
            # print("q_yaw:",q_yaw)

            ref = np.concatenate((ref, reference_velocities[j], q_yaw, np.zeros(4)))
            # ref[6]=1
            self.solver.set(j, "yref", ref)
        # the last MPC node has only a state reference but no input reference
        ref = np.concatenate((reference_positions[-1], reference_velocities[-1], q_yaw))
        # ref[6]=1
        self.solver.set(self._Herizon, "yref",ref)


        # Solve OCP
        self.solver.solve()
        # Get u
        w_opt_acados = np.ndarray((self._Herizon, 4))
        x_opt_acados = np.ndarray((self._Herizon + 1, len(x_init)))
        x_opt_acados[0, :] = self.solver.get(0, "x")
        for i in range(self._Herizon):
            w_opt_acados[i, :] = self.solver.get(i, "u")
            x_opt_acados[i + 1, :] = self.solver.get(i + 1, "x")

        w_opt_acados = np.reshape(w_opt_acados, (-1))
        return w_opt_acados,x_opt_acados
        
        
    def define_ocp(self):
        self.prev_q=None
        # ocp.acados_include_path = acados_source_path + '/include'
        # ocp.acados_lib_path = acados_source_path + '/lib'
        # 动力学
        x = ca.SX.sym('x', self._X_dim)
        u = ca.SX.sym('u', self._U_dim)
        x_dot = ca.SX.sym('x_dot', x.size1())
        f_impl = x_dot -self._dynamics(x, u)
        # 设置 OCP 模型
        self._ocp.model.name = 'quadrotor_mpc_simple_' + self._solver_id
        self._ocp.code_gen_opts.code_export_directory = os.path.expanduser(
            '~/.ros/c_generated_code_' + self._ocp.model.name)
        self._ocp.model.x = x
        self._ocp.model.u = u
        self._ocp.model.xdot = x_dot  
        self._ocp.model.f_expl_expr =  self._dynamics(x, u)
        self._ocp.model.f_impl_expr = f_impl

        # 时间步长和预测步长
        self._ocp.dims.N = self._Herizon
        self._ocp.solver_options.tf = self._Herizon * self._step

        # 设置状态和输入约束
        self._ocp.constraints.constr_type = 'BGH'
        print(np.array(self._X_lb))
        # self._ocp.constraints.lbx = np.array(self._X_lb[3:6]+self._X_lb[10:13])
        # self._ocp.constraints.ubx = np.array(self._X_ub[3:6]+self._X_ub[10:13])
        # self._ocp.constraints.idxbx = np.array([3,4,5,10,11,12])
        # self._ocp.constraints.lbx = np.array(self._X_lb[3:13])
        # self._ocp.constraints.ubx = np.array(self._X_ub[3:13])
        # self._ocp.constraints.idxbx = np.array([3,4,5,6,7,8,9,10,11,12])

        self._ocp.constraints.lbx = np.array(self._X_lb[3:10])
        self._ocp.constraints.ubx = np.array(self._X_ub[3:10])
        self._ocp.constraints.idxbx = np.array([3,4,5,6,7,8,9])
        
        self._ocp.constraints.lbu = np.array(self._U_lb)
        self._ocp.constraints.ubu = np.array(self._U_ub)
        self._ocp.constraints.idxbu = np.array([0, 1, 2, 3])


        # 设置目标函数
        self._ocp.cost.cost_type = 'LINEAR_LS'
        self._ocp.cost.cost_type_e = 'LINEAR_LS'
        nx=self._X_dim
        nu=self._U_dim
        ny = nx + nu
        # Weighted squared error loss function q = (p_xyz, v_xyz, qwxyz), r = (a_z, omega_x,omega_y, omega_xy)
        q_cost = np.array([15, 15, 15, 1e-5, 1e-5, 1e-5, 6.5, 1e-5, 1e-5, 6.5])
        r_cost = np.array([0.01, 0.15, 0.15, 0.15])
        self._ocp.cost.W = np.diag(np.concatenate((q_cost, r_cost)))
        q_cost_e = np.array([10, 10, 10, 1e-5, 1e-5, 1e-5, 0.5, 1e-5, 1e-5, 0.5])
        self._ocp.cost.W_e = np.diag(q_cost_e)
        self._ocp.cost.Vx = np.zeros((ny, nx))
        self._ocp.cost.Vx[:nx, :nx] = np.eye(nx)
        self._ocp.cost.Vu = np.zeros((ny, nu))
        self._ocp.cost.Vu[-nu:, -nu:] = np.eye(nu)
        self._ocp.cost.Vx_e = np.eye(nx)

       
        # Initial reference trajectory (will be overwritten)
        x_ref = np.zeros(nx)
        self._ocp.cost.yref = np.concatenate((x_ref, np.array([0.0, 0.0, 0.0, 0.0])))
        self._ocp.cost.yref_e = x_ref
        # Initial state (will be overwritten)
        self._ocp.constraints.x0 = x_ref

        # 配置求解器
        self._ocp.solver_options.qp_solver = 'FULL_CONDENSING_HPIPM'
        self._ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        self._ocp.solver_options.integrator_type = 'ERK'
        self._ocp.solver_options.print_level = 0
        self._ocp.solver_options.nlp_solver_type = 'SQP_RTI'
        # self._ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        # self._ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        # self._ocp.solver_options.integrator_type = 'ERK'
        # self._ocp.solver_options.print_level = 0
        # self._ocp.solver_options.nlp_solver_type = 'SQP_RTI'
         # compile acados ocp
        json_file = os.path.join(self._ocp.code_gen_opts.code_export_directory,
                                 self._ocp.model.name + '_acados_ocp.json')
        self.solver = AcadosOcpSolver(self._ocp, json_file=json_file)
if __name__ == "__main__":
    # quad = QuadrotorModel(BASEPATH+"quad/quad_robomaster.yaml")
    # tracker = TrackerMPC_FULL_AC(quad)
    quad = QuadrotorSimpleModel(BASEPATH+"quad/quad_sim.yaml")
    tracker = TrackerMPC_AC(quad)
    # tracker.define_opt()
    # print('finish define')
    # tracker._opt_solver.generate_dependencies("tracker_mpc_robomaster.c")
    # print('begin gcc')
    # system('gcc -fPIC -shared -O3 ' + "tracker_mpc_robomaster.c" + ' -o ' + '/home/fractal/IntelligentUAVChampionshipBase/flm_dev/src/control/uav_control/script/function_model/generated/track_mpc_robomaster.so')
    tracker.define_ocp()
    
