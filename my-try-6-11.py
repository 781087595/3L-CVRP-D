from cycler import cycler
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import csv
import itertools
import gurobipy as gp
from draw_packing_result import draw_packing_result
# 定义常量 GRB 以简化代码书写
GRB = gp.GRB

def generate_symmetric_matrix(n, low, high):
    matrix = np.random.randint(low=low, high=high, size=(n, n))
    sym_matrix = matrix + matrix.T
    np.fill_diagonal(sym_matrix, 0)
    sym_matrix[0][n-1] = 0
    sym_matrix[n-1][0] = 0
    return sym_matrix

# i,j,m,i^\prime	indices for nodes
# k	index for trucks，drones
# g	index for overlapping, g\in{1,2,\cdots,6}
# f	index for LIFO, f\in{1,2,\cdots,6}
# 以上索引

# 参数定义
# n	the number of customers
n = 6
# z	the number of trucks/drones
z = 2
# Q^t	the maximum load weight of a truck
Q_t = 100
# Q^d	the maximum load weight of a drone
Q_d = 6
# t_{ij}	truck traveling time between nodes i and j

t = generate_symmetric_matrix(n+2, 2, 5)
# t_{ij}^\prime	drone traveling time between nodes i and j
t_prime = generate_symmetric_matrix(n+2, 1, 2)
# T_1	truck service time
T_1 = 8
# T_2	drone service time
T_2 = 1
# T_3	drone launch time
T_3 = 1
# T_4	drone recovery time
T_4 = 1
# \bar{T}	maximum endurance of the drone
T_bar = 30
# q_i	load amount of package needed of customer i
q = np.random.randint(low=2, high=7, size=n)
# L,W,H	length, width, and height of carriage of truck
L = 12
W = 8
H = 10

# L^\prime,W^\prime,H^\prime	length, width, and height of carriage of drone
L_prime = 4
W_prime = 4
H_prime = 4
# l_i,w_i,h_i	length, width, and height of package needed of customer i
l = np.random.randint(low=3, high=5, size=n)
w = np.random.randint(low=3, high=5, size=n)
h = np.random.randint(low=3, high=5, size=n)
# M	a large number
M = 10000

# 集合
# N	set of all nodes, i,j\in{0,1,2,\cdots,n+1}
# N = np.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11])
N = np.arange(n+2)
# C	set of customers, i,j\in{1,2,\cdots,n}
# C = np.array([1, 2, 3, 4, 5, 6, 7, 8, 9, 10])
C = np.arange(1, n+1)
# V	set of all trucks carrying drones, k\in{1,2,\cdots,z}
# V = np.array([1, 2, 3])
V = np.arange(1, z+1)

# 创建PuLP模型
# problem = pl.LpProblem("3L-CVRP-D", pl.LpMinimize)
# 创建一个新的模型
model = gp.Model("3LCVRPD")
# 定义决策变量

# x_{ij}^k	1, if arc (i,j) is traversed by truck k,0 otherwise
# x_{ij}^k\in{0,1},\forallk\inV,i\in{N:i\neqn+1},j\in{N:j\neq0,j\neqi}
# x = pl.LpVariable.dicts("x", [(i, j, k) for i in N[:-1] for j in N[1:] if j != i for k in V], 0, 1, pl.LpBinary)
x = model.addVars([(i, j, k) for i in N[:-1] for j in N[1:] if j != i for k in V], vtype=GRB.BINARY)
# x = pl.LpVariable.dicts('x', [(i, j, k) for i in range(1, n+1) for j in range(1, n+1) if i != j for k in range(1, K+1)], lowBound=0, upBound=1, cat=pl.LpInteger)

# y_{imj}^k	1, if sortie (i,m,j) is performed by the drone belonging to truck k,0 otherwise
# y_{imj}^k\in0,1,\forallk\inV,i\inC,m\in{C:m\neqi},j\in{C:j\neqi,j\neqm}
# y = pl.LpVariable.dicts("y", [(i, m, j, k) for i in C for m in C if m != i for j in C if j != i and j != m for k in V], 0, 1, pl.LpBinary)
y = model.addVars([(i, m, j, k) for i in C for m in C if m != i for j in C if j != i and j != m for k in V], vtype=GRB.BINARY)
# y = pl.LpVariable.dicts('y', [(i, m, j, k) for i in range(1, n+1) for m in range(1, n+1) if m != i for j in range(1, n+1) if j != i and j != m for k in range(1, K+1)], lowBound=0, upBound=1, cat=pl.LpInteger)

# a_i^k	arrival time of truck k at node i
# a = pl.LpVariable.dicts("a", [(i, k) for i in N for k in V], 0, None, pl.LpContinuous)
a = model.addVars([(i, k) for i in N for k in V], lb=0, vtype=GRB.CONTINUOUS)
# a = pl.LpVariable.dicts('a', [(i, k) for i in range(n+2) for k in range(1, K+1)], lowBound=0, cat=pl.LpContinuous)
# a_0^k=0,\forallk\inV
# for k in V:
#     problem += a[0, k] == 0
for k in V:
    model.addConstr(a[0, k] == 0)
# a_i^{\prime k}	arrival time of the drone belonging to truck k at node i
# a_ = pl.LpVariable.dicts("a_", [(i, k) for i in N for k in V], 0, None, pl.LpContinuous)
a_ = model.addVars([(i, k) for i in N for k in V], lb=0, vtype=GRB.CONTINUOUS)
# a_i^k\geq0,a_i^{\prime k}\geq0,\forallk\inV,i\inN 定义决策变量的时候已经满足非负
# for k in range(z):
#     for i in range(n + 2):
#         m += a[i, k] >= 0
#         m += a_[i, k] >= 0

# \mathbit{S}_\mathbit{i}^\mathbit{k}	1, if package needed by customer i is served by truck k,0 otherwise
# S_i^k\in{0,1},\forallk\inV,i\inC
# S = pl.LpVariable.dicts("S", [(i, k) for i in C for k in V], 0, 1, pl.LpBinary)
S = model.addVars([(i, k) for i in C for k in V], vtype=GRB.BINARY)
# \propto_{ik},\beta_{ik},\gamma_{ik}	placement coordinates of the left front vertex of package needed by customer i inside truck k
# \propto_{ik},\beta_{ik},\gamma_{ik}\geq0,\forallk\inV,i\inC
# propto = pl.LpVariable.dicts("propto", [(i, k) for i in C for k in V], 0, None, pl.LpContinuous)
# beta = pl.LpVariable.dicts("beta", [(i, k) for i in C for k in V], 0, None, pl.LpContinuous)
# gamma = pl.LpVariable.dicts("gamma", [(i, k) for i in C for k in V], 0, None, pl.LpContinuous)
propto = model.addVars([(i, k) for i in C for k in V], lb=0, vtype=GRB.CONTINUOUS)
beta = model.addVars([(i, k) for i in C for k in V], lb=0, vtype=GRB.CONTINUOUS)
gamma = model.addVars([(i, k) for i in C for k in V], lb=0, vtype=GRB.CONTINUOUS)

# \delta_{gij}^k	a binary variable for non-overlapping of package needed by customer i and j for truck k
# \delta_{gij}^k\in{0,1},\forallk\inV,i\inC,j\in{C:j\neqi}
# delta = pl.LpVariable.dicts("delta", [(g, i, j, k) for g in range(1, 7) for i in C for j in C if j != i for k in V], 0, 1, pl.LpBinary)
delta = model.addVars([(g, i, j, k) for g in range(1, 7) for i in C for j in C if j != i for k in V], vtype=GRB.BINARY)
# \tau	continuous variable that indicates total completion time
# tau = pl.LpVariable("tau", 0, None, pl.LpContinuous)
tau = model.addVar(lb=0, vtype=GRB.CONTINUOUS)
# \eta_{fij}^k	a binary variable for LIFO
# \eta_{fij}^k\in{0,1},\forallk\inV,i\inC,j\in{C:j\neqi}
# eta = pl.LpVariable.dicts("eta", [(f, i, j, k) for f in range(1, 4) for i in C for j in C if j != i for k in V], 0, 1, pl.LpBinary)
eta = model.addVars([(f, i, j, k) for f in range(1, 4) for i in C for j in C if j != i for k in V], vtype=GRB.BINARY)


# 定义目标函数
# Minimize \tau+k∈V,j∈C,j≠0  x0jk
# problem += tau + pl.lpSum(x[0, j, k] for k in V for j in C)
model.setObjective(tau + gp.quicksum(x[0, j, k] for j in C for k in V), GRB.MINIMIZE)
# model.setObjective(tau, GRB.MINIMIZE)
# model.setObjective(tau, GRB.MINIMIZE)
# 添加约束条件

# （2）计算总的完成时间:
# \tau\geq a_{n+1}^k,\forall k\in V
# for k in V:
#     problem += tau >= a[n + 1, k]
for k in V:
    model.addConstr(tau >= a[n + 1, k])
# （3）所有客户只访问一次:
# k∈V,i∈N,i≠n+1,i≠m   ximk+k∈V,i∈C,i≠mj∈C,j≠mi≠j  yimjk=1,∀m∈C
# for m in C:
#     problem += pl.lpSum(x[i, m, k] for i in N[:-1] if i != m for k in V) + pl.lpSum(y[i, m, j, k] for i in C if i != m for j in C if j != m and j != i for k in V) == 1
for m in C:
    model.addConstr(gp.quicksum(x[i, m, k] for i in N[:-1] if i != m for k in V) + gp.quicksum(y[i, m, j, k] for i in C if i != m for j in C if j != m and j != i for k in V) == 1)
# （4）车辆从仓库出发且最后回到仓库:
# 注：x_{0(n+1)}^k=1表示第k辆卡车没有使用
# \sum_{j\in N,j\neq0}\hairsp\hairsp x_{0j}^k=1,\forall k\in V
# for k in V:
#     problem += pl.lpSum(x[0, j, k] for j in N[1:]) == 1
# # \sum_{i\in N,i\neq n+1}\hairsp\hairsp x_{i(n+1)}^k=1,\forall k\in V
# for k in V:
#     problem += pl.lpSum(x[i, n + 1, k] for i in N[:-1]) == 1
# for k in V:
#     #（4）车辆从仓库出发且最后回到仓库:
#     problem += pl.lpSum(x[0, j, k] for j in N[1:]) == 1
#     problem += pl.lpSum(x[i, n + 1, k] for i in N[:-1]) == 1
for k in V:
    model.addConstr(gp.quicksum(x[0, j, k] for j in N[1:]) == 1)
    model.addConstr(gp.quicksum(x[i, n + 1, k] for i in N[:-1]) == 1)    

# （5）流量守恒：车辆到达某顾客，必须离开该顾客到达下一顾客:
# \sum_{i\in N,i\neq n+1}\hairsp\hairsp x_{ij}^k-\sum_{i\in N,i\neq0}\hairsp\hairsp x_{ji}^k=0,\forall j\in C,\forall k\in V
# for j in C:
#     for k in V:
#         problem += pl.lpSum(x[i, j, k] for i in N[:-1] if i!= j) - pl.lpSum(x[j, i, k] for i in N[1:] if i!= j) == 0
for j in C:
    for k in V:
        model.addConstr(gp.quicksum(x[i, j, k] for i in N[:-1] if i!= j) - gp.quicksum(x[j, i, k] for i in N[1:] if i!= j) == 0)

# （6）消除子回路约束:
# i∈Sj∈S  xijk≤|S|-1,∀S⊆C,k∈V
# for k in range(z):
#     for S in range(1, n + 1):
#         m += pl.lpSum(x[i, j, k] for i in range(n + 2) if i in S for j in range(n + 2) if j in S) <= len(S) - 1
# for k in V:
#     for length in range(2, len(C)+1):
#         for subset in itertools.combinations(C, length):
#             problem += pl.lpSum(x[i, j, k] for i in subset for j in subset if j != i) <= len(subset) - 1
for k in V:
    for length in range(2, len(C)+1):
        for subset in itertools.combinations(C, length):
            model.addConstr(gp.quicksum(x[i, j, k] for i in subset for j in subset if j != i) <= len(subset) - 1)  
# （7）载重约束:
# 卡车载重：
# i∈N,i≠n+1j∈C,j≠i qj xijk+i∈Cm∈C,m≠ij∈C,j≠m,j≠i qm yimjk≤Qk,∀k∈V
# for k in V:
#     problem += pl.lpSum(q[j-1] * x[i, j, k] for i in N[:-1] for j in C if j != i) + pl.lpSum(q[m-1] * y[i, m, j, k] for i in C for m in C if m != i for j in C if j != m and j != i) <= Q_t
for k in V:
    model.addConstr(gp.quicksum(q[j-1] * x[i, j, k] for i in N[:-1] for j in C if j != i) + gp.quicksum(q[m-1] * y[i, m, j, k] for i in C for m in C if m != i for j in C if j != m and j != i) <= Q_t)

# # 无人机载重：
# # q_my_{imj}^k⩽Qd,∀k∈V,i∈C,m∈{C:m≠i},j∈{C:j≠i,j≠m}
# for k in V:
#     for i in C:
#         for m in C:
#             if m != i:
#                 for j in C:
#                     if j != i and j != m:
#                         problem += q[m-1] * y[i, m, j, k] <= Q_d

# # （8）无人机体积约束：
# # l_my_{imj}^k⩽L',∀k∈V,i∈C,m∈{C:m≠i},j∈{C:j≠i,j≠m}
# for k in V:
#     for i in C:
#         for m in C:
#             if m != i:
#                 for j in C:
#                     if j != i and j != m:
#                         problem += l[m-1] * y[i, m, j, k] <= L_prime
#                         problem += w[m-1] * y[i, m, j, k] <= W_prime
#                         problem += h[m-1] * y[i, m, j, k] <= H_prime
# for k in V:
#     for i in C:
#         for m in C:
#             if m != i:
#                 for j in C:
#                     if j != i and j != m:
#                         # 无人机载重：
#                         problem += q[m-1] * y[i, m, j, k] <= Q_d
#                         # （8）无人机体积约束：
#                         problem += l[m-1] * y[i, m, j, k] <= L_prime
#                         problem += w[m-1] * y[i, m, j, k] <= W_prime
#                         problem += h[m-1] * y[i, m, j, k] <= H_prime
#                         # （10）无人机的发射点和回收点由携带该无人机的卡车先后服务
#                         problem += 2 * y[i, m, j, k] <= pl.lpSum(x[i, s, k] for s in C if s != i) + pl.lpSum(x[s, j, k] for s in C if s != j)
#                         problem += a[j, k] >= a[i, k] + M * (y[i, m, j, k] - 1)

for k in V:
    for i in C:
        for m in C:
            if m != i:
                for j in C:
                    if j != i and j != m:
                        # 无人机载重：
                        model.addConstr(q[m-1] * y[i, m, j, k] <= Q_d)
                        # （8）无人机体积约束：
                        model.addConstr(l[m-1] * y[i, m, j, k] <= L_prime)
                        model.addConstr(w[m-1] * y[i, m, j, k] <= W_prime)
                        model.addConstr(h[m-1] * y[i, m, j, k] <= H_prime)
                        # （10）无人机的发射点和回收点由携带该无人机的卡车先后服务
                        model.addConstr(2 * y[i, m, j, k] <= gp.quicksum(x[i, s, k] for s in C if s != i) + gp.quicksum(x[s, j, k] for s in C if s != j))
                        model.addConstr(a[j, k] >= a[i, k] + M * (y[i, m, j, k] - 1))
# （9）无人机不可重复发射或者重复回收:
# m∈C,m≠ij∈C,j≠m,j≠i  yimjk≤1,∀k∈V,i∈C
# for k in V:
#     for i in C:
#         problem += pl.lpSum(y[i, m, j, k] for m in C if m != i for j in C if j != i and j != m) <= 1
for k in V:
    for i in C:
        model.addConstr(gp.quicksum(y[i, m, j, k] for m in C if m != i for j in C if j != i and j != m) <= 1)
# m∈C,m≠ii∈C,i≠m,i≠j  yimjk≤1,∀k∈V,j∈C
# for k in V:
#     for j in C:
#         problem += pl.lpSum(y[i, m, j, k] for m in C if m != i for j in C if j != i and j != m) <= 1
#         # （11）卡车到达某节点的时间:
#         # 1-	前一个节点是仓库
#         problem += a[j, k] >= a[0, k] + t[0, j] + M * (x[0, j, k] - 1)
for k in V:
    for j in C:
        model.addConstr(gp.quicksum(y[i, m, j, k] for m in C if m != i for j in C if j != i and j != m) <= 1)
        # （11）卡车到达某节点的时间:
        # 1-	前一个节点是仓库
        model.addConstr(a[j, k] >= a[0, k] + t[0, j] + M * (x[0, j, k] - 1))
# # （10）无人机的发射点和回收点由携带该无人机的卡车先后服务
# # {2y}_{imj}^k\les∈C,s≠i   xisk+s∈C,s≠j   xsjk,\forall k\in V,i\inC,m\in{C:m\neq i},j\in{C:j\neq i,j\neq w}
# for k in V:
#     for i in C:
#         for m in C:
#             if m != i:
#                 for j in C:
#                     if j != i and j != m:
#                         problem += 2 * y[i, m, j, k] <= pl.lpSum(x[i, s, k] for s in C if s != i) + pl.lpSum(x[s, j, k] for s in C if s != j)

# # a_j^k\geq a_i^k+M\left(\hairsp\hairsp y_{imj}^k-1\right),\forall k\in V,i\inC,m\in{C:m\neq i},j\in{C:j\neq i,j\neq m}
# for k in V:
#     for i in C:
#         for m in C:
#             if m != i:
#                 for j in C:
#                     if j != i and j != m:
#                         problem += a[j, k] >= a[i, k] + M * (y[i, m, j, k] - 1)
# 时间约束：
# （11）卡车到达某节点的时间:
# 1-	前一个节点是仓库
# # a_j^k\geq a_0^k+t_{ij}+M(\hairsp x_{0j}^k-1),\forall k\inV,j\inC
# for k in V:
#     for j in C:
#         problem += a[j, k] >= a[0, k] + t[0, j] + M * (x[0, j, k] - 1)
# 2-	前一个节点是顾客点且没有无人机发射和回收
# a_j^k\geq a_i^k+T_1+t_{ij}+M(\hairsp x_{ij}^k-1),\forall k\inV,i\inC,j\in{N:j\neq0,j\neq i}
# for k in V:
#     for i in C:
#         for j in N[1:]:
#             if j != i:
#                 problem += a[j, k] >= a[i, k] + T_1 + t[i, j] + M * (x[i, j, k] - 1)
#                 # 3-	前一个节点是顾客点且有无人机发射，没有无人机回收
#                 problem += a[j, k] >= a[i, k] + T_1 + T_3 + t[i, j] + M * (x[i, j, k] - 1) + M * (pl.lpSum(y[i, m, i_, k] for m in C if m != i for i_ in C if i_ != i and i_ != m) - 1)
#                 # 4-	前一个节点是顾客点且有无人机回收，没有无人机发射
#                 problem += a[j, k] >= a[i, k] + T_1 + T_4 + t[i, j] + M * (x[i, j, k] - 1) + M * (pl.lpSum(y[i_, m, i, k] for m in C if m != i for i_ in C if i_ != i and i_ != m) - 1)
#                 problem += a[j, k] >= a_[i, k] + T_4 + t[i, j] + M * (x[i, j, k] - 1) + M * (pl.lpSum(y[i_, m, i, k] for m in C if m != i for i_ in C if i_ != i and i_ != m) - 1)
#                 # 5-	前一个节点是顾客点且有无人机发射，有无人机回收
#                 problem += a[j, k] >= a[i, k] + T_1 +T_4 + T_3 + t[i, j] + M * (x[i, j, k] - 1) + M * (pl.lpSum(y[i, m, i_, k] for m in C if m != i for i_ in C if i_ != i and i_ != m) + pl.lpSum(y[i_, m, i, k] for m in C if m != i for i_ in C if i_ != i and i_ != m) - 2)
#                 problem += a[j, k] >= a_[i, k] + T_4 + T_3 + t[i, j] + M * (x[i, j, k] - 1) + M * (pl.lpSum(y[i, m, i_, k] for m in C if m != i for i_ in C if i_ != i and i_ != m) + pl.lpSum(y[i_, m, i, k] for m in C if m != i for i_ in C if i_ != i and i_ != m) - 2)

for k in V:
    for i in C:
        for j in N[1:]:
            if j != i:
                model.addConstr(a[j, k] >= a[i, k] + T_1 + t[i, j] + M * (x[i, j, k] - 1))
                # 3-	前一个节点是顾客点且有无人机发射，没有无人机回收
                model.addConstr(a[j, k] >= a[i, k] + T_1 + T_3 + t[i, j] + M * (x[i, j, k] - 1) + M * (gp.quicksum(y[i, m, i_, k] for m in C if m != i for i_ in C if i_ != i and i_ != m) - 1))
                # 4-	前一个节点是顾客点且有无人机回收，没有无人机发射
                model.addConstr(a[j, k] >= a[i, k] + T_1 + T_4 + t[i, j] + M * (x[i, j, k] - 1) + M * (gp.quicksum(y[i_, m, i, k] for m in C if m != i for i_ in C if i_ != i and i_ != m) - 1))
                model.addConstr(a[j, k] >= a_[i, k] + T_4 + t[i, j] + M * (x[i, j, k] - 1) + M * (gp.quicksum(y[i_, m, i, k] for m in C if m != i for i_ in C if i_ != i and i_ != m) - 1))
                # 5-	前一个节点是顾客点且有无人机发射，有无人机回收
                model.addConstr(a[j, k] >= a[i, k] + T_1 +T_4 + T_3 + t[i, j] + M * (x[i, j, k] - 1) + M * (gp.quicksum(y[i, m, i_, k] for m in C if m != i for i_ in C if i_ != i and i_ != m) + gp.quicksum(y[i_, m, i, k] for m in C if m != i for i_ in C if i_ != i and i_ != m) - 2))
# # 3-	前一个节点是顾客点且有无人机发射，没有无人机回收
# # a_j^k\geq a_i^k+T_1+T_3+t_{ij}+M\left(\hairsp x_{ij}^k-1\right)+M(\hairspm∈C,m≠ii'∈C,i'≠m,i'≠i  yimi'k-1),∀k∈V,i∈C,j∈{N:j≠0,j≠i}
# for k in V:
#     for i in C:
#         for j in N[1:]:
#             if j != i:
#                 problem += a[j, k] >= a[i, k] + T_1 + T_3 + t[i, j] + M * (x[i, j, k] - 1) + M * (pl.lpSum(y[i, m, i_, k] for m in C if m != i for i_ in C if i_ != i and i_ != m) - 1)
# # 4-	前一个节点是顾客点且有无人机回收，没有无人机发射
# # a_j^k\geq a_i^k+T_1+T_4+t_{ij}+M\left(\hairsp x_{ij}^k-1\right)+M(\hairspi'∈C,i'≠im∈C,m≠i,m≠i'  yi'mik-1),∀k∈V,i∈C,j∈{N:j≠0,j≠i}
# for k in V:
#     for i in C:
#         for j in N[1:]:
#             if j != i:
#                 problem += a[j, k] >= a[i, k] + T_1 + T_4 + t[i, j] + M * (x[i, j, k] - 1) + M * (pl.lpSum(y[i_, m, i, k] for m in C if m != i for i_ in C if i_ != i and i_ != m) - 1)
# a_j^k\geq a_i^{\prime k}+T_4+t_{ij}+M\left(\hairsp x_{ij}^k-1\right)+M(\hairspi'∈C,i'≠im∈C,m≠i,m≠i'  yi'mik-1),∀k∈V,i∈C,j∈{N:j≠0,j≠i}
# for k in V:
#     for i in C:
#         for j in N[1:]:
#             if j != i:
#                 problem += a[j, k] >= a_[i, k] + T_4 + t[i, j] + M * (x[i, j, k] - 1) + M * (pl.lpSum(y[i_, m, i, k] for m in C if m != i for i_ in C if i_ != i and i_ != m) - 1)
# # 5-	前一个节点是顾客点且有无人机发射，有无人机回收
# # a_j^k\geq a_i^k+T_1+T_4+T_3+t_{ij}+M\left(\hairsp x_{ij}^k-1\right)+M m∈C,m≠ii'∈C,i'≠m,i'≠i  yimi'k+i'∈C,i'≠im∈C,m≠i,m≠i'  yi'mik-2,\forall k\inV,i\inC,j\in{N:j\neq0,j\neq i}
# for k in V:
#     for i in C:
#         for j in N[1:]:
#             if j != i:
#                 problem += a[j, k] >= a[i, k] + T_1 +T_4 + T_3 + t[i, j] + M * (x[i, j, k] - 1) + M * (pl.lpSum(y[i, m, i_, k] for m in C if m != i for i_ in C if i_ != i and i_ != m) + pl.lpSum(y[i_, m, i, k] for m in C if m != i for i_ in C if i_ != i and i_ != m) - 2)
# # a_j^k\geq a_i^{\prime k}+T_4+T_3+t_{ij}+M\left(\hairsp x_{ij}^k-1\right)+M m∈C,m≠ii'∈C,i'≠m,i'≠i  yimi'k+i'∈C,i'≠im∈C,m≠i,m≠i'  yi'mik-2,\forall k\inV,i\inC,j\in{N:j\neq0,j\neq i}
# for k in V:
#     for i in C:
#         for j in N[1:]:
#             if j != i:
#                 problem += a[j, k] >= a_[i, k] + T_4 + T_3 + t[i, j] + M * (x[i, j, k] - 1) + M * (pl.lpSum(y[i, m, i_, k] for m in C if m != i for i_ in C if i_ != i and i_ != m) + pl.lpSum(y[i_, m, i, k] for m in C if m != i for i_ in C if i_ != i and i_ != m) - 2)

# （12）无人机到达某节点的时间:
# 1-	当前节点由无人机服务
# 1-1发射节点i处没有无人机回收
# a_j^{\prime k}\geq a_i^k+T_1+T_3+t_{ij}^\prime+M m∈C,m≠i,m≠j  yijmk-1,\forall k\in V,i\inC,j\in{C:j\neq i}
# for k in V:
#     for i in C:
#         for j in C:
#             if j != i:
#                 problem += a_[j, k] >= a[i, k] + T_1 + T_3 + t_prime[i, j] + M * (pl.lpSum(y[i, j, m, k] for m in C if m != i and m != j) - 1)
#                 # 1-2发射节点i处有无人机回收
#                 problem += a_[j, k] >= a[i, k] + T_1 + T_4 + T_3 + t_prime[i, j] + M * (pl.lpSum(y[i, j, m, k] for m in C if m != i and m != j) - 1) + M * (pl.lpSum(y[i_, m, i, k] for i_ in C if i_ != i for m in C if m != i and m != i_) - 1)
#                 problem += a_[j, k] >= a_[i, k] + T_4 + T_3 + t_prime[i, j] + M * (pl.lpSum(y[i, j, m, k] for m in C if m != i and m != j) - 1) + M * (pl.lpSum(y[i_, m, i, k] for i_ in C if i_ != i for m in C if m != i and m != i_) - 1)
#                 # 2-	当前节点为无人机回收点
#                 problem += a_[j, k] >= a_[i, k] + T_2 + t_prime[i, j] + M * (pl.lpSum(y[m, i, j, k] for m in C if m != i and m != j) - 1)
for k in V:
    for i in C:
        for j in C:
            if j != i:
                model.addConstr(a_[j, k] >= a[i, k] + T_1 + T_3 + t_prime[i, j] + M * (gp.quicksum(y[i, j, m, k] for m in C if m != i and m != j) - 1))
                # 1-2发射节点i处有无人机回收
                model.addConstr(a_[j, k] >= a[i, k] + T_1 + T_4 + T_3 + t_prime[i, j] + M * (gp.quicksum(y[i, j, m, k] for m in C if m != i and m != j) - 1) + M * (gp.quicksum(y[i_, m, i, k] for i_ in C if i_ != i for m in C if m != i and m != i_) - 1))
                model.addConstr(a_[j, k] >= a_[i, k] + T_4 + T_3 + t_prime[i, j] + M * (gp.quicksum(y[i, j, m, k] for m in C if m != i and m != j) - 1) + M * (gp.quicksum(y[i_, m, i, k] for i_ in C if i_ != i for m in C if m != i and m != i_) - 1))
                # 2-	当前节点为无人机回收点
                model.addConstr(a_[j, k] >= a_[i, k] + T_2 + t_prime[i, j] + M * (gp.quicksum(y[m, i, j, k] for m in C if m != i and m != j) - 1))

# # 1-2发射节点i处有无人机回收
# # a_j^{\prime k}\geq a_i^k+T_1+T_4+T_3+t_{ij}^\prime+M m∈C,m≠i,m≠j  yijmk-1+M(\hairspi'∈C,i'≠im∈C,m≠i,m≠i'  yi'mik-1),∀k∈V,i∈C,j∈{C:j≠i}
# for k in V:
#     for i in C:
#         for j in C:
#             if j != i:
#                 problem += a_[j, k] >= a[i, k] + T_1 + T_4 + T_3 + t_prime[i, j] + M * (pl.lpSum(y[i, j, m, k] for m in C if m != i and m != j) - 1) + M * (pl.lpSum(y[i_, m, i, k] for i_ in C if i_ != i for m in C if m != i and m != i_) - 1)
# # a_j^{\prime k}\geq a_i^{\prime k}+T_4+T_3+t_{ij}^\prime+M m∈C,m≠i,m≠j  yijmk-1+M(\hairspi'∈C,i'≠im∈C,m≠i,m≠i'  yi'mik-1),∀k∈V,i∈C,j∈{C:j≠i}
# for k in V:
#     for i in C:
#         for j in C:
#             if j != i:
#                 problem += a_[j, k] >= a_[i, k] + T_4 + T_3 + t_prime[i, j] + M * (pl.lpSum(y[i, j, m, k] for m in C if m != i and m != j) - 1) + M * (pl.lpSum(y[i_, m, i, k] for i_ in C if i_ != i for m in C if m != i and m != i_) - 1)
# # 2-	当前节点为无人机回收点
# # a_j^{\prime k}\geq a_i^{\prime k}+T_2+t_{ij}^\prime+M m∈C,m≠i,m≠j  ymijk-1,\forall k\inV,i\inC,j\in{C:j\neq i}
# for k in V:
#     for i in C:
#         for j in C:
#             if j != i:
#                 problem += a_[j, k] >= a_[i, k] + T_2 + t_prime[i, j] + M * (pl.lpSum(y[m, i, j, k] for m in C if m != i and m != j) - 1)

# （13）无人机的出行加上悬停等待满足续航限制
# \left(t_{im}^\prime+t_{mj}^\prime\right)y_{imj}^k⩽T,∀k∈V,i∈N:i≠n+1,m\in{C:m\neq i},j\in{N:j\neq0,j\neq i,j\neq m}
# for k in V:
#     for i in C:
#         for m in C:
#             if m != i:
#                 for j in C:
#                     if j != i and j != m:
#                         problem += (t_prime[i, m] + t_prime[m, j]) * y[i, m, j, k] <= T_bar
#                         problem += (t_prime[i, m] + t_prime[m, j]) * y[i, m, j, k] + a[j, k] + T_1 - a_[j, k] <= T_bar
for k in V:
    for i in C:
        for m in C:
            if m != i:
                for j in C:
                    if j != i and j != m:
                        model.addConstr((t_prime[i, m] + t_prime[m, j]) * y[i, m, j, k] <= T_bar)
                        model.addConstr((t_prime[i, m] + t_prime[m, j]) * y[i, m, j, k] + a[j, k] + T_1 - a_[j, k] <= T_bar)

# # \left(t_{im}^\prime+t_{mj}^\prime\right)y_{imj}^k+a_j^k+T_1-a_j^{\prime k}⩽T,\forallk\inV,i\inC,m\in{C:m\neq i},j\in{C:j\neq i,j\neq m}
# for k in V:
#     for i in C:
#         for m in C:
#             if m != i:
#                 for j in C:
#                     if j != i and j != m:
#                         problem += (t_prime[i, m] + t_prime[m, j]) * y[i, m, j, k] + a[j, k] + T_1 - a_[j, k] <= T_bar
# （14）确定顾客i的物品是否被卡车k装载:
# \sum_{j\in N,j\neq0}\hairsp\hairsp x_{ij}^k+m∈C,m≠ij∈C,j≠m,j≠i  ymijk=Sik,∀k∈V,∀i∈C
# for k in V:
#     for i in C:
#         problem += pl.lpSum(x[i, j, k] for j in N[1:] if j != i) + pl.lpSum(y[m, i, j, k] for m in C if m != i for j in C if j != m and j != i) == S[i, k]
for k in V:
    for i in C:
        model.addConstr(gp.quicksum(x[i, j, k] for j in N[1:] if j != i) + gp.quicksum(y[m, i, j, k] for m in C if m != i for j in C if j != m and j != i) == S[i, k])

# 装箱约束：
# （15）顾客i的物品完全在车厢内:
# \propto_{ik}-L+l_i\le\left(1-S_i^k\right)M{\ }\forall i\in C,\forall k\in V
# for k in V:
#     for i in C:
#         problem += propto[i, k] - L + l[i-1] <= (1 - S[i, k]) * M
for k in V:
    for i in C:
        model.addConstr(propto[i, k] - L + l[i-1] <= (1 - S[i, k]) * M)
# \beta_{ik}-W+w_i\le\left(1-S_i^k\right)M{\ }\forall i\in C,\forall k\in V
# for k in V:
#     for i in C:
#         problem += beta[i, k] - W + w[i-1] <= (1 - S[i, k]) * M
for k in V:
    for i in C:
        model.addConstr(beta[i, k] - W + w[i-1] <= (1 - S[i, k]) * M)
# \gamma_{ik}-H+h_i\le\left(1-S_i^k\right)M{\ }\forall i\in C,\forall k\in V
# for k in V:
#     for i in C:
#         problem += gamma[i, k] - H + h[i-1] <= (1 - S[i, k]) * M
for k in V:
    for i in C:
        model.addConstr(gamma[i, k] - H + h[i-1] <= (1 - S[i, k]) * M)
# （16）防止同一车辆内的两个箱子重叠：
# 注：物品i,j在x轴上的投影：i:[cx_{ik},cx_{ik}+l_i],j:[cx_{ik},cx_{ik}+l_i]
# \propto_{jk}+l_j\le\propto_{ik}+\delta_{1ij}^kM{\ }\forall i,j\in C,j\neq i,\forall k\in V
# for k in V:
#     for i in C:
#         for j in C:
#             if j != i:
#                 problem += propto[j, k] + l[j-1] <= propto[i, k] + delta[1, i, j, k] * M
for k in V:
    for i in C:
        for j in C:
            if j != i:
                model.addConstr(propto[j, k] + l[j-1] <= propto[i, k] + delta[1, i, j, k] * M)
# \propto_{ik}+l_i\le\propto_{jk}+\delta_{2ij}^kM{\ }\forall i,j\in C,j\neq i,\forall k\in V
# for k in V:
#     for i in C:
#         for j in C:
#             if j != i:
#                 problem += propto[i, k] + l[i-1] <= propto[j, k] + delta[2, i, j, k] * M
for k in V:
    for i in C:
        for j in C:
            if j != i:
                model.addConstr(propto[i, k] + l[i-1] <= propto[j, k] + delta[2, i, j, k] * M)
# \beta_{jk}+w_j\le\beta_{ik}+\delta_{3ij}^kM{\ }\forall i,j\in C,j\neq i,\forall k\in V
# for k in V:
#     for i in C:
#         for j in C:
#             if j != i:
#                 problem += beta[j, k] + w[j-1] <= beta[i, k] + delta[3, i, j, k] * M
for k in V:
    for i in C:
        for j in C:
            if j != i:
                model.addConstr(beta[j, k] + w[j-1] <= beta[i, k] + delta[3, i, j, k] * M)
# \beta_{ik}+w_i\le\beta_{jk}+\delta_{4ij}^kM{\ }\forall i,j\in C,j\neq i,\forall k\in V
# for k in V:
#     for i in C:
#         for j in C:
#             if j != i:
#                 problem += beta[i, k] + w[i-1] <= beta[j, k] + delta[4, i, j, k] * M
for k in V:
    for i in C:
        for j in C:
            if j != i:
                model.addConstr(beta[i, k] + w[i-1] <= beta[j, k] + delta[4, i, j, k] * M)
# \gamma_{jk}+h_j\le\gamma_{ik}+\delta_{5ij}^kM{\ }\forall i,j\in C,j\neq i,\forall k\in V
# for k in V:
#     for i in C:
#         for j in C:
#             if j != i:
#                 problem += gamma[j, k] + h[j-1] <= gamma[i, k] + delta[5, i, j, k] * M
for k in V:
    for i in C:
        for j in C:
            if j != i:
                model.addConstr(gamma[j, k] + h[j-1] <= gamma[i, k] + delta[5, i, j, k] * M)
# \gamma_{ik}+h_i\le\gamma_{jk}+\delta_{6ij}^kM{\ }\forall i,j\in C,j\neq i,\forall k\in V
# for k in V:
#     for i in C:
#         for j in C:
#             if j != i:
#                 problem += gamma[i, k] + h[i-1] <= gamma[j, k] + delta[6, i, j, k] * M
for k in V:
    for i in C:
        for j in C:
            if j != i:
                model.addConstr(gamma[i, k] + h[i-1] <= gamma[j, k] + delta[6, i, j, k] * M)
# \ \sum_{g=1}^{6}\hairsp\hairsp\delta_{gij}^k\le5+\left(1-S_i^k\right)+\left(1-S_j^k\right){\ }\forall i,j\in C,j\neq i,\forall k\in V
# for k in V:
#     for i in C:
#         for j in C:
#             if j != i:
#                 problem += pl.lpSum(delta[g, i, j, k] for g in range(1, 7)) <= 5 + (1 - S[i, k]) + (1 - S[j, k])
for k in V:
    for i in C:
        for j in C:
            if j != i:
                model.addConstr(gp.quicksum(delta[g, i, j, k] for g in range(1, 7)) <= 5 + (1 - S[i, k]) + (1 - S[j, k]))
# （17）后进先出约束：先出的要么在上面，要么在右侧，要么在前面靠近车厢门的方位
# \propto_{ik}-\propto_{jk}\geq l_j-M\eta_{1ij}^k{\ }\forall i,j\in C,j\neq i,\forall k\in V
# for k in V:
#     for i in C:
#         for j in C:
#             if j != i:
#                 problem += propto[i, k] - propto[j, k] >= l[j-1] - M * eta[1, i, j, k]
for k in V:
    for i in C:
        for j in C:
            if j != i:
                model.addConstr(propto[i, k] - propto[j, k] >= l[j-1] - M * eta[1, i, j, k])
# \beta_{ik}-\beta_{jk}\geq w_j-M\eta_{2ij}^k{\ }\forall i,j\in C,j\neq i,\forall k\in V
# for k in V:
#     for i in C:
#         for j in C:
#             if j != i:
#                 problem += beta[i, k] - beta[j, k] >= w[j-1] - M * eta[2, i, j, k]
for k in V:
    for i in C:
        for j in C:
            if j != i:
                model.addConstr(beta[i, k] - beta[j, k] >= w[j-1] - M * eta[2, i, j, k])
# \gamma_{ik}-\gamma_{jk}\geq h_j-M\eta_{3ij}^k{\ }\forall i,j\in C,j\neq i,\forall k\in V
# for k in V:
#     for i in C:
#         for j in C:
#             if j != i:
#                 problem += gamma[i, k] - gamma[j, k] >= h[j-1] - M * eta[3, i, j, k]
for k in V:
    for i in C:
        for j in C:
            if j != i:
                model.addConstr(gamma[i, k] - gamma[j, k] >= h[j-1] - M * eta[3, i, j, k])
# \sum_{f=1}^{3}\hairsp\hairsp\eta_{fij}^k\le2+\left(1-x_{ij}^k\right),\forall i,j\in C,j\neq i,\forall k\in V
# for k in V:
#     for i in C:
#         for j in C:
#             if j != i:
#                 problem += pl.lpSum(eta[f, i, j, k] for f in range(1, 4)) <= 2 + (1 - x[i, j, k])
for k in V:
    for i in C:
        for j in C:
            if j != i:
                model.addConstr(gp.quicksum(eta[f, i, j, k] for f in range(1, 4)) <= 2 + (1 - x[i, j, k]))
# \sum_{f=1}^{3}\hairsp\hairsp\eta_{fij}^k\le2+\left(1-y_{ijm}^k\right),\forall i,m,j\in C,m\neq i,j\neq i,j\neq m,\forall k\in V
# for k in V:
#     for i in C:
#         for m in C:
#             if m != i:
#                 for j in C:
#                     if j != i and j != m:
#                         problem += pl.lpSum(eta[f, i, j, k] for f in range(1, 4)) <= 2 + (1 - y[i, j, m, k])
for k in V:
    for i in C:
        for m in C:
            if m != i:
                for j in C:
                    if j != i and j != m:
                        model.addConstr(gp.quicksum(eta[f, i, j, k] for f in range(1, 4)) <= 2 + (1 - y[i, j, m, k]))
# \sum_{f=1}^{3}\hairsp\hairsp\eta_{fij}^k\le2+\left(1-y_{mij}^k\right),\forall i,m,j\in C,m\neq i,j\neq i,j\neq m,\forall k\in V
# for k in V:
#     for i in C:
#         for m in C:
#             if m != i:
#                 for j in C:
#                     if j != i and j != m:
#                         problem += pl.lpSum(eta[f, i, j, k] for f in range(1, 4)) <= 2 + (1 - y[m, i, j, k])
for k in V:
    for i in C:
        for m in C:
            if m != i:
                for j in C:
                    if j != i and j != m:
                        model.addConstr(gp.quicksum(eta[f, i, j, k] for f in range(1, 4)) <= 2 + (1 - y[m, i, j, k]))
# \sum_{f=1}^{3}\hairsp\hairsp\eta_{fij}^k\le2+\left(2-y_{mii^\prime}^k-x_{mj}^k\right),\forall i,m,j,i^\prime\in C,m\neq i,j\neq i,j\neq m,i^\prime\neq i,i^\prime\neq m,i^\prime\neq j,\forall k\in V
# for k in V:
#     for i in C:
#         for m in C:
#             if m != i:
#                 for j in C:
#                     if j != i and j != m:
#                         for i_ in C:
#                             if i_ != i and i_ != m and i_ != j:
#                                 problem += pl.lpSum(eta[f, i, j, k] for f in range(1, 4)) <= 2 + (2 - y[m, i, i_, k] - x[m, j, k])
for k in V:
    for i in C:
        for m in C:
            if m != i:
                for j in C:
                    if j != i and j != m:
                        for i_ in C:
                            if i_ != i and i_ != m and i_ != j:
                                model.addConstr(gp.quicksum(eta[f, i, j, k] for f in range(1, 4)) <= 2 + (2 - y[m, i, i_, k] - x[m, j, k]))
# （18）不可悬空放置：

x1 = model.addVars([(i, j, k) for i in C for j in C if j != i for k in V], lb=0, vtype=GRB.CONTINUOUS)
x2 = model.addVars([(i, j, k) for i in C for j in C if j != i for k in V], lb=0, vtype=GRB.CONTINUOUS)
y1 = model.addVars([(i, j, k) for i in C for j in C if j != i for k in V], lb=0, vtype=GRB.CONTINUOUS)
y2 = model.addVars([(i, j, k) for i in C for j in C if j != i for k in V], lb=0, vtype=GRB.CONTINUOUS)
new_propto = model.addVars([(i, k) for i in C for k in V], lb=0, vtype=GRB.CONTINUOUS)
new_beta = model.addVars([(i, k) for i in C for k in V], lb=0, vtype=GRB.CONTINUOUS)
area = model.addVars([(i, j, k) for i in C for j in C if j != i for k in V], lb=0, vtype=GRB.CONTINUOUS)
area2 = model.addVars([(i, j, k) for i in C for j in C if j != i for k in V], lb=0, vtype=GRB.CONTINUOUS)
area_l = model.addVars([(i, j, k) for i in C for j in C if j != i for k in V], lb=0, vtype=GRB.CONTINUOUS)
area_w = model.addVars([(i, j, k) for i in C for j in C if j != i for k in V], lb=0, vtype=GRB.CONTINUOUS)
if_x1overx2 = model.addVars([(i, j, k) for i in C for j in C if j != i for k in V], vtype=GRB.BINARY)
if_y1overy2 = model.addVars([(i, j, k) for i in C for j in C if j != i for k in V], vtype=GRB.BINARY)
if_sametruck = model.addVars([(i, j, k) for i in C for j in C if j != i for k in V], vtype=GRB.BINARY)
eps = 0.0001

if_gamma = model.addVars([(i, j, k) for i in C for j in C if j != i for k in V], vtype=GRB.BINARY)
# gamma[i, k] == 0 则 if_gamma[i, j, k] == 0
# gamma[i, k] > 0 则 if_gamma[i, j, k] == 1
if_gamma0 = model.addVars([(i, k) for i in C for k in V], vtype=GRB.BINARY)
for k in V:
    for i in C:
        model.addConstr(if_gamma0[i, k] >= gamma[i, k] / M)
        model.addConstr(if_gamma0[i, k] <= gamma[i, k] / eps)


# b1 = model.addVars([(i, j, k) for i in C for j in C if j != i for k in V], vtype=GRB.BINARY)
# b2 = model.addVars([(i, j, k) for i in C for j in C if j != i for k in V], vtype=GRB.BINARY)
# b = model.addVars([(i, j, k) for i in C for j in C if j != i for k in V], vtype=GRB.BINARY)
for k in V:
    for i in C:
        model.addConstr(new_propto[i, k] == propto[i, k] + l[i - 1])
        model.addConstr(new_beta[i, k] == beta[i, k] + w[i - 1])
        for j in C:
            if j != i:
                
                model.addConstr(x1[i, j, k] == gp.min_(new_propto[j, k], new_propto[i, k]))
                model.addConstr(y1[i, j, k] == gp.min_(new_beta[j, k], new_beta[i, k]))

                model.addConstr(x2[i, j, k] == gp.max_(propto[j, k], propto[i, k]))
                model.addConstr(y2[i, j, k] == gp.max_(beta[j, k], beta[i, k]))

                model.addConstr(x1[i, j, k] >= x2[i, j, k] + eps - M * (1 - if_x1overx2[i, j, k]))
                model.addConstr(x1[i, j, k] <= x2[i, j, k] + M * if_x1overx2[i, j, k])

                model.addConstr(y1[i, j, k] >= y2[i, j, k] + eps - M * (1 - if_y1overy2[i, j, k]))
                model.addConstr(y1[i, j, k] <= y2[i, j, k] + M * if_y1overy2[i, j, k])
                
                model.addConstr((if_x1overx2[i, j, k] == 1) >> (area_l[i, j, k] == x1[i, j, k]-x2[i, j, k]))
                model.addConstr((if_x1overx2[i, j, k] == 0) >> (area_l[i, j, k] == 0))
                model.addConstr((if_y1overy2[i, j, k] == 1) >> (area_w[i, j, k] == x1[i, j, k]-x2[i, j, k]))
                model.addConstr((if_y1overy2[i, j, k] == 0) >> (area_w[i, j, k] == 0))
                model.addConstr(if_sametruck[i, j, k] == S[i, k] * S[j, k])
                # model.addConstr((if_sametruck[i, j, k] == 1) >> (area[i, j, k] == area_l[i, j, k] * area_w[i, j, k]))
                # model.addConstr((if_sametruck[i, j, k] == 0) >> (area[i, j, k] == 0))
                model.addConstr(area[i, j, k] <= M * if_sametruck[i, j, k])
                model.addConstr(area[i, j, k] >= area_l[i, j, k] * area_w[i, j, k] - (1 - if_sametruck[i, j, k]) * M)
                model.addConstr(area[i, j, k] <= area_l[i, j, k] * area_w[i, j, k] + (1 - if_sametruck[i, j, k]) * M)
                # model.addConstr(height_difference[i, j, k] == gp.abs_(gamma[j, k] - gamma[i, k]))

                model.addConstr((1 - if_gamma[i, j, k]) * (gamma[j, k] + h[j-1]) <= gamma[i, k] - eps * (1 - if_gamma[i, j, k]))
                model.addConstr((1 - if_gamma[i, j, k]) * (gamma[j, k] + h[j-1]) >= gamma[i, k] + eps * (1 - if_gamma[i, j, k]))
                model.addConstr(area2[i, j, k] == area[i, j, k] * if_gamma[i, j, k])

                # gamma是连续变量，if_gamma是0-1变量
                # gamma[j, k] + h[j-1] == gamma[i, k] 则 if_gamma[i, j, k] == 1
                # gamma[j, k] + h[j-1] != gamma[i, k] 则 if_gamma[i, j, k] == 0
                # m.addConstr(gamma[j, k] + h[j-1] >= gamma[i, k] + eps - M * b1[i, j, k])
                # m.addConstr(gamma[j, k] + h[j-1] <= gamma[i, k] + M * ( 1 - b1[i, j, k]))
                # m.addConstr(gamma[i, k] >= gamma[j, k] + h[j-1] + eps - M * b2[i, j, k])
                # m.addConstr(gamma[i, k] <= gamma[j, k] + h[j-1] + M * ( 1 - b2[i, j, k]))
                # model.addConstr(b[i, j, k] == b1[i, j, k] * b2[i, j, k])         

                
for k in V:
    for i in C:
        model.addConstr(gp.quicksum(area2[i, j, k] for j in C if j != i) >= 0.6 * propto[i, k] * beta[i, k] - M * (1- S[i, k]) -M * (1-if_gamma0[i, k]))







# 求解模型
# problem.solve()
model.setParam('NonConvex', 2)
model.optimize()
# # 如果有解，输出
# if pl.LpStatus[problem.status] == 'Optimal':
#     print("Optimal solution found!")
# else:
#     print("No optimal solution found.")
# if model.status == GRB.OPTIMAL:
#     print("Optimal solution found!")
# else:
#     print("No optimal solution found.")


# 记录包裹坐标
coordinate = []

if model.status == GRB.OPTIMAL:
    print("Optimal solution found!")

    for k in V:
        print(f"truck {k}:")
        route = []
        for i in N[:-1]:
            for j in N[1:]:
                if j != i and x[i, j, k].x > 0.5:
                    route.append((i, j))
        print(f"Route: {route}")
        
        print(f"drone {k}:")
        route2 = []
        for i in C:
            for m in C:
                if m != i:
                    for j in C:
                        if j != i and j != m and y[i, m, j, k].x > 0.5:
                            route2.append((i, m, j))
        print(f"Route: {route2}")

        for i in C:
            if S[i, k].x > 0.5:
                coordinate.append((i, k, propto[i, k].x, beta[i, k].x, gamma[i, k].x))

# 包裹的三维尺寸
print(f"包裹的三维尺寸：")
print([(l[i], w[i], h[i]) for i in range(n)])
# 包裹的坐标
print(f"包裹的坐标: {coordinate}")

# 画图
goods_dimensions = [[],[]]
# for i in C:
#     goods_dimensions.append((l[i-1], w[i-1], h[i-1]))

goods_positions = []

for elem in coordinate:
    for k in V:
        if elem[1] == k:
            goods_dimensions[k-1].append((l[elem[0]-1], w[elem[0]-1], h[elem[0]-1]))

print(goods_dimensions)

if model.status == GRB.OPTIMAL:
    for k in V:
        print(k)
        draw_packing_result(L, W, H, goods_dimensions[k-1], [elem[-3:] for elem in coordinate if elem[1] == k], k)