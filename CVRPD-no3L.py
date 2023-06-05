from cycler import cycler
import pulp as pl
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import csv
import itertools

def generate_symmetric_matrix(n, low, high):
    matrix = np.random.randint(low=low, high=high, size=(n, n))
    sym_matrix = matrix + matrix.T
    np.fill_diagonal(sym_matrix, 0)
    sym_matrix[0][n-1] = 0
    sym_matrix[n-1][0] = 0
    return sym_matrix


# # 定义问题参数
# num_customers = 10
# num_vehicles = 3
# capacity = 100

# with open('A-n32-k5demand.txt', 'r') as f:
#     reader = csv.reader(f, delimiter=' ')
#     demand = []
#     for row in reader:
#         demand.append(int(row[1]))

# with open('A-n32-k5.txt', 'r') as f:
#     reader = csv.reader(f, delimiter=' ')
#     coordinates = []
#     for row in reader:
#         coordinates.append((float(row[1]), float(row[2])))

# coords = coordinates[:num_customers+1]
# dist = lambda a, b: np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

# # 计算距离矩阵
# distance_matrix = np.array([[dist(coords[i], coords[j]) for j in range(num_customers + 1)] for i in range(num_customers + 1)])

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
Q_d = 5
# t_{ij}	truck traveling time between nodes i and j

t = generate_symmetric_matrix(n+2, 2, 5)
# t_{ij}^\prime	drone traveling time between nodes i and j
t_prime = generate_symmetric_matrix(n+2, 1, 2)
# T_1	truck service time
T_1 = 10
# T_2	drone service time
T_2 = 1
# T_3	drone launch time
T_3 = 1
# T_4	drone recovery time
T_4 = 1
# \bar{T}	maximum endurance of the drone
T_bar = 10
# q_i	load amount of package needed of customer i
q = np.random.randint(low=2, high=7, size=n)
# L,W,H	length, width, and height of carriage of truck
L = 150
W = 120
H = 70
# L^\prime,W^\prime,H^\prime	length, width, and height of carriage of drone
L_prime = 4
W_prime = 4
H_prime = 4
# l_i,w_i,h_i	length, width, and height of package needed of customer i
l = np.random.randint(low=2, high=6, size=n)
w = np.random.randint(low=2, high=6, size=n)
h = np.random.randint(low=2, high=6, size=n)
# M	a large number
M = 1000000

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
problem = pl.LpProblem("3L-CVRP-D", pl.LpMinimize)

# 定义决策变量

# x_{ij}^k	1, if arc (i,j) is traversed by truck k,0 otherwise
# x_{ij}^k\in{0,1},\forallk\inV,i\in{N:i\neqn+1},j\in{N:j\neq0,j\neqi}
x = pl.LpVariable.dicts("x", [(i, j, k) for i in N[:-1] for j in N[1:] if j != i for k in V], 0, 1, pl.LpBinary)
# x = pl.LpVariable.dicts('x', [(i, j, k) for i in range(1, n+1) for j in range(1, n+1) if i != j for k in range(1, K+1)], lowBound=0, upBound=1, cat=pl.LpInteger)

# y_{imj}^k	1, if sortie (i,m,j) is performed by the drone belonging to truck k,0 otherwise
# y_{imj}^k\in0,1,\forallk\inV,i\inC,m\in{C:m\neqi},j\in{C:j\neqi,j\neqm}
y = pl.LpVariable.dicts("y", [(i, m, j, k) for i in C for m in C if m != i for j in C if j != i and j != m for k in V], 0, 1, pl.LpBinary)
# y = pl.LpVariable.dicts('y', [(i, m, j, k) for i in range(1, n+1) for m in range(1, n+1) if m != i for j in range(1, n+1) if j != i and j != m for k in range(1, K+1)], lowBound=0, upBound=1, cat=pl.LpInteger)

# a_i^k	arrival time of truck k at node i
a = pl.LpVariable.dicts("a", [(i, k) for i in N for k in V], 0, None, pl.LpContinuous)
# a = pl.LpVariable.dicts('a', [(i, k) for i in range(n+2) for k in range(1, K+1)], lowBound=0, cat=pl.LpContinuous)
# a_0^k=0,\forallk\inV
for k in V:
    problem += a[0, k] == 0
# a_i^{\prime k}	arrival time of the drone belonging to truck k at node i
a_ = pl.LpVariable.dicts("a_", [(i, k) for i in N for k in V], 0, None, pl.LpContinuous)
# a_i^k\geq0,a_i^{\prime k}\geq0,\forallk\inV,i\inN 定义决策变量的时候已经满足非负
# for k in range(z):
#     for i in range(n + 2):
#         m += a[i, k] >= 0
#         m += a_[i, k] >= 0

# \mathbit{S}_\mathbit{i}^\mathbit{k}	1, if package needed by customer i is served by truck k,0 otherwise
# S_i^k\in{0,1},\forallk\inV,i\inC
S = pl.LpVariable.dicts("S", [(i, k) for i in C for k in V], 0, 1, pl.LpBinary)

# \propto_{ik},\beta_{ik},\gamma_{ik}	placement coordinates of the left front vertex of package needed by customer i inside truck k
# \propto_{ik},\beta_{ik},\gamma_{ik}\geq0,\forallk\inV,i\inC
propto = pl.LpVariable.dicts("propto", [(i, k) for i in C for k in V], 0, None, pl.LpContinuous)
beta = pl.LpVariable.dicts("beta", [(i, k) for i in C for k in V], 0, None, pl.LpContinuous)
gamma = pl.LpVariable.dicts("gamma", [(i, k) for i in C for k in V], 0, None, pl.LpContinuous)

# \delta_{gij}^k	a binary variable for non-overlapping of package needed by customer i and j for truck k
# \delta_{gij}^k\in{0,1},\forallk\inV,i\inC,j\in{C:j\neqi}
delta = pl.LpVariable.dicts("delta", [(g, i, j, k) for g in range(1, 7) for i in C for j in C if j != i for k in V], 0, 1, pl.LpBinary)
# \tau	continuous variable that indicates total completion time
tau = pl.LpVariable("tau", 0, None, pl.LpContinuous)
# \eta_{fij}^k	a binary variable for LIFO
# \eta_{fij}^k\in{0,1},\forallk\inV,i\inC,j\in{C:j\neqi}
eta = pl.LpVariable.dicts("eta", [(f, i, j, k) for f in range(1, 4) for i in C for j in C if j != i for k in V], 0, 1, pl.LpBinary)



# 定义目标函数
# Minimize \tau+k∈V,j∈C,j≠0  x0jk
problem += tau + pl.lpSum(x[0, j, k] for k in V for j in C)

# 添加约束条件

# （2）计算总的完成时间:
# \tau\geq a_{n+1}^k,\forall k\in V
for k in V:
    problem += tau >= a[n + 1, k]

# （3）所有客户只访问一次:
# k∈V,i∈N,i≠n+1,i≠m   ximk+k∈V,i∈C,i≠mj∈C,j≠mi≠j  yimjk=1,∀m∈C
for m in C:
    problem += pl.lpSum(x[i, m, k] for i in N[:-1] if i != m for k in V) + pl.lpSum(y[i, m, j, k] for i in C if i != m for j in C if j != m and j != i for k in V) == 1

# （4）车辆从仓库出发且最后回到仓库:
# 注：x_{0(n+1)}^k=1表示第k辆卡车没有使用
# \sum_{j\in N,j\neq0}\hairsp\hairsp x_{0j}^k=1,\forall k\in V
for k in V:
    problem += pl.lpSum(x[0, j, k] for j in N[1:]) == 1
# \sum_{i\in N,i\neq n+1}\hairsp\hairsp x_{i(n+1)}^k=1,\forall k\in V
for k in V:
    problem += pl.lpSum(x[i, n + 1, k] for i in N[:-1]) == 1

# （5）流量守恒：车辆到达某顾客，必须离开该顾客到达下一顾客:
# \sum_{i\in N,i\neq n+1}\hairsp\hairsp x_{ij}^k-\sum_{i\in N,i\neq0}\hairsp\hairsp x_{ji}^k=0,\forall j\in C,\forall k\in V
for j in C:
    for k in V:
        problem += pl.lpSum(x[i, j, k] for i in N[:-1] if i!= j) - pl.lpSum(x[j, i, k] for i in N[1:] if i!= j) == 0


# （6）消除子回路约束:
# i∈Sj∈S  xijk≤|S|-1,∀S⊆C,k∈V
# for k in range(z):
#     for S in range(1, n + 1):
#         m += pl.lpSum(x[i, j, k] for i in range(n + 2) if i in S for j in range(n + 2) if j in S) <= len(S) - 1
for k in V:
    for length in range(2, len(C)+1):
        for subset in itertools.combinations(C, length):
            problem += pl.lpSum(x[i, j, k] for i in subset for j in subset if j != i) <= len(subset) - 1
    
# （7）载重约束:
# 卡车载重：
# i∈N,i≠n+1j∈C,j≠i qj xijk+i∈Cm∈C,m≠ij∈C,j≠m,j≠i qm yimjk≤Qk,∀k∈V
for k in V:
    problem += pl.lpSum(q[j-1] * x[i, j, k] for i in N[:-1] for j in C if j != i) + pl.lpSum(q[m-1] * y[i, m, j, k] for i in C for m in C if m != i for j in C if j != m and j != i) <= Q_t


# 无人机载重：
# q_my_{imj}^k⩽Qd,∀k∈V,i∈C,m∈{C:m≠i},j∈{C:j≠i,j≠m}
for k in V:
    for i in C:
        for m in C:
            if m != i:
                for j in C:
                    if j != i and j != m:
                        problem += q[m-1] * y[i, m, j, k] <= Q_d

# （8）无人机体积约束：
# l_my_{imj}^k⩽L',∀k∈V,i∈C,m∈{C:m≠i},j∈{C:j≠i,j≠m}
for k in V:
    for i in C:
        for m in C:
            if m != i:
                for j in C:
                    if j != i and j != m:
                        problem += l[m-1] * y[i, m, j, k] <= L_prime
                        problem += w[m-1] * y[i, m, j, k] <= W_prime
                        problem += h[m-1] * y[i, m, j, k] <= H_prime

# （9）无人机不可重复发射或者重复回收:
# m∈C,m≠ij∈C,j≠m,j≠i  yimjk≤1,∀k∈V,i∈C
for k in V:
    for i in C:
        problem += pl.lpSum(y[i, m, j, k] for m in C if m != i for j in C if j != i and j != m) <= 1

# m∈C,m≠ii∈C,i≠m,i≠j  yimjk≤1,∀k∈V,j∈C
for k in V:
    for j in C:
        problem += pl.lpSum(y[i, m, j, k] for m in C if m != i for j in C if j != i and j != m) <= 1

# （10）无人机的发射点和回收点由携带该无人机的卡车先后服务
# {2y}_{imj}^k\les∈C,s≠i   xisk+s∈C,s≠j   xsjk,\forall k\in V,i\inC,m\in{C:m\neq i},j\in{C:j\neq i,j\neq w}
for k in V:
    for i in C:
        for m in C:
            if m != i:
                for j in C:
                    if j != i and j != m:
                        problem += 2 * y[i, m, j, k] <= pl.lpSum(x[i, s, k] for s in C if s != i) + pl.lpSum(x[s, j, k] for s in C if s != j)

# a_j^k\geq a_i^k+M\left(\hairsp\hairsp y_{imj}^k-1\right),\forall k\in V,i\inC,m\in{C:m\neq i},j\in{C:j\neq i,j\neq m}
for k in V:
    for i in C:
        for m in C:
            if m != i:
                for j in C:
                    if j != i and j != m:
                        problem += a[j, k] >= a[i, k] + M * (y[i, m, j, k] - 1)
# 时间约束：
# （11）卡车到达某节点的时间:
# 1-	前一个节点是仓库
# a_j^k\geq a_0^k+t_{ij}+M(\hairsp x_{0j}^k-1),\forall k\inV,j\inC
for k in V:
    for j in C:
        problem += a[j, k] >= a[0, k] + t[0, j] + M * (x[0, j, k] - 1)
# 2-	前一个节点是顾客点且没有无人机发射和回收
# a_j^k\geq a_i^k+T_1+t_{ij}+M(\hairsp x_{ij}^k-1),\forall k\inV,i\inC,j\in{N:j\neq0,j\neq i}
for k in V:
    for i in C:
        for j in N[1:]:
            if j != i:
                problem += a[j, k] >= a[i, k] + T_1 + t[i, j] + M * (x[i, j, k] - 1)
# 3-	前一个节点是顾客点且有无人机发射，没有无人机回收
# a_j^k\geq a_i^k+T_1+T_3+t_{ij}+M\left(\hairsp x_{ij}^k-1\right)+M(\hairspm∈C,m≠ii'∈C,i'≠m,i'≠i  yimi'k-1),∀k∈V,i∈C,j∈{N:j≠0,j≠i}
for k in V:
    for i in C:
        for j in N[1:]:
            if j != i:
                problem += a[j, k] >= a[i, k] + T_1 + T_3 + t[i, j] + M * (x[i, j, k] - 1) + M * (pl.lpSum(y[i, m, i_, k] for m in C if m != i for i_ in C if i_ != i and i_ != m) - 1)
# 4-	前一个节点是顾客点且有无人机回收，没有无人机发射
# a_j^k\geq a_i^k+T_1+T_4+t_{ij}+M\left(\hairsp x_{ij}^k-1\right)+M(\hairspi'∈C,i'≠im∈C,m≠i,m≠i'  yi'mik-1),∀k∈V,i∈C,j∈{N:j≠0,j≠i}
for k in V:
    for i in C:
        for j in N[1:]:
            if j != i:
                problem += a[j, k] >= a[i, k] + T_1 + T_4 + t[i, j] + M * (x[i, j, k] - 1) + M * (pl.lpSum(y[i_, m, i, k] for m in C if m != i for i_ in C if i_ != i and i_ != m) - 1)
# a_j^k\geq a_i^{\prime k}+T_4+t_{ij}+M\left(\hairsp x_{ij}^k-1\right)+M(\hairspi'∈C,i'≠im∈C,m≠i,m≠i'  yi'mik-1),∀k∈V,i∈C,j∈{N:j≠0,j≠i}
for k in V:
    for i in C:
        for j in N[1:]:
            if j != i:
                problem += a[j, k] >= a_[i, k] + T_4 + t[i, j] + M * (x[i, j, k] - 1) + M * (pl.lpSum(y[i_, m, i, k] for m in C if m != i for i_ in C if i_ != i and i_ != m) - 1)
# 5-	前一个节点是顾客点且有无人机发射，有无人机回收
# a_j^k\geq a_i^k+T_1+T_4+T_3+t_{ij}+M\left(\hairsp x_{ij}^k-1\right)+M m∈C,m≠ii'∈C,i'≠m,i'≠i  yimi'k+i'∈C,i'≠im∈C,m≠i,m≠i'  yi'mik-2,\forall k\inV,i\inC,j\in{N:j\neq0,j\neq i}
for k in V:
    for i in C:
        for j in N[1:]:
            if j != i:
                problem += a[j, k] >= a[i, k] + T_1 +T_4 + T_3 + t[i, j] + M * (x[i, j, k] - 1) + M * (pl.lpSum(y[i, m, i_, k] for m in C if m != i for i_ in C if i_ != i and i_ != m) + pl.lpSum(y[i_, m, i, k] for m in C if m != i for i_ in C if i_ != i and i_ != m) - 2)
# a_j^k\geq a_i^{\prime k}+T_4+T_3+t_{ij}+M\left(\hairsp x_{ij}^k-1\right)+M m∈C,m≠ii'∈C,i'≠m,i'≠i  yimi'k+i'∈C,i'≠im∈C,m≠i,m≠i'  yi'mik-2,\forall k\inV,i\inC,j\in{N:j\neq0,j\neq i}
for k in V:
    for i in C:
        for j in N[1:]:
            if j != i:
                problem += a[j, k] >= a_[i, k] + T_4 + T_3 + t[i, j] + M * (x[i, j, k] - 1) + M * (pl.lpSum(y[i, m, i_, k] for m in C if m != i for i_ in C if i_ != i and i_ != m) + pl.lpSum(y[i_, m, i, k] for m in C if m != i for i_ in C if i_ != i and i_ != m) - 2)

# （12）无人机到达某节点的时间:
# 1-	当前节点由无人机服务
# 1-1发射节点i处没有无人机回收
# a_j^{\prime k}\geq a_i^k+T_1+T_3+t_{ij}^\prime+M m∈C,m≠i,m≠j  yijmk-1,\forall k\in V,i\inC,j\in{C:j\neq i}
for k in V:
    for i in C:
        for j in C:
            if j != i:
                problem += a_[j, k] >= a[i, k] + T_1 + T_3 + t_prime[i, j] + M * (pl.lpSum(y[i, j, m, k] for m in C if m != i and m != j) - 1)
# 1-2发射节点i处有无人机回收
# a_j^{\prime k}\geq a_i^k+T_1+T_4+T_3+t_{ij}^\prime+M m∈C,m≠i,m≠j  yijmk-1+M(\hairspi'∈C,i'≠im∈C,m≠i,m≠i'  yi'mik-1),∀k∈V,i∈C,j∈{C:j≠i}
for k in V:
    for i in C:
        for j in C:
            if j != i:
                problem += a_[j, k] >= a[i, k] + T_1 + T_4 + T_3 + t_prime[i, j] + M * (pl.lpSum(y[i, j, m, k] for m in C if m != i and m != j) - 1) + M * (pl.lpSum(y[i_, m, i, k] for i_ in C if i_ != i for m in C if m != i and m != i_) - 1)
# a_j^{\prime k}\geq a_i^{\prime k}+T_4+T_3+t_{ij}^\prime+M m∈C,m≠i,m≠j  yijmk-1+M(\hairspi'∈C,i'≠im∈C,m≠i,m≠i'  yi'mik-1),∀k∈V,i∈C,j∈{C:j≠i}
for k in V:
    for i in C:
        for j in C:
            if j != i:
                problem += a_[j, k] >= a_[i, k] + T_4 + T_3 + t_prime[i, j] + M * (pl.lpSum(y[i, j, m, k] for m in C if m != i and m != j) - 1) + M * (pl.lpSum(y[i_, m, i, k] for i_ in C if i_ != i for m in C if m != i and m != i_) - 1)
# 2-	当前节点为无人机回收点
# a_j^{\prime k}\geq a_i^{\prime k}+T_2+t_{ij}^\prime+M m∈C,m≠i,m≠j  ymijk-1,\forall k\inV,i\inC,j\in{C:j\neq i}
for k in V:
    for i in C:
        for j in C:
            if j != i:
                problem += a_[j, k] >= a_[i, k] + T_2 + t_prime[i, j] + M * (pl.lpSum(y[m, i, j, k] for m in C if m != i and m != j) - 1)

# （13）无人机的出行加上悬停等待满足续航限制
# \left(t_{im}^\prime+t_{mj}^\prime\right)y_{imj}^k⩽T,∀k∈V,i∈N:i≠n+1,m\in{C:m\neq i},j\in{N:j\neq0,j\neq i,j\neq m}
for k in V:
    for i in C:
        for m in C:
            if m != i:
                for j in C:
                    if j != i and j != m:
                        problem += (t_prime[i, m] + t_prime[m, j]) * y[i, m, j, k] <= T_bar
# \left(t_{im}^\prime+t_{mj}^\prime\right)y_{imj}^k+a_j^k+T_1-a_j^{\prime k}⩽T,\forallk\inV,i\inC,m\in{C:m\neq i},j\in{C:j\neq i,j\neq m}
for k in V:
    for i in C:
        for m in C:
            if m != i:
                for j in C:
                    if j != i and j != m:
                        problem += (t_prime[i, m] + t_prime[m, j]) * y[i, m, j, k] + a[j, k] + T_1 - a_[j, k] <= T_bar
# （14）确定顾客i的物品是否被卡车k装载:
# \sum_{j\in N,j\neq0}\hairsp\hairsp x_{ij}^k+m∈C,m≠ij∈C,j≠m,j≠i  ymijk=Sik,∀k∈V,∀i∈C
for k in V:
    for i in C:
        problem += pl.lpSum(x[i, j, k] for j in N[1:] if j != i) + pl.lpSum(y[m, i, j, k] for m in C if m != i for j in C if j != m and j != i) == S[i, k]


# 求解模型
problem.solve()

# 如果有解，输出
if pl.LpStatus[problem.status] == 'Optimal':
    print("Optimal solution found!")
else:
    print("No optimal solution found.")

