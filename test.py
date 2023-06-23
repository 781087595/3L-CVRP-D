# import pulp

# # 创建问题
# problem = pulp.LpProblem('linearization_example', pulp.LpMaximize)

# # 创建决策变量
# x = pulp.LpVariable('x', lowBound=0, upBound=5, cat='Integer')
# y = pulp.LpVariable('y', lowBound=0, upBound=4, cat='Integer')
# u = pulp.LpVariable.dicts('u', range(6), cat='Binary')
# v = pulp.LpVariable.dicts('v', range(5), cat='Binary')

# # 添加约束条件
# problem += x == pulp.lpSum([pulp.lpSum([u[i]] * i) for i in range(6)])
# problem += y == pulp.lpSum([pulp.lpSum([v[i]] * i) for i in range(5)])
# problem += 4 * x + 5 * y <= 20

# # 添加目标函数
# problem += 2 * x + 3 * y

# # 求解问题
# status = problem.solve()

# # 输出结果
# print(f'status: {pulp.LpStatus[status]}')
# print(f'x: {x.value()}, y: {y.value()}, obj: {problem.objective.value()}')
goods_dimensions = [[] for _ in range(3)]
print(goods_dimensions)