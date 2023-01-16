a = part3
dx = DXform(x)
goal = [950,950]
start = [2120,2500]
dx.plan(goal)
p = dx.query(start)
dx.plot(p)