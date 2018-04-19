import sympy

lambda1 = sympy.symbol("lambda1")
lambda2 = sympy.symbol("lambda2")
lambda3 = sympy.symbol("lambda3")
alpha = sympy.symbol("alpha")

lhs = (1 / lambda2)
rhs = (alpha / lambda1) + ((1 - alpha) / lambda3)
sympy.pprint(sympy.solve(lhs - rhs, alpha))
