# a = sympy.Matrix(sympy.symbols("a_x, a_y, a_z"))  # Accelerometer
# m = sympy.Matrix(sympy.symbols('m_x, m_y, m_z'))  # Magnetomer reading
# w = sympy.Matrix(sympy.symbols("w_x, w_y, w_z"))  # Gyroscope
# dt = sympy.symbols("dt")  # Timestep
#
# G = sympy.Matrix([0, 0, 1])  # Earth-frame gravitational acceleration
# bx, bz = sympy.symbols("b_x, b_z")
# B = sympy.Matrix([bx, 0, bz])  # Earth-frame magnetic field
#
# # p = Quaternion(sympy.symbols("p_1, p_2, p_3 p_4"))
# # q = Quaternion(sympy.symbols("q_1, q_2, q_3 q_4"))
#
#
# q = Quaternion(sympy.symbols("q_0, q_1, q_2, q_3"))
# # Earth-to-Sensor frame acceleration rotation error
# f_1 = q.conj().rot(G) - a
# # Earth-to-Sensor frame magnetic field rotation error
# f_2 = q.conj().rot(B) - m
#
# J_1 = f_1.jacobian(q)
# J_2 = f_2.jacobian(q)
#
# fgrad = (J_1.T * f_1) + (J_2.T * f_2)  # Error Gradient
# alpha = sympy.symbols('alpha')  # Scaling Factor
# gd = q - alpha * fgrad  # Gradient Descent
#
# beta = sympy.symbols('beta')
#
# # Angular rate to angular velocity
# dq = 0.5 * q.prod(Quaternion.fromRV(0, w))
# dq_est = dq - beta * fgrad
# q_est = q + dq_est * dt
#
# sympy.pprint(sympy.simplify(q_est))
