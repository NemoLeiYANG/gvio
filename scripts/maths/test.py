import sympy


# Quaternion
q0, q1, q2, q3 = sympy.symbols("q0 q1 q2 q3")

# Quaternion to rotation matrix
R = sympy.Matrix([[0, 0, 0], [0, 0, 0], [0, 0, 0]])

R[0] = q0**2 + q1**2 + q2**2 + q3**2
R[1] = 2 * (q1 * q2 - q0 * q3)
R[2] = 2 * (q1 * q3 - q0 * q2)

R[3] = 2 * (q1 * q2 + q0 * q3)
R[4] = q0**2 - q1**2 + q2**2 - q3**2
R[5] = 2 * (q2 * q3 - q0 * q1)

R[6] = 2 * (q1 * q3 - q0 * q2)
R[7] = 2 * (q2 * q3 + q0 * q1)
R[8] = q0**2 - q1**2 - q2**2 + q3**2

# Partial derivative of rotation matrix w.r.t. quaternion
dRdq0 = R.diff(q0)
dRdq1 = R.diff(q1)
dRdq2 = R.diff(q2)
dRdq3 = R.diff(q3)

dRdq = sympy.zeros(3, 12)
dRdq[0] = dRdq0
dRdq[3] = dRdq1
dRdq[6] = dRdq2
dRdq[9] = dRdq3

# sympy.pprint(dRdq)


# Rotation matrix
r11, r12, r13 = sympy.symbols("r11 r12 r13")
r21, r22, r23 = sympy.symbols("r21 r22 r23")
r31, r32, r33 = sympy.symbols("r31 r32 r33")

R = sympy.zeros(3, 3)
R[0] = r11
R[1] = r12
R[2] = r13

R[3] = r21
R[4] = r22
R[5] = r23

R[6] = r31
R[7] = r32
R[8] = r33

px, py, pz = sympy.symbols("px py pz")
p = sympy.Matrix([[px], [py], [pz]])

g = R * p

# Partial derivative of R * p w.r.t. rotation matrix
dgdr11 = g.diff(r11)
dgdr12 = g.diff(r12)
dgdr13 = g.diff(r13)

dgdr21 = g.diff(r21)
dgdr22 = g.diff(r22)
dgdr23 = g.diff(r23)

dgdr31 = g.diff(r31)
dgdr32 = g.diff(r32)
dgdr33 = g.diff(r33)


dgdR = sympy.zeros(3, 9)

dgdR[0] = dgdr11
dgdR[1] = dgdr12
dgdR[2] = dgdr13

dgdR[3] = dgdr21
dgdR[4] = dgdr22
dgdR[5] = dgdr23

dgdR[6] = dgdr31
dgdR[7] = dgdr32
dgdR[8] = dgdr33

sympy.pprint(dgdR)




