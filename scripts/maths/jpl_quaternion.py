import sympy
from sympy.matrices import eye as I


class Quaternion(sympy.Matrix):
    @property
    def r(self):
        return self[3]

    @property
    def v(self):
        return self[0:3, :]

    @classmethod
    def fromRV(cls, r, v):
        return Quaternion([v[0], v[1], v[2], r])

    def conj(self):
        return Quaternion.fromRV(self.r, -1 * self.v)

    def prod(self, other):
        p1, p2, p3 = other.v
        p4 = other.r
        q1, q2, q3 = self.v
        q4 = self.r

        x = q4 * p1 + q3 * p2 - q2 * p3 + q1 * p4
        y = -q3 * p1 + q4 * p2 + q1 * p3 + q2 * p4
        z = q2 * p1 - q1 * p2 + q4 * p3 + q3 * p4
        w = -q1 * p1 - q2 * p2 - q3 * p3 + q4 * p4

        return Quaternion.fromRV(w, [x, y, z])

    def rot(self, w):
        W = Quaternion.fromRV(0, w)
        return self.prod(W).prod(self.conj())[1:, :]


def skew(w):
    return sympy.Matrix([[0.0, -w[2], w[1]],
                         [w[2], 0.0, -w[0]],
                         [-w[1], w[0], 0.0]])


def C(q):
    q1, q2, q3, q4 = q

    q_vec = sympy.Matrix([q1, q2, q3])

    C = (2 * q4**2 - 1) * I(3) - (2 * q4 * skew(q_vec)) + 2 * q_vec * q_vec.T
    print(C.jacobian(q))
