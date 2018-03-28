import sympy
from sympy.printing.str import StrPrinter

"""
Source : https://zulko.wordpress.com/2012/04/15/symbolic-matrix-differentiation-with-sympy/
"""


def matrices(names):
    return sympy.symbols(names, commutative=False)


# Matrix specific transformations
d = sympy.Function("d", commutative=False)
inv = sympy.Function("inv", commutative=False)

class t(sympy.Function):
    """ The matrix transpose, with special rules
    t(A + B) = t(A) + t(B) and t(A B) = t(B) t(A)
    """
    is_commutative = False

    def __new__(cls, arg):
        if arg.is_Add:
            return sympy.Add(*[t(A) for A in arg.args])
        elif arg.is_Mul:
            L = len(arg.args)
            return sympy.Mul(*[t(arg.args[L-i-1]) for i in range(L)])
        else:
            return sympy.Function.__new__(cls, arg)


# Matrix differentiation rules
# e = expression
# s = a list of symbols respsect to which we want to differentiate
MATRIX_DIFF_RULES = {
    sympy.Symbol: lambda e, s: d(e) if s.has(e) else 0,
    sympy.Add: lambda e, s: sympy.Add(*[matrix_diff(arg, s) for arg in e.args]),  # NOQA
    sympy.Mul: lambda e, s: sympy.Mul(matrix_diff(e.args[0], s), sympy.Mul(*e.args[1:])) + sympy.Mul(e.args[0], matrix_diff(sympy.Mul(*e.args[1:]), s)),  # NOQA
    t: lambda e, s: t(matrix_diff(e.args[0], s)),
    inv: lambda e, s: - e * matrix_diff(e.args[0], s) * e
}


def matrix_diff(expr, symbols):
    if expr.__class__ in MATRIX_DIFF_RULES.keys():
        return MATRIX_DIFF_RULES[expr.__class__](expr, symbols)
    else:
        return 0


class MatrixConsolePrinter(StrPrinter):
    """Nice printing for console mode : X¯¹, X', ∂X"""

    def _print_inv(self, expr):
        if expr.args[0].is_Symbol:
            return self._print(expr.args[0]) + '¯¹'
        else:
            return '(' + self._print(expr.args[0]) + ')¯¹'

    def _print_t(self, expr):
        return self._print(expr.args[0]) + "'"

    def _print_d(self, expr):
        if expr.args[0].is_Symbol:
            return '∂' + self._print(expr.args[0])
        else:
            return '∂(' + self._print(expr.args[0]) + ')'


def print_matrix(m):
    mem = sympy.Basic.__str__
    sympy.Basic.__str__ = lambda self: MatrixConsolePrinter().doprint(self)
    print(str(m).replace('*', ''))
    sympy.Basic.__str__ = mem


C_IG, p_G_f, p_G_C = sympy.symbols("C_IG p_G_f p_G_C", commutative=False)
p_C_f = C_IG * (p_G_f - p_G_C)

print(matrix_diff(p_C_f, C_IG))
