import numpy as np

def skew(v):
    return np.array([[0, -v[2,0], v[1,0]],
                     [v[2,0], 0, -v[0,0]],
                     [-v[1,0], v[0,0], 0]])

for i in range(1000):
    w0 = np.random.rand(3, 1)
    v = np.ones((3, 1))

    # analytical = skew(w0).dot(skew(v))
    # analytical = -2 * skew(w0).dot(skew(v))
    analytical = np.eye(3) * w0.T.dot(v) + w0.dot(v.T) - 2.0*v.dot(w0.T)
    x0 = skew(w0).dot(skew(w0)).dot(v)

    eps = np.eye(3) * 1e-6
    finite_difference = np.zeros((3, 3))
    for i in range(3):
        wi = w0 + eps[:, i, None]
        xi = skew(wi).dot(skew(wi)).dot(v)
        finite_difference[:, i, None] = (xi - x0) / 1e-6

    # if (analytical - finite_difference > 1e-3).any():
    print "===============\nanalytical:\n", np.around(analytical, 3)
    print "===============\nfinite_difference:\n", np.around(finite_difference, 3)
