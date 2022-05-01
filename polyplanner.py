import numpy as np

def time_scaling(t, t0, T):
    return (t - t0)/T
    
def trajectory_coefficients(beta, m, n, T):
    p = np.zeros(n)
    for k in range(m, n):
        p[n - k - 1] = (np.math.factorial(k)*beta**(k-m))/((np.math.factorial(k-m))*T**m)
    return p

def derivative(coef, Ti):
    c = []
    n = len(coef)
    for i in range(n-1):
        ci = (n-1-i)*coef[i]/Ti
        c.append(ci)
    c = np.array(c)
    return c

def local_planner_polynomial(points, v_list, T):
    S = np.hstack([[0], np.cumsum(T)])
    n_legs = len(T)
    n_coeff = n_legs*4
    A = np.zeros((n_coeff, n_coeff))
    b = np.zeros(n_coeff)

    eq_num = 0
    for i in range(n_legs):        
        # p1(0) = x[0]
        A[eq_num, 4*i:4*(i+1)] = trajectory_coefficients(beta=0, m=0, n=4, T=T[i])
        b[eq_num] = points[i]
        eq_num += 1
    
        # p1'(0) = v[0]
        A[eq_num, 4*i:4*(i+1)] = trajectory_coefficients(beta=0, m=1, n=4, T=T[i])
        b[eq_num] = v_list[i]
        eq_num += 1
    
        # p1(1) = x[1]
        A[eq_num, 4*i:4*(i+1)] = trajectory_coefficients(beta=1, m=0, n=4, T=T[i])
        b[eq_num] = points[i + 1]
        eq_num += 1
    
        # p1'(1) = v[1]
        A[eq_num, 4*i:4*(i+1)] = trajectory_coefficients(beta=1, m=1, n=4, T=T[i])
        b[eq_num] = v_list[i + 1]
        eq_num += 1

    c = np.linalg.inv(A).dot(b)
        
    t = []
    x = []
    v = []
    a = []
    for i in range(len(T)):
        ti = np.linspace(S[i], S[i+1])
        beta = time_scaling(ti, S[i], T[i])
        cp = c[4*i:4*(i+1)]
        cv = derivative(cp, T[i])
        ca = derivative(cv, T[i])

        xi = np.polyval(cp, beta)
        vi = np.polyval(cv, beta)
        ai = np.polyval(ca, beta)

        t.append(ti)
        x.append(xi)
        v.append(vi)
        a.append(ai)

    x = np.hstack(x)
    v = np.hstack(v)
    a = np.hstack(a)
    t = np.hstack(t)
    return locals()