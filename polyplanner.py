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

def local_planner_polynomial(x_list, v_list, T, plot=False):
    n = 4 # this will be fixed to support 3rd order polynomials
    if not len(x_list) == len(T) + 1:
        raise ValueError('x_list must be 1 longer than T')
    if not len(x_list) == len(v_list):
        raise ValueError('x_list and v_list must be same length')

    S = np.hstack([[0], np.cumsum(T)])
    n_legs = len(T)
    n_coeff = n_legs*n
    A = np.zeros((n_coeff, n_coeff))
    b = np.zeros(n_coeff)

    eq_num = 0
    for i in range(n_legs):        
        # p1(0) = x[0]
        A[eq_num, n*i:n*(i+1)] = trajectory_coefficients(beta=0, m=0, n=n, T=T[i])
        b[eq_num] = x_list[i]
        eq_num += 1
    
        # p1'(0) = v[0]
        A[eq_num, n*i:n*(i+1)] = trajectory_coefficients(beta=0, m=1, n=n, T=T[i])
        b[eq_num] = v_list[i]
        eq_num += 1
    
        # p1(1) = x[1]
        A[eq_num, n*i:n*(i+1)] = trajectory_coefficients(beta=1, m=0, n=n, T=T[i])
        b[eq_num] = x_list[i + 1]
        eq_num += 1
    
        # p1'(1) = v[1]
        A[eq_num, n*i:n*(i+1)] = trajectory_coefficients(beta=1, m=1, n=n, T=T[i])
        b[eq_num] = v_list[i + 1]
        eq_num += 1

    rank = np.linalg.matrix_rank(A)
    if rank < n_coeff:
        print('Matrix A not full rank, check constraints, rank: ', rank, '/', n_coeff)
        c = np.linalg.pinv(A).dot(b)
    else:
        c = np.linalg.inv(A).dot(b)
        
    t = []
    x = []
    v = []
    a = []
    for i in range(len(T)):
        ti = np.linspace(S[i], S[i+1])
        beta = time_scaling(ti, S[i], T[i])
        cp = c[n*i:n*(i+1)]
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