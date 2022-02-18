import numpy as np
from scipy.optimize import minimize

def direction(theta, phi):
    '''Return the direction vector of a cylinder defined
    by the spherical coordinates theta and phi.
    '''
    return np.array([np.cos(phi) * np.sin(theta), np.sin(phi) * np.sin(theta),
                     np.cos(theta)])

def projection_matrix(w):
    '''Return the projection matrix  of a direction w.'''
    return np.identity(3) - np.dot(np.reshape(w, (3,1)), np.reshape(w, (1, 3)))

def skew_matrix(w):
    '''Return the skew matrix of a direction w.'''
    return np.array([[0, -w[2], w[1]],
                     [w[2], 0, -w[0]],
                     [-w[1], w[0], 0]])

def calc_A(Ys):
    '''Return the matrix A from a list of Y vectors.'''
    return sum(np.dot(np.reshape(Y, (3,1)), np.reshape(Y, (1, 3)))
            for Y in Ys)

def calc_A_hat(A, S):
    '''Return the A_hat matrix of A given the skew matrix S'''
    return np.dot(S, np.dot(A, np.transpose(S)))

def preprocess_data(Xs_raw):
    '''Translate the center of mass (COM) of the data to the origin.
    Return the prossed data and the shift of the COM'''
    n = len(Xs_raw)
    Xs_raw_mean = sum(X for X in Xs_raw) / n

    return [X - Xs_raw_mean for X in Xs_raw], Xs_raw_mean

def G(w, Xs):
    '''Calculate the G function given a cylinder direction w and a
    list of data points Xs to be fitted.'''
    n = len(Xs)
    P = projection_matrix(w)
    Ys = [np.dot(P, X) for X in Xs]
    A = calc_A(Ys)
    A_hat = calc_A_hat(A, skew_matrix(w))

    
    u = sum(np.dot(Y, Y) for Y in Ys) / n
    v = np.dot(A_hat, sum(np.dot(Y, Y) * Y for Y in Ys)) / np.trace(np.dot(A_hat, A))

    return sum((np.dot(Y, Y) - u - 2 * np.dot(Y, v)) ** 2 for Y in Ys)

def C(w, Xs):
    '''Calculate the cylinder center given the cylinder direction and 
    a list of data points.
    '''
    n = len(Xs)
    P = projection_matrix(w)
    Ys = [np.dot(P, X) for X in Xs]
    A = calc_A(Ys)
    A_hat = calc_A_hat(A, skew_matrix(w))

    return np.dot(A_hat, sum(np.dot(Y, Y) * Y for Y in Ys)) / np.trace(np.dot(A_hat, A))

def r(w, Xs):
    '''Calculate the radius given the cylinder direction and a list
    of data points.
    '''
    n = len(Xs)
    P = projection_matrix(w)
    c = C(w, Xs)

    return np.sqrt(sum(np.dot(c - X, np.dot(P, c - X)) for X in Xs) / n)

def fit(data, guess_angles=None):
    '''Fit a list of data points to a cylinder surface. The algorithm implemented
    here is from David Eberly's paper "Fitting 3D Data with a Cylinder" from 
    https://www.geometrictools.com/Documentation/CylinderFitting.pdf

    Arguments:
        data - A list of 3D data points to be fitted.
        guess_angles[0] - Guess of the theta angle of the axis direction
        guess_angles[1] - Guess of the phi angle of the axis direction
    
    Return:
        Direction of the cylinder axis
        A point on the cylinder axis
        Radius of the cylinder
        Fitting error (G function)
    '''
    Xs, t = preprocess_data(data)  

    # Set the start points

    start_points = [(0, 0), (np.pi / 2, 0), (np.pi / 2, np.pi / 2)]
    if guess_angles:
        start_points = guess_angles

    # Fit the cylinder from different start points 

    best_fit = None
    best_score = float('inf')

    for sp in start_points:
        fitted = minimize(lambda x : G(direction(x[0], x[1]), Xs),
                    sp, method='Powell', tol=1e-6)

        if fitted.fun < best_score:
            best_score = fitted.fun
            best_fit = fitted

    w = direction(best_fit.x[0], best_fit.x[1])

    return w, C(w, Xs) + t, r(w, Xs), best_fit.fun 
