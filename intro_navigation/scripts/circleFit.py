import numpy as np


def points_circle(xc, yc, r, n=100):
    x = [xc + np.cos(2*np.pi/n*val)*r for val in range(0, n+1)]
    y = [yc + np.sin(2*np.pi/n*val)*r for val in range(0, n+1)]
    return x, y


def fit_circle(x, y):
    # coordinates of the barycenter
    x_m = np.mean(x)
    y_m = np.mean(y)
    # calculation of the reduced coordinates
    u = x - x_m
    v = y - y_m
    # linear system defining the center (uc, vc) in reduced coordinates:
    #    Suu * uc +  Suv * vc = (Suuu + Suvv)/2
    #    Suv * uc +  Svv * vc = (Suuv + Svvv)/2
    Suv  = np.sum(u*v)
    Suu  = np.sum(u**2)
    Svv  = np.sum(v**2)
    Suuv = np.sum(u**2 * v)
    Suvv = np.sum(u * v**2)
    Suuu = np.sum(u**3)
    Svvv = np.sum(v**3)
    # Solving the linear system
    A = np.array([[Suu, Suv], [Suv, Svv]])
    B = np.array([Suuu + Suvv, Svvv + Suuv])/2.0
    uc, vc = np.linalg.solve(A, B)
    xc_1 = x_m + uc
    yc_1 = y_m + vc
    # Calcul des distances au centre (xc_1, yc_1)
    Ri_1 = np.sqrt((x-xc_1)**2 + (y-yc_1)**2)
    R_1 = np.mean(Ri_1)
    residu_1 = sum((Ri_1-R_1)**2)
    return xc_1, yc_1, R_1