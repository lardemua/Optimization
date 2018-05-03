import numpy as np
from numpy.linalg import inv


def points2imageFromT(T, K, P, dist):

    # Calculation of the point in the image in relation to the chess reference
    xypix = []

    fx = K[0, 0]
    fy = K[1, 1]
    cx = K[0, 2]
    cy = K[1, 2]

    k1 = dist[0, 0]
    k2 = dist[0, 1]
    p1 = dist[0, 2]
    p2 = dist[0, 3]
    k3 = dist[0, 4]

    P = P[:, 0:3]

    for p in P:

        rot = np.matrix(T[0: 3, 0: 3])
        xyz = rot.dot(p) + T[0: 3, 3]

        xl = xyz[0, 0] / xyz[0, 2]
        yl = xyz[0, 1] / xyz[0, 2]

        r_square = xl**2 + yl**2

        xll = xl * (1 + k1 * r_square + k2 * r_square**2 + k3 *
                    r_square**3) + 2 * p1 * xl * yl + p2 * (r_square + 2 * xl**2)
        yll = yl * (1 + k1 * r_square + k2 * r_square**2 + k3 *
                    r_square**3) + p1 * (r_square + 2 * yl**2) + 2 * p2 * xl * yl

        u = fx * xll + cx
        v = fy * yll + cy

        xypix.append([u, v])

    return np.array(xypix)


def costFunction(x, dist, intrinsics, X, Pc, detections):
    """Cost function
    """

    # Updates the transformations from the cameras and the arucos to the map defined in the X class using the x optimized vector
    X.fromVector(list(x))

    # Cost calculation
    cost = 0

    for detection in detections:
        camera = [camera for camera in X.cameras if camera.id ==
                  detection.camera[1:]][0]

        aruco = [aruco for aruco in X.arucos if aruco.id ==
                 detection.aruco[1:]][0]

        Ta = aruco.getT()
        Tc = camera.getT()
        T = np.matmul(inv(Tc), Ta)

        xypix = points2imageFromT(T, intrinsics, Pc, dist)

        # TODO Eucledean distance wrong
        for i in range(len(xypix)):
            distance = ((detection.corner[0][i, 0] - xypix[i, 0]
                         ) ** 2 + (detection.corner[0][i, 1] - xypix[i, 1])**2) ** (1/2.0)

            cost = cost + distance

    return cost
