import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt
import random

#-------------------------------------------------------------------------------
#--- FUNCTION DEFINITION
#-------------------------------------------------------------------------------


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


def costFunction(x, dist, intrinsics, X, Pc, detections, args, handles, handle_fun):
    """Cost function
    """

    # Updates the transformations from the cameras and the arucos to the map defined in the X class using the x optimized vector
    X.fromVector(list(x), args)

    # Cost calculation
    cost = []
    costFunction.counter += 1
    # print costFunction.counter
    multiples = [100*n for n in range(1, 100+1)]

    if not handles:
        handles = detections

    for detection, handle in zip(detections, handles):
        camera = [camera for camera in X.cameras if camera.id ==
                  detection.camera[1:]][0]

        aruco = [aruco for aruco in X.arucos if aruco.id ==
                 detection.aruco[1:]][0]

        Ta = aruco.getT()
        Tc = camera.getT()
        T = np.matmul(inv(Tc), Ta)

        xypix = points2imageFromT(T, intrinsics, Pc, dist)

        distanceFourPoints = 0

        if args['option1'] == 'corners':
            for i in range(len(xypix)):

                distance = ((detection.corner[0][i, 0] - xypix[i, 0]
                             ) ** 2 + (detection.corner[0][i, 1] - xypix[i, 1])**2) ** (1/2.0)

                distanceFourPoints = distanceFourPoints + distance
        else:
            xcm = (detection.corner[0][0, 0] + detection.corner[0][1, 0] +
                   detection.corner[0][2, 0] + detection.corner[0][3, 0])/4
            ycm = (detection.corner[0][0, 1] + detection.corner[0][1, 1] +
                   detection.corner[0][2, 1] + detection.corner[0][3, 1])/4

            distanceFourPoints = (
                (xcm - xypix[0, 0]) ** 2 + (ycm - xypix[0, 1]) ** 2) ** (1/2.0)

        cost.append(distanceFourPoints)

        if args['do'] and costFunction.counter in multiples:

            # draw
            # redraw plot 2D
            handle.handle_scatter.set_offsets(xypix[:, :])

            # redraw text 2D
            handle.handle_text.set_position((xypix[0, 0], xypix[0, 1]))

            X.setPlot3D(Pc)

    if args['do'] and costFunction.counter in multiples:
        if handle_fun:
            handle_fun.set_ydata(cost)
        plt.draw()
        plt.pause(0.01)

    return cost
