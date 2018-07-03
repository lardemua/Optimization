import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt
import random
import cv2

from transformations import random_quaternion
from transformations import quaternion_slerp

#-------------------------------------------------------------------------------
#--- FUNCTION DEFINITION
#-------------------------------------------------------------------------------


##
# @brief Averages a list of transformations
#
# @param lT a list of transforms, each defined by a tuple ( (tx, ty, tz), (qw, qx, qy, qz) )
#
# @return the averaged transform, a tuple ( (tx,ty,tz), (qw, qx, qy, qz) )
def averageTransforms(l_transforms):
    N = len(l_transforms)
    print("Computing the average of " + str(N) + " transforms")

    # Get a list of all the translations l_t
    l_t = [i[0] for i in l_transforms]
    # print(l_t)

    # compute the average translation
    tmp1 = sum(v[0] for v in l_t) / N
    tmp2 = sum(v[1] for v in l_t) / N
    tmp3 = sum(v[2] for v in l_t) / N
    avg_t = (tmp1, tmp2, tmp3)
    # print(avg_t)

    # Get a list of the rotation quaternions
    l_q = [i[1] for i in l_transforms]
    #print l_q

    # Average the quaterions using an incremental slerp approach
    acc = 1.0  # accumulator, counts the number of observations inserted already
    avg_q = random_quaternion()  # can be random for start, since it will not be used
    for q in l_q:
        # How to deduce the ratio on each iteration
        # w_q = 1 / (acc + 1)
        # w_qavg = acc  / (acc + 1)
        # ratio = w_q / w_qavg <=> 1 / acc
        avg_q = quaternion_slerp(avg_q, q, 1.0/acc)  # run pairwise slerp
        acc = acc + 1  # increment acc

    avg_q = tuple(avg_q)
    # print(avg_q)

    return (avg_t, avg_q)


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


def costFunction(x, dist, intrinsics, X, Pc, detections, args, handles, handle_fun, s):
    """Cost function
    """

    # Updates the transformations from the cameras and the arucos to the map defined in the X class using the x optimized vector
    X.fromVector(list(x), args)
    width, height, _ = s[0].raw.shape

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

        # if np.isnan(distanceFourPoints):
        #     print "xcm = " + str(xcm)
        #     print "ycm = " + str(ycm)

        # if args['do'] and costFunction.counter in multiples:

        #     # draw
        #     k = int(camera.id)
        #     if args['option1'] == 'corners':
        #         for i in range(4):
        #             if 0 < xypix[i][0] < height and 0 < xypix[i][1] < width:
        #                 cv2.circle(s[k].raw, (int(xypix[i][0]), int(xypix[i][1])),
        #                            5, (230, 250, 100), -1)
        #     else:
        #         if 0 < xypix[0][0] < height and 0 < xypix[0][1] < width:
        #             cv2.circle(s[k].raw, (int(xypix[0][0]), int(xypix[0][1])),
        #                        5, (230, 250, 100), -1)
        #     cv2.imshow('camera'+str(k), s[k].raw)

        #     X.setPlot3D(Pc)

    if args['do'] and costFunction.counter in multiples:
        if handle_fun:
            handle_fun.set_ydata(cost)
        plt.draw()
        plt.pause(0.01)

    return cost

#-------------------------------------------------------------------------------
#--- OTHER FUNCTION DEFINITION
#-------------------------------------------------------------------------------


'''cap_module.py

The purpose of this module
is to provide functions
for copying and pasting
directories and files.

This is a level 1 module.'''

#=========================
# Level 1 Functions: Files
#=========================


def copy_file(path):
    '''copy_file(string)

    Import the needed functions.
    Assert that the path is a file.
    Return all file data.'''
    from os.path import basename, isfile
    assert isfile(path)
    return (basename(path), file(path, 'rb', 0).read())


def paste_file(file_object, path):
    '''paste_file(tuple, string)

    Import needed functions.
    Assert that the path is a directory.
    Create all file data.'''
    from os.path import isdir, join
    assert isdir(path)
    file(join(path, file_object[0]), 'wb', 0).write(file_object[1])

#===============================
# Level 2 Functions: Directories
#===============================


def copy_dir(path):
    '''copy_dir(string)

    Import needed functions.
    Assert that path is a directory.
    Setup a storage area.
    Write all data to the storage area.
    Return the storage area.'''
    from os import listdir
    from os.path import basename, isdir, isfile, join
    assert isdir(path)
    dir = (basename(path) + "dataset_optimized", list())
    for name in listdir(path):
        next_path = join(path, name)
        if isdir(next_path):
            dir[1].append(copy_dir(next_path))
        elif isfile(next_path):
            dir[1].append(copy_file(next_path))
    return dir


def paste_dir(dir_object, path):
    '''paste_dir(tuple, string)

    Import needed functions.
    Assert that the path is a directory.
    Edit the path and create a directory as needed.
    Create all directories and files as needed.'''
    from os import mkdir
    from os.path import isdir, join, exists
    from shutil import rmtree

    if exists(path + '/' + dir_object[0]):
        rmtree(path + '/' + dir_object[0])

    assert isdir(path)
    if dir_object[0] is not '':
        path = join(path, dir_object[0])
        mkdir(path)
    for object in dir_object[1]:
        if type(object[1]) is list:
            paste_dir(object, path)
        else:
            paste_file(object, path)
