#!/usr/bin/env python
"""Calibration Optimization Camera-Camera

Program whose function is to optimize the calibration of n cameras at the same time.
"""

#-------------------------------------------------------------------------------
#--- IMPORT MODULES
#-------------------------------------------------------------------------------
import time  # Estimate time of process
import argparse  # Read command line arguments
import numpy as np  # Arrays and opencv images
import cv2  # OpenCV library
import glob  # Finds all the pathnames
import os  # Using operating system dependent functionality
from tqdm import tqdm  # Show a smart progress meter
import matplotlib.pyplot as plt  # Library to do plots 2D
import matplotlib.animation as animation  # Animate plots
from matplotlib import gridspec  # Make subplott
from mpl_toolkits.mplot3d import Axes3D  # Library to do plots 3D
from scipy.sparse import lil_matrix  # Lib Sparse bundle adjustment
from scipy.optimize import least_squares  # Lib optimization

#-------------------------------------------------------------------------------
#--- DEFINITIONS
#-------------------------------------------------------------------------------


class stru:
    def __init__(self):
        self.filename = None
        self.raw = None
        self.gray = None
        self.corners = None
        self.xypix = None


#-------------------------------------------------------------------------------
#--- HEADER
#-------------------------------------------------------------------------------
__author__ = "Filipe Oliveira Costa"
__date__ = "Abril 2018"
__copyright__ = "Copyright 2015, V3V"
__credits__ = ["Filipe Oliveira Costa"]
__license__ = "GPL"
__version__ = "1.0"
__maintainer__ = "Filipe Costa"
__email__ = "costa.filipe@ua.pt"
__status__ = "Development"

#-------------------------------------------------------------------------------
#--- FUNCTION DEFINITION
#-------------------------------------------------------------------------------


def getWorldPoints(s, square_size):
    """Get Global coordinates

    """
    n_points = len(s[0].corners)

    nx = 8  # number squares to direction x
    ny = 6  # number squares to direction y
    worldPoints = np.zeros((n_points, 4))

    for iy in range(ny):
        y = iy * square_size
        for ix in range(nx):
            x = ix * square_size
            worldPoints[ix+nx*iy, 0] = x
            worldPoints[ix+nx*iy, 1] = y
            worldPoints[ix+nx*iy, 2] = 0
            worldPoints[ix+nx*iy, 3] = 1

    worldPoints = worldPoints.transpose()
    return worldPoints


def TFromDxyzDCM(dxyz, rod):
    """Transformation matrix

        Homogeneous transformation matrix from dxyz and DCM
    """
    T = np.zeros((4, 4))
    T[3, 3] = 1
    T[0:3, 3] = dxyz.transpose()
    DCM = cv2.Rodrigues(rod)
    T[0:3, 0:3] = DCM[0]

    return T


def intrinsicToVector(intrinsics):
    """Intrinsic vector

        Get a vector with the values of the interinsic matrix that will be iterated
    """
    intrinsic_vector = [intrinsics[0, 0], intrinsics[1, 1],
                        intrinsics[0, 2], intrinsics[1, 2]]

    return intrinsic_vector


def vectorToIntrinsic(intrinsic_vector):
    """Intrinsic matrix

        Get a interinsic matrix with the values of the vector
    """
    intrinsics = np.array([[intrinsic_vector[0], 0, intrinsic_vector[2], 0],
                           [0, intrinsic_vector[1], intrinsic_vector[3], 0],
                           [0, 0, 1, 0]])

    return intrinsics


# def points2imageOpenCV(objectPoints, rvec, tvec, cameraMatrix, distCoeffs):
#     [imagePoints, jacobian, aspectRatio] = cv2.projectPoints(
#         objectPoints, rvec, tvec, cameraMatrix, distCoeffs)

#     return imagePoints


def points2image(dxyz, rod, K, P, dist):
    """Converts world points into image points

        Computes the list of pixels given by the projection of P 3D points onto the image given the postion and intrinsics of the camera
    """
    # Compute T matrix from dxyz and rod
    T = TFromDxyzDCM(dxyz, rod)

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

    P = P.transpose()[:, 0:3]

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


def costFunction(x0, worldPoints, dist, s):
    """Cost function
    """
    # Geometric transformation
    for k in range(K):

        dxyz = x0[k*N: k*N+3]
        rod = x0[k*N+3: k*N+6]
        intrinsic_vector = x0[k*N+6: k*N+10]
        intrinsics = vectorToIntrinsic(intrinsic_vector)

        s[k].xypix = points2image(
            dxyz, rod, intrinsics, worldPoints, dist)

        ### Draw iterate projections ###

    # Cost calculation
    sum_dist = 0

    for ni in range(K):

        sum_dist = sum_dist + sum((s[ni].corners[:, 0][..., 0] - s[ni].xypix[:, 0])
                                  ** 2 + (s[ni].corners[:, 0][..., 1] - s[ni].xypix[:, 1])**2)

    fc = sum_dist ** (1/2.0)
    cost = fc

    return cost


# def objectiveFunction(x):
#     """Compute the error between the several aruco marker positions

#         This funcion uses the total reprojection error

#     """
#     residuals = 0

#     return residuals



#-------------------------------------------------------------------------------
#--- MAIN
#-------------------------------------------------------------------------------
if __name__ == "__main__":

    #---------------------------------------
    #--- Argument parser
    #---------------------------------------
    # ap = argparse.ArgumentParser()
    # ap.add_argument("-y", "--calibration_yaml",
    # help="path to the yaml camera calibration file", required=True)
    # args = vars(ap.parse_args())

    #---------------------------------------
    #--- Intitialization
    #---------------------------------------

    d = np.load("cameraParameters.npy")
    # Dictionary elements -> "mtx", "dist", "rvecs", "tvecs"

    # Intrinsic matrix
    mtx = np.zeros((3, 4))
    mtx[:, :3] = d.item().get('mtx')
    mtx[:, 3] = np.array([0, 0, 0]).transpose()  # homogenize

    dist = d.item().get('dist')
    rvecs = d.item().get('rvecs')
    tvecs = d.item().get('tvecs')
    # print d.item().get('name')
    # exit()

    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 105, 0.001)

    # prepare object points
    objp = np.zeros((8*6, 3), np.float32)
    objp[:, :2] = np.mgrid[0:8, 0:6].T.reshape(-1, 2)
    objp = objp * 0.105

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

    s = [stru() for i in range(3)]
    s[0].filename = '../images/0001.jpg'  # 8
    s[1].filename = '../images/0014.jpg'  # 11
    s[2].filename = '../images/0024.jpg'  # 3

    K = len(s)

    fig = plt.figure()
    gs = gridspec.GridSpec(K, K+1)

    for k in tqdm(range(K)):

        # load image
        s[k].raw = cv2.imread(s[k].filename)

        ###
        # # undistorct image
        # img = cv2.imread(s[k].filename)
        # h,  w = img.shape[:2]
        # newcameramtx, roi = cv2.getOptimalNewCameraMatrix(
        #     d.item().get('mtx'), d.item().get('dist'), (w, h), 1, (w, h))
        # s[k].raw = cv2.undistort(img, d.item().get(
        #     'mtx'), d.item().get('dist'), None, newcameramtx)
        ###
        s[k].gray = cv2.cvtColor(s[k].raw, cv2.COLOR_BGR2GRAY)

        # find the chess board corners
        ret, s[k].corners = cv2.findChessboardCorners(s[k].gray, (8, 6), None)

        # drawing sttuff
        ax = fig.add_subplot(gs[k, K])

        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)

            cv2.cornerSubPix(s[k].gray, s[k].corners,
                             (11, 11), (-1, -1), criteria)
            imgpoints.append(s[k].corners)

            # Draw and display the corners
            cv2.drawChessboardCorners(s[k].raw, (8, 6), s[k].corners, ret)
            # cv2.imshow('img', s[k].raw)

            ax.imshow(cv2.cvtColor(s[k].raw, cv2.COLOR_BGR2RGB))
            ax.axis('off')
            # cv2.waitKey(10)

    # Compute Chessboard 3D worl Coordinates
    worldPoints = getWorldPoints(s, 0.105)

    # Draw 3d plot with reference systems
    ax3D = fig.add_subplot(gs[:, :K], projection='3d')
    ax3D.scatter(worldPoints[0, :], worldPoints[1, :],
                 worldPoints[2, :], c='b', marker='*')
    ax3D.scatter(worldPoints[0, 0], worldPoints[1, 0],
                 worldPoints[2, 0], c='g', marker='o')

    # Referential
    exsize = 0.150
    ax3D.plot([0, exsize], [0, 0], [0, 0], 'r-')
    ax3D.plot([0, 0], [0, exsize], [0, 0], 'g-')
    ax3D.plot([0, 0], [0, 0], [0, exsize], 'b-')

    ax3D.set_xlabel('x axis')
    ax3D.set_ylabel('y axis')
    ax3D.set_zlabel('z axis')

    #---------------------------------------
    #--- Initial guess for parameters (create x0).
    #---------------------------------------
    x0 = []  # the parameters(extend list x0 as needed)

    # ---- Camera 1 - Position and Rotation
    idx = 7
    dxyz = [tvecs[idx][0][0], tvecs[idx][1][0], tvecs[idx][2][0]]
    rod = [rvecs[idx][0][0], rvecs[idx][1][0], rvecs[idx][2][0]]
    intrinsic_vector = intrinsicToVector(mtx)

    x0 = x0 + dxyz + rod + intrinsic_vector

    # ---- Camera 2 - Position and Rotation
    idx = 10
    dxyz = [tvecs[idx][0][0], tvecs[idx][1][0], tvecs[idx][2][0]]
    rod = [rvecs[idx][0][0], rvecs[idx][1][0], rvecs[idx][2][0]]
    intrinsic_vector = intrinsicToVector(mtx)

    x0 = x0 + dxyz + rod + intrinsic_vector

    # ---- Camera 3 - Position and Rotation
    idx = 2
    dxyz = [tvecs[idx][0][0], tvecs[idx][1][0], tvecs[idx][2][0]]
    rod = [rvecs[idx][0][0], rvecs[idx][1][0], rvecs[idx][2][0]]
    intrinsic_vector = intrinsicToVector(mtx)

    x0 = x0 + dxyz + rod + intrinsic_vector

    # convert list to array
    x0 = np.array(x0)

    # print x0
    print("number of parameters = " + str(len(x0)))

    # Draw initial estimate
    N = 10  # num_values_per_camera
    for k in tqdm(range(K)):
        dxyz = x0[k*N: k*N+3]
        rod = x0[k*N+3: k*N+6]
        intrinsic_vector = x0[k*N+6: k*N+10]
        intrinsics = vectorToIntrinsic(intrinsic_vector)

        s[k].xypix = points2image(
            dxyz, rod, intrinsics, worldPoints, dist)

        # Draw intial projections
        ax = fig.add_subplot(gs[k, K])
        plt.plot(s[k].xypix[:, 0], s[k].xypix[:, 1], 'r*')
        plt.plot(s[k].xypix[0, 0], s[k].xypix[0, 1], 'g*')

    #---------------------------------------
    #--- Test call of objective function
    #---------------------------------------
    # call objective function with initial guess (just for testing)

    # initial_residuals = objectiveFunction(x0)
    # print("initial_residuals = " + str(initial_residuals))

    #---------------------------------------
    #--- Set the bounds for the parameters
    #---------------------------------------
    # bounds : 2-tuple of array_like, optional
    # Lower and upper bounds on independent variables. Defaults to no bounds. Each array must match the size of x0 or be a scalar, in the latter case a bound will be the same for all variables. Use np.inf with an appropriate sign to disable bounds on all or some variables.
    # camera_params = params[:n_cameras * 9].reshape((n_cameras, 9))
    # points_3d = params[n_cameras * 9:].reshape((n_points, 3))

    # Bmin = []
    # Bmax = []
    # for i in range(0, len(views)):
    #     for i in range(0, 6):
    #         Bmin.append(-np.inf)
    #         Bmax.append(np.inf)

    # delta = 0.1
    # for i in range(len(views) * 6, len(x0)):
    #     Bmin.append(x0[i] - delta)
    #     Bmax.append(x0[i] + delta)

    # # for i in range(len(x0)-4, len(x0)):
    #     # Bmin.append(-np.inf)
    #     # Bmax.append(np.inf)

    # bounds = (Bmin, Bmax)

    #---------------------------------------
    #--- Optimization (minimization)
    #---------------------------------------
    print("\n\nStarting minimization")

    # --- Without sparsity matrix
    t0 = time.time()

    res = least_squares(costFunction, x0, verbose=2, x_scale='jac',
                        ftol=1e-6, xtol=1e-6, method='trf', args=(worldPoints, dist, s))
    # bounds=bounds
    t1 = time.time()

    print("Optimization took {0:.0f} seconds".format(t1 - t0))

    for k in tqdm(range(K)):
        ax = fig.add_subplot(gs[k, K])
        plt.plot(s[k].xypix[:, 0], s[k].xypix[:, 1], 'y*')
        plt.plot(s[k].xypix[0, 0], s[k].xypix[0, 1], 'g*')

    plt.show()

    #---------------------------------------
    #--- Present the results
    #---------------------------------------

    # solution_residuals = objectiveFunction(
    # res.x, marker_detections, views, pinhole_camera_model)

    # print("OPTIMIZATON FINISHED")
    # print("Initial (x0) average error = " + str(np.average(initial_residuals)))
    # print("Solution average error = " + str(np.average(solution_residuals)))
