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
import sys
import os  # Using operating system dependent functionality
from tqdm import tqdm  # Show a smart progress meter
import matplotlib.pyplot as plt  # Library to do plots 2D
import matplotlib.animation as animation  # Animate plots
from matplotlib import gridspec  # Make subplott
from mpl_toolkits.mplot3d import Axes3D  # Library to do plots 3D
from scipy.sparse import lil_matrix  # Lib Sparse bundle adjustment
from scipy.optimize import least_squares  # Lib optimization
import cv2.aruco as aruco  # Aruco Markers
import pickle
import networkx as nx
from numpy.linalg import inv
from itertools import combinations

#-------------------------------------------------------------------------------
#--- DEFINITIONS
#-------------------------------------------------------------------------------
from myClasses import *

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


# def TFromDxyzDCM(dxyz, rod):
#     """Transformation matrix

#         Homogeneous transformation matrix from dxyz and rod
#     """
#     T = np.zeros((4, 4))
#     T[3, 3] = 1
#     T[0:3, 3] = dxyz.transpose()
#     DCM = cv2.Rodrigues(rod)
#     T[0:3, 0:3] = DCM[0]

#     return T


# def DxyzDCMFromT(T):
#     """Transformation matrix

#         Homogeneous transformation matrix from DCM
#     """
#     dxyz = T[0:3, 3]
#     dxyz = dxyz.transpose()
#     rod, j = cv2.Rodrigues(T[0:3, 0:3])
#     rod = rod.transpose()

#     return dxyz, rod[0]


# def points2image(dxyz, rod, K, P, dist):
#     """Converts world points into image points

#         Computes the list of pixels given by the projection of P 3D points onto the image given the postion and intrinsics of the camera
#     """

#     # Compute T matrix from dxyz and rod
#     T = TFromDxyzDCM(dxyz, rod)

#     points2imageFromT(T, K, P, dist)


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


def costFunction(x0, dist, intrinsics, X, Pc):
    """Cost function
    """

    X.fromVector(list(x0))

    # Cost calculation
    sum_dist = 0

    for k in range(K):
        camera = [camera for camera in X.cameras if camera.id == str(k)][0]

        for j in range(len(s[k].ids)):

            a_id = s[k].ids[j]
            aruco = [aruco for aruco in X.arucos if aruco.id ==
                     str(a_id[0])][0]

            Ta = aruco.getT()
            Tc = camera.getT()
            T = np.matmul(inv(Tc), Ta)

            xypix = points2imageFromT(T, intrinsics, Pc, dist)

            # # Draw intial projections
            # ax = fig1.add_subplot(2, (K+1)/2, k+1)
            # ax.plot(xypix[:, 0], xypix[:, 1], 'g.')
            # ax.text(xypix[0, 0], xypix[0, 1],
            #         "A" + str(a_id[0]), color='yellow')

            sum_dist = sum_dist + sum((s[k].corners[j][0][:, 0] - xypix[:, 0])
                                      ** 2 + (s[k].corners[j][0][:, 1] - xypix[:, 1])**2)

    cost = sum_dist ** (1/2.0)

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

    # Read all images (each image correspond to a camera)
    images = sorted(
        glob.glob((os.path.join('../CameraImages/DataSet5', '*.png'))))

    K = len(images)

    # Read data calibration camera (Dictionary elements -> "mtx", "dist")
    d = np.load("CameraParameters/cameraParameters.npy")

    # Define aruco dictionary
    aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
    parameters = aruco.DetectorParameters_create()
    marksize = 0.082

    # Intrinsic matrix
    mtx = d.item().get('mtx')
    intrinsics = np.zeros((3, 4))
    intrinsics[:, :3] = mtx

    dist = d.item().get('dist')

    # Detect Aruco Markers
    s = [stru() for i in range(K)]

    for i in range(K):
        s[i].filename = images[i]

    fig1 = plt.figure()

    detections = []

    for k in range(K):

        # load image
        s[k].raw = cv2.imread(s[k].filename)
        s[k].gray = cv2.cvtColor(s[k].raw, cv2.COLOR_BGR2GRAY)

        # lists of ids and the corners beloning to each id
        s[k].corners, s[k].ids, rejectedImgPoints = aruco.detectMarkers(
            s[k].gray, aruco_dict, parameters=parameters)

        font = cv2.FONT_HERSHEY_SIMPLEX  # font for displaying text

        nids = len(s[k].ids)

        if np.all(nids != 0):
            print "----------------------------"
            print("> Camera " + str(k))
            s[k].rvec, s[k].tvec, _ = aruco.estimatePoseSingleMarkers(
                s[k].corners, marksize, mtx, dist)  # Estimate pose of each marker

            for i in range(len(s[k].rvec)):
                rvec = s[k].rvec[i][0]
                tvec = s[k].tvec[i][0]

                detection = MyDetection(
                    rvec, tvec, 'C' + str(k), 'A' + str(s[k].ids[i][0]))

                print(detection)
                print(detection.printValues())
                detections.append(detection)

            for i in range(nids):
                aruco.drawAxis(s[k].raw, mtx, dist, s[k].rvec[i],
                               s[k].tvec[i], 0.05)  # Draw Axis

                cv2.putText(s[k].raw, "Id: " + str(s[k].ids[i][0]), (s[k].corners[i][0][0][0], s[k].corners[i][0][0][1]),
                            font, 5, (76, 0, 153), 5, cv2.LINE_AA)

            s[k].raw = aruco.drawDetectedMarkers(s[k].raw, s[k].corners)

        # drawing sttuff
        ax = fig1.add_subplot(2, (K+1)/2, k+1)
        ax.imshow(cv2.cvtColor(s[k].raw, cv2.COLOR_BGR2RGB))
        ax.axis('off')
        plt.title("camera " + str(k))

    #---------------------------------------
    #--- Initial guess for parameters (create x0).
    #---------------------------------------

    # Find list of markers ids existents
    list_of_ids = []
    for i in range(K):
        a = list_of_ids
        b = []
        [b.append(id[0]) for id in s[i].ids]
        c = [e for e in b if e not in a]
        if len(c) > 0:
            [list_of_ids.append(ci) for ci in c]

    # Start the Aruco nodes graph and insert nodes (the images)
    GA = nx.Graph()

    for a_id in list_of_ids:
        # find all cameras that detected this aruco
        cameras = []
        for i in range(K):  # cycle all cameras
            aruco_cam_ids = s[i].ids
            if a_id in aruco_cam_ids:
                GA.add_edge("A" + str(a_id), "C" + str(i), weight=1)

    fig3 = plt.figure()
    # Draw graph
    pos = nx.random_layout(GA)
    colors = range(4)
    edges, weights = zip(*nx.get_edge_attributes(GA, 'weight').items())

    edge_labels = nx.draw_networkx_edge_labels(GA, pos)

    nx.draw(GA, pos, node_color='#A0CBE2', edgelist=edges, edge_color=weights, width=6,
            edge_cmap=plt.cm.Greys_r, with_labels=True, alpha=1, node_size=3500, font_color='k', edge_labels=edge_labels)

    print "----------------------------\n\n" + "Created Nodes:"
    print GA.nodes
    print "\n" + "----------------------------"
    print('GA is connected ' + str(nx.is_connected(GA)))
    print "----------------------------\n"

    map_node = 'A595'  # to be defined by hand

    if not map_node in GA.nodes:
        raise ValueError(
            'Must define a map that exists in the graph. Should be one of ' + str(GA.nodes))

    X = MyX()

    # cycle all nodes in graph
    for node in GA.nodes:

        print "--------------------------------------------------------"
        print('Solving for ' + node + "...")
        path = nx.shortest_path(GA, node, map_node)

        T = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0],
                      [0, 0, 0, 1]], dtype=np.float)

        for i in range(1, len(path)):
            start = path[i-1]
            end = path[i]
            start_end = [start, end]

            if start[0] == 'C':  # start is an aruco type node
                is_camera = True
            else:
                is_camera = False

            det = [
                x for x in detections if x.aruco in start_end and x.camera in start_end][0]

            print(det)

            Ti = det.getT()

            if is_camera:  # Validated! When going from aruco to camera must invert the tranformation given by the aruco detection
                print('Will invert...')
                Ti = inv(Ti)

            T = np.matmul(Ti, T)

            # print("Ti = \n" + str(Ti))
            # print("T = \n" + str(T))

        if node[0] == 'C':  # node is a camera
            # derive rvec tvec from T
            camera = MyCamera(T=T, id=node[1:])
            X.cameras.append(camera)
        else:
            aruco = MyAruco(T=T, id=node[1:])
            X.arucos.append(aruco)

    l = 0.082
    Pc = np.array([[-l/2, l/2, 0], [l/2, l/2, 0],
                   [l/2, -l/2, 0], [-l/2, -l/2, 0]])

    for k in range(K):
        camera = [camera for camera in X.cameras if camera.id == str(k)][0]

        for a_id in s[k].ids:
            aruco = [aruco for aruco in X.arucos if aruco.id ==
                     str(a_id[0])][0]

            Ta = aruco.getT()
            Tc = camera.getT()
            T = np.matmul(inv(Tc), Ta)

            xypix = points2imageFromT(T, intrinsics, Pc, dist)

            # Draw intial projections
            ax = fig1.add_subplot(2, (K+1)/2, k+1)
            ax.plot(xypix[:, 0], xypix[:, 1], 'rd')
            ax.text(xypix[0, 0], xypix[0, 1],
                    "A" + str(a_id[0]), color='yellow')

    # Draw projection 3D
    fig2 = plt.figure()
    ax3D = fig2.add_subplot(111, projection='3d')
    plt.title("3D projection of aruco markers")
    ax3D.set_xlabel('X')
    ax3D.set_ylabel('Y')
    ax3D.set_zlabel('Z')
    ax3D.set_aspect('equal')
    X.plotArucosIn3D(ax3D)

    # plt.show()
    # exit()

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

    X.toVector()
    x0 = np.array(X.v, dtype=np.float)

    # --- Without sparsity matrix
    t0 = time.time()

    res = least_squares(costFunction, x0, verbose=2, x_scale='jac',
                        ftol=1e-10, xtol=1e-10, method='trf', args=(dist, intrinsics, X, Pc))
    # bounds=bounds
    t1 = time.time()

    print("\nOptimization took {0:.0f} seconds".format(t1 - t0))

    plt.show()

    #---------------------------------------
    #--- Present the results
    #---------------------------------------

    # solution_residuals = objectiveFunction(
    # res.x, marker_detections, views, pinhole_camera_model)

    # print("OPTIMIZATON FINISHED")
    # print("Initial (x0) average error = " + str(np.average(initial_residuals)))
    # print("Solution average error = " + str(np.average(solution_residuals)))
