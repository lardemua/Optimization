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


def TFromDxyzDCM(dxyz, rod):
    """Transformation matrix

        Homogeneous transformation matrix from dxyz and rod
    """
    T = np.zeros((4, 4))
    T[3, 3] = 1
    T[0:3, 3] = dxyz.transpose()
    DCM = cv2.Rodrigues(rod)
    T[0:3, 0:3] = DCM[0]

    return T


def DxyzDCMFromT(T):
    """Transformation matrix

        Homogeneous transformation matrix from DCM
    """
    dxyz = T[0:3, 3]
    dxyz = dxyz.transpose()
    rod, j = cv2.Rodrigues(T[0:3, 0:3])
    rod = rod.transpose()

    return dxyz, rod[0]


def points2image(dxyz, rod, K, P, dist):
    """Converts world points into image points

        Computes the list of pixels given by the projection of P 3D points onto the image given the postion and intrinsics of the camera
    """

    # Compute T matrix from dxyz and rod
    T = TFromDxyzDCM(dxyz, rod)

    points2imageFromT(T, K, P, dist)


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


def costFunction(x0, dist, intrinsics):
    """Cost function
    """
    # Geometric transformation
    worldPoints = np.zeros((len(marks)*4, 4))
    st = N*K
    for i in range(4*len(marks)):
        worldPoints[i, :] = x0[st+i*4: st+i*4+4]

    for i in range(K):
        s[i].worldPoints = np.zeros((len(s[i].ids)*4, 4))
        for j in range(len(s[i].ids)):
            n = [e for e in range(len(marks))
                 if marks[e].id == s[i].ids[j]]
            s[i].worldPoints[j*4:j*4+4, :] = worldPoints[n[0]*4:n[0]*4+4, :]

    for k in range(K):
        dxyz = x0[k*N: k*N+3]
        rod = x0[k*N+3: k*N+6]

        s[k].xypix = points2image(
            dxyz, rod, intrinsics, s[k].worldPoints, dist)

        ### Draw iterate projections ###

    # Cost calculation
    sum_dist = 0

    for ni in range(K):

        sum_dist = sum_dist + sum((s[ni].pix[:, 0] - s[ni].xypix[:, 0])
                                  ** 2 + (s[ni].pix[:, 1] - s[ni].xypix[:, 1])**2)

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

    # Read all images (each image correspond to a camera)
    images = sorted(
        glob.glob((os.path.join('../CameraImages/DataSet6', '*.png'))))

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
            print("camera " + str(k))
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

    # Get list of interest points in image
    for i in range(K):
        num_marks = len(s[i].ids)
        s[i].pix = np.zeros((4*num_marks, 2))
        for j in range(num_marks):
            s[i].pix[j*4: j*4+4, 0: 2] = s[i].corners[j][0]

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

    print GA.nodes
    print('GA is connected ' + str(nx.is_connected(GA)))

    # map_node = 'A446'  # to be defined by hand
    map_node = 'A595'  # to be defined by hand
    X = MyX()

    # cycle all nodes in graph
    for node in GA.nodes:

        print('Solving for ' + node)
        path = nx.shortest_path(GA, map_node, node)

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

            if is_camera:  # start is an aruco type node #seems to be validated!
                print('Will invert')
                Ti = inv(Ti)

            T = np.matmul(Ti, T)

            print("Ti = \n" + str(Ti))
            print("T = \n" + str(T))

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

    for detection in detections:

        # print(detection)
        T = detection.getT()
        # print T

        xypix = points2imageFromT(T, intrinsics, Pc, dist)

        # Draw intial projections
        ax = fig1.add_subplot(2, (K+1)/2, k+1)
        ax.plot(xypix[:, 0], xypix[:, 1], 'gd')
        ax.text(xypix[0, 0], xypix[0, 1], detection.aruco, color='yellow')

    # Draw projection 3D
    fig2 = plt.figure()
    ax3D = fig2.add_subplot(111, projection='3d')
    plt.title("3D projection of aruco markers")
    ax3D.set_xlabel('X')
    ax3D.set_ylabel('Y')
    ax3D.set_zlabel('Z')
    ax3D.set_aspect('equal')
    X.plotArucosIn3D(ax3D)

    #---------------------------------------
    #--- Test Miguel
    #---------------------------------------

    fig3 = plt.figure()
    ax3D = fig3.add_subplot(111, projection='3d')
    plt.title("3D projection of aruco markers")
    ax3D.set_xlabel('X')
    ax3D.set_ylabel('Y')
    ax3D.set_zlabel('Z')
    ax3D.set_aspect('equal')

    # Get the transforms
    A444_T_C0 = detections[0].getT()
    A595_T_C0 = detections[1].getT()

    s = 0.1
    # Define in the Origin of the A595 reference system
    P_A444 = np.array([[0, 0, 0, 1], [s, 0, 0, 1], [0, s, 0, 1], [
                      0, 0, s, 1]], dtype=np.float).transpose()
    P_O = np.array([[0, 0, 0, 1], [s, 0, 0, 1], [0, s, 0, 1],
                    [0, 0, s, 1]], dtype=np.float).transpose()

    print(A444_T_C0)
    print(inv(A444_T_C0))

    # Compute the P_A595 transformed to the C0 reference system
    P_C0 = np.matmul(A444_T_C0, P_A444)

    # Compute the P_A595 transformed to the A444 reference system in two ways (should give the same result)

    # By using the previously obtained P_C0
    P_A595 = np.matmul(np.matmul(inv(A444_T_C0), A595_T_C0), P_O)

    # By first combining all transformations from A595 to A444 and them multiplying the A595 point
    #P_A444_2 = np.matmul(np.matmul(inv(A444_T_C0), A595_T_C0), P_A595)

    # Plot all the points. The test consists in seeing of P_A444 and P_A444_2 are coincident

    def myPlot(P, label):
        # Function to plot origin and reference system
        o = P[0:3, 0]
        x = P[0:3, 1]
        y = P[0:3, 2]
        z = P[0:3, 3]
        # print(o)
        # print(x)
        # print(y)
        # print(z)
        ax3D.plot([o[0]], [o[1]], [o[2]], 'cs')
        ax3D.text(o[0], o[1], o[2], label, color='darkorchid')
        ax3D.plot([o[0], x[0]], [o[1], x[1]], [o[2], x[2]], 'r-')  # x axis
        ax3D.plot([o[0], y[0]], [o[1], y[1]], [o[2], y[2]], 'g-')  # y axis
        ax3D.plot([o[0], z[0]], [o[1], z[1]], [o[2], z[2]], 'b-')  # z axis

        ax3D.text(x[0], x[1], x[2], 'x', color='red')
        ax3D.text(y[0], y[1], y[2], 'y', color='green')
        ax3D.text(z[0], z[1], z[2], 'z', color='blue')

    myPlot(P_A444, 'P_A444')
    myPlot(P_C0, 'P_C0')
    myPlot(P_A595, 'P_A595')

    #P = P_A444_2
    #ax3D.plot([P[0]], [P[1]], [P[2]], marker='x',color='blue', markersize=16, linewidth=14)
    #ax3D.text(P[0], P[1], P[2], 'P_A444_2', color='darkorchid')

    ax3D.set_xlabel('X')
    ax3D.set_ylabel('Y')
    ax3D.set_zlabel('Z')
    ax3D.set_aspect('equal')

    #---------------------------------------
    #--- End of test Miguel
    #---------------------------------------

    plt.show()
    exit()

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

    res = least_squares(costFunction, np.array(X.v), verbose=2, x_scale='jac',
                        ftol=1e-10, xtol=1e-10, method='trf', args=(dist, intrinsics))
    # bounds=bounds
    t1 = time.time()

    print("Optimization took {0:.0f} seconds".format(t1 - t0))

    for k in tqdm(range(K)):
        ax = fig1.add_subplot(2, (K+1)/2, k+1)
        ax.plot(s[k].xypix[:, 0], s[k].xypix[:, 1], 'y*')
        ax.plot(s[k].xypix[0, 0], s[k].xypix[0, 1], 'g*')

    fig1.show()
    fig2.show()
    fig3.show()
    plt.waitforbuttonpress()

    #---------------------------------------
    #--- Present the results
    #---------------------------------------

    # solution_residuals = objectiveFunction(
    # res.x, marker_detections, views, pinhole_camera_model)

    # print("OPTIMIZATON FINISHED")
    # print("Initial (x0) average error = " + str(np.average(initial_residuals)))
    # print("Solution average error = " + str(np.average(solution_residuals)))
