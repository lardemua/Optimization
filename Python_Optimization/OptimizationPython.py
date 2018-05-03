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
from costFunctions import *

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
#--- MAIN
#-------------------------------------------------------------------------------
if __name__ == "__main__":

    #---------------------------------------
    #--- Argument parser
    #---------------------------------------

    ap = argparse.ArgumentParser()
    ap.add_argument("-d", action='store_true',
                    help="draw initial estimation", required=False)
    ap.add_argument("-no", action='store_true',
                    help="do not optimize", required=False)
    ap.add_argument("-do", action='store_true',
                    help="to draw during optimization", required=False)

    args = vars(ap.parse_args())

    #---------------------------------------
    #--- Intitialization
    #---------------------------------------

    # Read all images (each image correspond to a camera)
    filenames = sorted(
        glob.glob((os.path.join('../CameraImages/DataSet1', '*.png'))))

    # Read data calibration camera (Dictionary elements -> "mtx", "dist")
    d = np.load("CameraParameters/cameraParameters.npy")

    # Define aruco dictionary
    aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
    parameters = aruco.DetectorParameters_create()
    marksize = 0.082

    # Intrinsic matrix and distortion vector
    mtx = d.item().get('mtx')
    intrinsics = np.zeros((3, 4))
    intrinsics[:, :3] = mtx
    dist = d.item().get('dist')

    if args['d'] or args['do']:
        fig1 = plt.figure()

    # Detect Aruco Markers
    detections = []

    K = len(filenames)  # number of cameras
    k = 0
    for filename in filenames:

        # load image
        raw = cv2.imread(filename)
        gray = cv2.cvtColor(raw, cv2.COLOR_BGR2GRAY)

        # lists of ids and the corners beloning to each id
        corners, ids, _ = aruco.detectMarkers(
            gray, aruco_dict, parameters=parameters)

        font = cv2.FONT_HERSHEY_SIMPLEX  # font for displaying text

        if len(ids) > 0:
            print "----------------------------"
            print("> Camera " + str(k))
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, marksize, mtx, dist)  # Estimate pose of each marker

            for rvec, tvec, idd, corner in zip(rvecs, tvecs, ids, corners):

                detection = MyDetection(
                    rvec[0], tvec[0], 'C' + str(k), 'A' + str(idd[0]), corner)

                print(detection)
                print(detection.printValues())
                detections.append(detection)

                if args['d'] or args['do']:
                    aruco.drawAxis(raw, mtx, dist, rvec,
                                   tvec, 0.05)  # Draw Axis
                    cv2.putText(raw, "Id:" + str(idd[0]), (corner[0][0, 0],
                                                           corner[0][0, 1]), font, 5, (0, 255, 0), 5, cv2.LINE_AA)

            raw = aruco.drawDetectedMarkers(raw, corners)

        if args['d'] or args['do']:
            # drawing sttuff
            ax = fig1.add_subplot(2, (K+1)/2, k+1)
            ax.imshow(cv2.cvtColor(raw, cv2.COLOR_BGR2RGB))
            ax.axis('off')
            plt.title("camera " + str(k))

            for corner in corners:
                ax.scatter(corner[0][:, 0], corner[0][:, 1], marker='s',
                           facecolors='none', edgecolors='r')
        k = k+1

    #---------------------------------------
    #--- Initial guess for parameters (create x0).
    #---------------------------------------

    # Start the Aruco nodes graph and insert nodes (the images)
    GA = nx.Graph()

    for detection in detections:
        GA.add_edge(detection.aruco, detection.camera, weight=1)

    if args['d'] or args['do']:
        # Draw graph
        fig3 = plt.figure()

    pos = nx.random_layout(GA)
    colors = range(4)
    edges, weights = zip(*nx.get_edge_attributes(GA, 'weight').items())

    if args['d'] or args['do']:
        edge_labels = nx.draw_networkx_edge_labels(GA, pos)

        nx.draw(GA, pos, node_color='#A0CBE2', edgelist=edges, edge_color=weights, width=6, edge_cmap=plt.cm.Greys_r,
                with_labels=True, alpha=1, node_size=3500, font_color='k', edge_labels=edge_labels)

    print "----------------------------\n\n" + "Created Nodes:"
    print GA.nodes
    print "\n" + "----------------------------"
    print('-> GA is connected ' + str(nx.is_connected(GA)))
    print "----------------------------"

    if not nx.is_connected(GA):
        exit()

    map_node = 'A471'  # to be defined by hand

    while not map_node in GA.nodes:
        # raise ValueError('Must define a map that exists in the graph. Should be one of ' + str(GA.nodes))
        print 'Must define a map that exists in the graph. \nShould be one of ' + \
            str(GA.nodes)
        name = raw_input("Insert a valid map node: ")
        map_node = str(name)
        print "\n"

    print "-> Map node is " + map_node
    print "-----------------------\n"

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
        print("Transformation from " + node +
              " to " + map_node + " is: \n" + str(T))

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

    if args['d'] or args['do']:
        for detection in detections:
            camera = [camera for camera in X.cameras if camera.id ==
                      detection.camera[1:]][0]

            aruco = [aruco for aruco in X.arucos if aruco.id ==
                     detection.aruco[1:]][0]

            Ta = aruco.getT()
            Tc = camera.getT()
            T = np.matmul(inv(Tc), Ta)

            xypix = points2imageFromT(T, intrinsics, Pc, dist)

            k = int(camera.id)

            # Draw intial projections
            ax = fig1.add_subplot(2, (K+1)/2, k+1)
            ax.plot(xypix[:, 0], xypix[:, 1], '+c')
            ax.text(xypix[0, 0], xypix[0, 1], "A" + aruco.id, color='yellow')

        # Draw projection 3D
        fig2 = plt.figure()
        ax3D = fig2.add_subplot(111, projection='3d')
        plt.title("3D projection of aruco markers")
        ax3D.set_xlabel('X')
        ax3D.set_ylabel('Y')
        ax3D.set_zlabel('Z')
        ax3D.set_aspect('equal')
        X.plotArucosIn3D(ax3D, 'k.')

    # Get vector x0
    X.toVector()
    x0 = np.array(X.v, dtype=np.float)

    #---------------------------------------
    #--- Test call of objective function
    #---------------------------------------

    # call objective function with initial guess (just for testing)
    initial_residuals = costFunction(x0, dist, intrinsics, X, Pc, detections)

    print("\n-> Initial cost = " + str(initial_residuals)) + "\n"

    if not args['no']:
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

        import random
        x_random = x0 * np.array([random.uniform(0.9, 1.1)
                                  for _ in xrange(len(x0))], dtype=np.float)

        # cost_random = costFunction(x_random, dist, intrinsics, s, X, Pc, fig1)
        # print(x_random)

        # exit(0)

        # --- Without sparsity matrix
        t0 = time.time()

        # # Method 1
        # res = least_squares(costFunction, x0, verbose=2, loss='soft_l1',
        #                     f_scale=0.1, args=(dist, intrinsics, X, Pc, detections))

        # Method 2
        res = least_squares(costFunction, x0, verbose=2, x_scale='jac', ftol=1e-10,
                            xtol=1e-10, method='dogbox', args=(dist, intrinsics, X, Pc, detections))

        # print(res.x)
        t1 = time.time()

        print("\nOptimization took {0:.0f} seconds".format(t1 - t0))

        X.fromVector(list(res.x))

        if args['d'] or args['do']:
            X.plotArucosIn3D(ax3D, 'g*')

        #---------------------------------------
        #--- Present the results
        #---------------------------------------

        solution_residuals = costFunction(
            res.x, dist, intrinsics, X, Pc, detections)

        print("\nOPTIMIZATON FINISHED")
        print("Initial (x0) average error = " +
              str(np.average(initial_residuals)))
        print("Solution average error = " +
              str(np.average(solution_residuals))) + "\n"

    if args['d'] or args['do']:
        plt.show()
