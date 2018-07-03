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

from transformations import random_quaternion
from transformations import quaternion_slerp
from transformations import quaternion_from_matrix
from transformations import quaternion_matrix

from os.path import basename
import subprocess

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
__version__ = "3.0"
__maintainer__ = "Filipe Costa"
__email__ = "costa.filipe@ua.pt"
__status__ = "Development"

#-------------------------------------------------------------------------------
#--- MY FUNCTIONS
#-------------------------------------------------------------------------------

##
# @brief Executes the command in the shell in a blocking manner
#
# @param cmd a string with teh command to execute
#
# @return


def bash(cmd):
    print "Executing command: " + cmd
    p = subprocess.Popen(
        cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    for line in p.stdout.readlines():
        print line,
        p.wait()


#-------------------------------------------------------------------------------
#--- MAIN
#-------------------------------------------------------------------------------
if __name__ == "__main__":

    #---------------------------------------
    #--- Argument parser
    #---------------------------------------

    ap = argparse.ArgumentParser()

    # ap.add_argument('--version', action='version', version='%(prog)s 2.0')
    ap.add_argument('dir', metavar='Directory',
                    type=str, help='Directory of the dataset to optimize')

    ap.add_argument('option1', choices=[
                    'center', 'corners'], help="Chose if is the centers or the corners of Arucos to do the otimization.")
    ap.add_argument('option2', choices=[
                    'all', 'translation'], help="Chose if use translation and rotation or only the translations in the vector x to otimize.")
    ap.add_argument('option3', choices=[
                    'fromaruco', 'fromfile'], help="Get initial estimation from the markers detection or from a file.")

    ap.add_argument("-d", action='store_true',
                    help="draw initial estimation", required=False)
    ap.add_argument("-no", action='store_true',
                    help="do not optimize", required=False)
    ap.add_argument("-do", action='store_true',
                    help="to draw during optimization", required=False)
    ap.add_argument("-saveResults", action='store_true',
                    help="Save results of the optimization", required=False)
    ap.add_argument("-processDataset", action='store_true',
                    help="Process the point clouds with the results obtained from the optimization process", required=False)

    ap.add_argument("-ms", metavar="marksize",
                    help="size of the aruco markers (m)", required=False)

    args = vars(ap.parse_args())

    if args['ms']:
        marksize = float(args['ms'])
    else:
        marksize = 0.082

    #---------------------------------------
    #--- Intitialization
    #---------------------------------------

    Directory = args['dir']

    newDirectory = os.path.dirname(Directory) + "/dataset_optimized"

    print Directory
    print newDirectory

    if True or args['option3'] == 'fromfile':

        if args['saveResults']:
            # Create new directory
            if not os.path.exists(newDirectory):
                os.makedirs(newDirectory)

            bash('cp ' + Directory + '/* ' + newDirectory + '/')
            # dir_object = copy_dir(Directory)
            # paste_dir(dir_object, Directory[:30])
            print "----------------------------\nNew path was created\n----------------------------"
            exit(0)
            # -------

        # Read all images (each image correspond to a camera)
        filenames = sorted(
            glob.glob((os.path.join(Directory, '*.jpg'))))

        # Read data calibration camera (Dictionary elements -> "mtx", "dist")
        parameters = []
        text_file = open("CameraParameters/Mycalibration.txt", "r")
        for value in text_file.read().split(', '):
            value = value.replace("\n", "")
            parameters.append(value)
        parameters = np.array(parameters)
        text_file.close()

        # Intrinsic matrix and distortion vector
        mtx = np.zeros((3, 3))
        mtx[0][0:3] = parameters[0:3]
        mtx[1][0:3] = parameters[3:6]
        mtx[2][0:3] = parameters[6:9]
        intrinsics = np.zeros((3, 4))
        intrinsics[:, :3] = mtx
        dist = np.zeros((1, 5))
        dist[:] = parameters[9:14]

        # Read data extrinsic calibration camera (OpenConstructor)
        textfilenames = sorted(
            glob.glob((os.path.join(Directory, '00*.txt'))))

        for w, cltx in enumerate(textfilenames):
            nt = len(cltx)
            if cltx[nt-6] == "-" or cltx[nt-7] == "-" or cltx[nt-8] == "-":
                del textfilenames[w]
    else:
        # Read all images (each image correspond to a camera)
        filenames = sorted(
            glob.glob((os.path.join(Directory, '*.jpg'))))

        # Read data calibration camera (Dictionary elements -> "mtx", "dist")
        d = np.load("CameraParameters/cameraParameters.npy")

        # TODO intrinsics
        # Intrinsic matrix and distortion vector
        mtx = d.item().get('mtx')
        intrinsics = np.zeros((3, 4))
        intrinsics[:, :3] = mtx
        dist = d.item().get('dist')

    # Define aruco dictionary
    aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
    parameters = aruco.DetectorParameters_create()

    s = [stru() for i in range(len(filenames))]

    # Detect Aruco Markers
    detections = []

    # print(filenames)
    # filenames = [x for x in filenames if x ==                 '../CameraImages/Aruco_Board_2/dataset/00000000.jpg']

    # exit(0)

    # K = len(filenames)  # number of cameras
    k = 0
    # for filename in filenames[0:50:30]:
    for filename in filenames:

        # load image
        raw = cv2.imread(filename)
        s[k].raw = raw
        gray = cv2.cvtColor(raw, cv2.COLOR_BGR2GRAY)

        # lists of ids and the corners beloning to each id
        corners, ids, _ = aruco.detectMarkers(
            gray, aruco_dict, parameters=parameters)

        font = cv2.FONT_HERSHEY_SIMPLEX  # font for displaying text

        if not ids is None:
            if len(ids) > 0:
                # print "----------------------------"
                # print("> Camera " + str(k))

                # Estimate pose of each marker
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    corners, marksize, mtx, dist)

                for rvec, tvec, idd, corner in zip(rvecs, tvecs, ids, corners):

                    detection = MyDetection(
                        rvec[0], tvec[0], 'C' + str(k), 'A' + str(idd[0]), corner)

                    # print(detection)
                    # print(detection.printValues())
                    detections.append(detection)

                    if args['d'] or args['do']:
                        aruco.drawAxis(raw, mtx, dist, rvec,
                                       tvec, 0.05)  # Draw Axis
                        cv2.putText(raw, "Id:" + str(idd[0]), (corner[0][0, 0],
                                                               corner[0][0, 1]), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

                raw = aruco.drawDetectedMarkers(raw, corners)

        if args['d'] or args['do']:
            # drawing sttuff
            size_square = 7
            for corner in corners:
                if args['option1'] == 'corners':
                    for ij in range(len(corner[0])):
                        x1 = int(corner[0][ij][0]-size_square)
                        y1 = int(corner[0][ij][1]-size_square)
                        x2 = int(corner[0][ij][0]+size_square)
                        y2 = int(corner[0][ij][1]+size_square)
                        cv2.rectangle(raw, (x1, y1), (x2, y2),
                                      (0, 0, 255), 2)
                else:
                    xcm = (corner[0][0, 0] + corner[0][1, 0] +
                           corner[0][2, 0] + corner[0][3, 0])/4
                    ycm = (corner[0][0, 1] + corner[0][1, 1] +
                           corner[0][2, 1] + corner[0][3, 1])/4
                    x1 = int(xcm-size_square)
                    y1 = int(ycm-size_square)
                    x2 = int(xcm+size_square)
                    y2 = int(ycm+size_square)
                    cv2.rectangle(raw, (x1, y1), (x2, y2), (0, 0, 255), 2)
        k = k+1

    width, height, _ = raw.shape

    #---------------------------------------
    #--- Get camera transformations by OpenConstructor.
    #---------------------------------------

    X = MyX()

    if args['option3'] == 'fromfile':
        print "\n" + "--------------------------------------------------------"
        print "Get initial extrinsic parameters of the cameras...\n"

        for j, namefile in enumerate(textfilenames):
            Tot = np.zeros((4, 4))
            txtfile = open(namefile)
            for i, line in enumerate(txtfile):
                if 0 < i < 5:
                    paramVect = []
                    for param in line.strip().split(' '):
                        paramVect.append(param)

                    Tot[i-1][0:] = np.array(paramVect)
            Tt = Tot.transpose()

            # T cameras from OpenContructor
            camera = MyCamera(T=Tt, id=str(j))
            X.cameras.append(camera)
            # --------
            txtfile.close()
            print "Initial Extrinsic matrix of camera " + str(j) + "...\n"
            print "T = \n" + str(Tt)
            print "\n--------------------------------------------------------"

    #---------------------------------------
    #--- Initial guess for parameters (create x0).
    #---------------------------------------

    # Start the Aruco nodes graph and insert nodes (the images)
    GA = nx.Graph()

    GA.add_edge('Map', 'C0', weight=1)

    if args['option3'] == 'fromfile':
        for i in range(1, len(filenames)):
            GA.add_edge('Map', 'C'+str(i), weight=1)

    for detection in detections:
        GA.add_edge(detection.aruco, detection.camera, weight=1)

    if args['d'] or args['do']:
        # Draw graph
        fig2 = plt.figure()

    pos = nx.random_layout(GA)
    pos = nx.spring_layout(GA)
    pos = nx.kamada_kawai_layout(GA)

    colors = range(4)
    edges, weights = zip(*nx.get_edge_attributes(GA, 'weight').items())

    if args['d'] or args['do']:
        # edge_labels = nx.draw_networkx_edge_labels(GA, pos)

        nx.draw(GA, pos, node_color='#A0CBE2', edgelist=edges, edge_color=weights, width=6, edge_cmap=plt.cm.Greys_r,
                with_labels=True, alpha=1, node_size=3500, font_color='k')  # , edge_labels=edge_labels)

        plt.waitforbuttonpress(0.1)

    print "----------------------------\n\n" + "Created Nodes:"
    print GA.nodes
    print "\n" + "----------------------------"
    print('-> GA is connected ' + str(nx.is_connected(GA)))
    print "----------------------------"

    if not nx.is_connected(GA):
        exit()

    map_node = 'Map'  # to be defined by hand

    while not map_node in GA.nodes:
        # raise ValueError('Must define a map that exists in the graph. Should be one of ' + str(GA.nodes))
        print 'Must define a map that exists in the graph. \nShould be one of ' + \
            str(GA.nodes)
        name = raw_input("Insert a valid map node: ")
        map_node = str(name)
        print "\n"

    print "-> Map node is " + map_node
    print "-----------------------\n"

    # cycle all nodes in graph
    for node in tqdm(GA.nodes):

        # print "--------------------------------------------------------"
        # print('Solving for ' + node + "...")

        # path = nx.shortest_path(GA, node, map_node)
        # print(path)

        paths_shortest = list(nx.all_shortest_paths(GA, node, map_node))
        shortest_path_lenght = len(paths_shortest[0])
        paths = paths_shortest

        # if len(paths_shortest) < 4:  # if small number of shortes paths compute some more
        #     paths_simple = list(nx.all_simple_paths(
        #         GA, node, map_node, cutoff=shortest_path_lenght+2))
        #     paths.extend(paths_simple)

        #     paths = [list(x) for x in set(tuple(x) for x in paths)]

        if paths == []:
            paths = [[node]]

        # for testing keep only the first element
        # paths = [paths[0]]
        # [[ 0.99928637  0.00437887  0.03751761  0.19445132]
        #  [-0.00443158  0.99998931  0.00132196 -0.0940675 ]
        #  [-0.03751142 -0.00148728  0.99929509 -0.66690812]
        #  [ 0.          0.          0.          1.        ]]

        # print(paths)

        # exit()

        # import random
        # path = random.choice(paths)

        transformations_for_path = []

        for path in paths:
            T = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [
                          0, 0, 1, 0], [0, 0, 0, 1]], dtype=np.float)

            for i in range(1, len(path)):
                start = path[i-1]
                end = path[i]
                start_end = [start, end]

                if start == 'Map':
                    if args['option3'] == 'fromaruco':
                        Ti = inv(np.identity(4))
                    else:
                        camera = [camera for camera in X.cameras if camera.id ==
                                  end[1:]][0]
                        Ti = inv(camera.getT())

                elif end == 'Map':
                    if args['option3'] == 'fromaruco':
                        Ti = np.identity(4)
                    else:
                        camera = [camera for camera in X.cameras if camera.id ==
                                  start[1:]][0]
                        Ti = camera.getT()

                else:
                    if start[0] == 'C':  # start is an aruco type node
                        is_camera = True
                    else:
                        is_camera = False

                    det = [
                        x for x in detections if x.aruco in start_end and x.camera in start_end][0]

                    # print(det)

                    Ti = det.getT()

                    if is_camera:  # Validated! When going from aruco to camera must invert the tranformation given by the aruco detection
                        # print('Will invert...')
                        Ti = inv(Ti)

                T = np.matmul(Ti, T)

                # print("Ti = \n" + str(Ti))
                # print("T = \n" + str(T))

            q = quaternion_from_matrix(T, isprecise=False)
            t = (tuple(T[0:3, 3]), tuple(q))
            # print t
            # exit()
            transformations_for_path.append(t)

        qm = averageTransforms(transformations_for_path)
        # print qm

        T = quaternion_matrix(qm[1])
        T[0, 3] = qm[0][0]
        T[1, 3] = qm[0][1]
        T[2, 3] = qm[0][2]
        # print T
        # exit()

        # print("Transformation from " + node +
        #       " to " + map_node + " is: \n" + str(T))

        if node[0] == 'C' and args['option3'] == 'fromaruco':  # node is a camera
            camera = MyCamera(T=T, id=node[1:])
            X.cameras.append(camera)
        elif node == 'Map':
            camera = MyCamera(T=T, id='Map')
            X.cameras.append(camera)
        else:
            aruco = MyAruco(T=T, id=node[1:])
            X.arucos.append(aruco)

    #---------------------------------------
    #--- Draw Initial guess.
    #---------------------------------------
    l = marksize
    if args['option1'] == 'corners':
        Pc = np.array([[-l/2, l/2, 0], [l/2, l/2, 0],
                       [l/2, -l/2, 0], [-l/2, -l/2, 0]])
    else:
        Pc = np.array([[0, 0, 0]])

    handles = []

    if args['d'] or args['do']:
        size_square = 5
        for detection in detections:
            camera = [camera for camera in X.cameras if camera.id ==
                      detection.camera[1:]][0]  # fetch the camera for this detection

            aruco = [aruco for aruco in X.arucos if aruco.id ==
                     detection.aruco[1:]][0]  # fetch the aruco for this detection

            Ta = aruco.getT()
            Tc = camera.getT()
            T = np.matmul(inv(Tc), Ta)

            xypix = points2imageFromT(T, intrinsics, Pc, dist)

            k = int(camera.id)

            # Draw intial projections
            if args['option1'] == 'corners':
                for i in range(4):
                    if 0 < xypix[i][0] < height and 0 < xypix[i][1] < width:
                        cv2.circle(s[k].raw, (int(xypix[i][0]), int(xypix[i][1])),
                                   5, (0, 128, 255), -1)
            else:
                if 0 < xypix[0][0] < height and 0 < xypix[0][1] < width:
                    x1 = int(xypix[0][0]-size_square)
                    y1 = int(xypix[0][1]-size_square)
                    x2 = int(xypix[0][0]+size_square)
                    y2 = int(xypix[0][1]+size_square)
                    cv2.rectangle(s[k].raw, (x1, y1),
                                  (x2, y2), (0, 128, 255), -1)

            cv2.namedWindow('camera'+str(k), flags=cv2.WINDOW_NORMAL)
            cv2.resizeWindow('camera'+str(k), width=480, height=270)
            cv2.imshow('camera'+str(k), s[k].raw)

        # Draw projection 3D
        plt.ion()
        fig3 = plt.figure()
        ax3D = fig3.add_subplot(111, projection='3d')
        plt.hold(True)
        # plt.title("3D projection of aruco markers")
        ax3D.set_xlabel('X (m)')
        ax3D.set_ylabel('Y (m)')
        ax3D.set_zlabel('Z (m)')
        ax3D.set_aspect('equal')
        X.plot3D(ax3D, 'k.', Pc)

        fig3.show()
        plt.waitforbuttonpress(0.1)

        key = ord('w')
        while key != ord('o'):
            key = cv2.waitKey(20)
            plt.waitforbuttonpress(0.01)

    # Get vector x0
    X.toVector(args)
    x0 = np.array(X.v, dtype=np.float)
    # print len(x0)
    # exit()

    import random
    x_random = x0 * np.array([random.uniform(0.85, 1.15)
                              for _ in xrange(len(x0))], dtype=np.float)
    # x0 = x_random

    #---------------------------------------
    #--- Test call of objective function
    #---------------------------------------

    costFunction.counter = 0

    # call objective function with initial guess (just for testing)
    initial_residuals = costFunction(
        x0, dist, intrinsics, X, Pc, detections, args, handles, None, s)

    handle_fun = None
    if args['d'] or args['do']:
        # Draw graph
        fig4 = plt.figure()
        axcost = fig4.add_subplot(111)
        plt.plot(initial_residuals, 'b', label="Initial residuals")

        handle_fun, = plt.plot(initial_residuals, 'r--',
                               label="Final residuals")
        plt.legend(loc='best')
        axcost.set_xlabel('Detections')
        axcost.set_ylabel('Cost')
        plt.ylim(ymin=-1)
        plt.waitforbuttonpress(0.1)
        # plt.xticks([])

    print("\n-> Initial cost = " + str(initial_residuals)) + "\n"

    if not args['no']:

        #---------------------------------------
        #--- Create the sparse matrix
        #---------------------------------------

        M = len(detections)
        N = len(x0)

        A = lil_matrix((M, N), dtype=int)

        id_detection = 0

        for detection in detections:

            camera = [camera for camera in X.cameras if camera.id ==
                      detection.camera[1:]][0]

            idxs_camera = X.idxsFromCamera(camera.id, args)

            aruco = [aruco for aruco in X.arucos if aruco.id ==
                     detection.aruco[1:]][0]

            idxs_aruco = X.idxsFromAruco(aruco.id, args)

            idxs = np.append(idxs_camera, idxs_aruco)

            A[id_detection, idxs] = 1

            id_detection = id_detection + 1

        print('A shape = ' + str(A.shape))
        print('A =\n' + str(A.toarray()))

        # if args['d'] or args['do']:
        #     # Draw graph
        #     fig5 = plt.figure()
        #     plt.imshow(A.toarray(), cmap='Greys',  interpolation='nearest')
        #     plt.waitforbuttonpress()

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

        # cost_random = costFunction(x_random, dist, intrinsics, s, X, Pc)
        # print(x_random)

        # exit(0)

        # --- Without sparsity matrix
        t0 = time.time()

        # Method
        res = least_squares(costFunction, x0, verbose=2, jac_sparsity=A, x_scale='jac', ftol=1e-4,
                            xtol=1e-4, method='trf', args=(dist, intrinsics, X, Pc, detections, args, handles, handle_fun, s))

        # print(res.x)
        t1 = time.time()

        print("\nOptimization took {0:.0f} seconds".format(t1 - t0))

        X.fromVector(list(res.x), args)

        # if args['d'] or args['do']:
        #     X.plot3D(ax3D, 'g*', Pc)

        #---------------------------------------
        #--- Present the results
        #---------------------------------------

        solution_residuals = costFunction(
            res.x, dist, intrinsics, X, Pc, detections, args, handles, None, s)

        print("\nOPTIMIZATON FINISHED")
        print("Initial (x0) average error = " +
              str(np.average(initial_residuals)))
        print("Solution average error = " +
              str(np.average(solution_residuals))) + "\n"

        #---------------------------------------
        #--- Save results
        #---------------------------------------
        if args['saveResults']:
            print "saving results..."

            textfilenames = sorted(
                glob.glob((os.path.join(newDirectory, '00*.txt'))))
            for w, cltx in enumerate(textfilenames):
                nt = len(cltx)
                if cltx[nt-6] == "-" or cltx[nt-7] == "-" or cltx[nt-8] == "-":
                    del textfilenames[w]

            for j, namefile in enumerate(textfilenames):

                camera = [camera for camera in X.cameras if camera.id ==
                          str(j)][0]
                T = camera.getT()
                Tp = T.transpose()

                lines = open(namefile).read().splitlines()
                for i in range(4):
                    lines[i+1] = str(Tp[i][0]) + ' ' + str(Tp[i][1]) + ' ' + \
                        str(Tp[i][2]) + ' ' + str(Tp[i][3])

                open(namefile, 'w').write('\n'.join(lines))
                print "Save T optimized for camera " + str(j) + "..."

        #---------------------------------------
        #--- Draw final results
        #---------------------------------------
        if args['d'] or args['do']:
            # fig5 = plt.figure()
            # plt.plot(solution_residuals, 'r--')
            handle_fun.set_ydata(solution_residuals)
            plt.waitforbuttonpress(0.1)

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
                if args['option1'] == 'corners':
                    for i in range(4):
                        if 0 < xypix[i][0] < height and 0 < xypix[i][1] < width:
                            cv2.circle(s[k].raw, (int(xypix[i][0]), int(xypix[i][1])),
                                       5, (0, 255, 255), -1)
                else:
                    if 0 < xypix[0][0] < height and 0 < xypix[0][1] < width:
                        cv2.circle(s[k].raw, (int(xypix[0][0]), int(xypix[0][1])),
                                   5, (0, 255, 255), -1)
                cv2.imshow('camera'+str(k), s[k].raw)

            while key != ord('q'):
                key = cv2.waitKey()

        if args['processDataset'] and args['saveResults']:

            bash('./processDataset.py ' + Directory)
