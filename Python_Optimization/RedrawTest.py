#!/usr/bin/env python
"""
Test plots with sets. Check https://github.com/FilipeOCosta/Optimization/issues/15
Example of how to draw and redraw 3d points
"""

#-------------------------------------------------------------------------------
#--- IMPORT MODULES
#-------------------------------------------------------------------------------
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time

#-------------------------------------------------------------------------------
#--- DEFINITIONS
#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
#--- HEADER
#-------------------------------------------------------------------------------
__author__ = "Miguel Oliveira"
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


#-------------------------------------------------------------------------------
#--- MAIN
#-------------------------------------------------------------------------------
if __name__ == "__main__":

    #50 random points
    N = 15
    x = np.random.uniform(low=0, high=1, size=(N,))
    y = np.random.uniform(low=0, high=1, size=(N,))
    z = np.random.uniform(low=0, high=1, size=(N,))

    #Prepare figure
    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim3d(0,1)
    ax.set_ylim3d(0,1)
    ax.set_zlim3d(0,1)

    plt.hold(True) #to draw text and points

    #Plot the points and get a handle to the plot
    handle_scatter = ax.scatter(x, y, z, s=10) #Draw N points
    handle_plot, = ax.plot(x, y, z, 'm--') #Draw lines between all points
    handle_text = ax.text(x[0], y[0], z[0], "A777", color='darkorchid') #Draw text on the first point

    fig.show()

    #Resample the points and redraw
    for i in range(0, 20):

        #resample 3d coordinates
        x = np.random.uniform(low=0, high=1, size=(N,))
        y = np.random.uniform(low=0, high=1, size=(N,))
        z = np.random.uniform(low=0, high=1, size=(N,))

        #redraw scatter
        handle_scatter._offsets3d = (x,y,z)

        #redraw plot
        #handle_plot.set_position((x[0], y[0]))
        #handle_plot._offsets3d = (x[0],y[0],z[0])
        handle_plot.set_xdata(x)
        handle_plot.set_ydata(y)
        handle_plot.set_3d_properties(zs=z)


        #redraw text
        handle_text.set_position((x[0], y[0]))
        handle_text.set_3d_properties(z=z[0], zdir='x')
        handle_text.set_text('A' + str(i))

        plt.draw()
        plt.pause(1)
