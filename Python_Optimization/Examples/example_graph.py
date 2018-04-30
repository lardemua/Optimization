#!/usr/bin/env python
"""
This is a long, multiline description
"""

#########################
##    IMPORT MODULES   ##
#########################
import sys
import os
import cv2
from matplotlib import pyplot as plt
import pickle

import networkx as nx

#########################
##      HEADER         ##
#########################
__author__ = "Miguel Riem de Oliveira"
__date__ = "December 2015"
__copyright__ = "Copyright 2015, V3V"
__credits__ = ["Miguel Riem de Oliveira"]
__license__ = "GPL"
__version__ = "1.0"
__maintainer__ = "Miguel Oliveira"
__email__ = "m.riem.oliveira@gmail.com"
__status__ = "Development"


#########################
## FUNCTION DEFINITION ##
#########################


##########
## MAIN ##
##########

if __name__ == "__main__":

    #--------------------#
    ### Initialization ###
    #--------------------#

    # Start the graph and insert nodes (the images)
    G = nx.Graph()
    G.add_node('batata')
    G.add_node('cenoura')
    G.add_node('laranja')
    G.add_node('tangerina')
    G.add_node('alface')

    G.add_edge('batata', 'cenoura', weight=1)
    G.add_edge('batata', 'tangerina', weight=1)
    G.add_edge('batata', 'laranja', weight=1)
    G.add_edge('laranja', 'tangerina', weight=1)

    print('G is connected ' + str(nx.is_connected(G)))

    # Draw graph
    pos = nx.random_layout(G)
    colors = range(4)
    edges, weights = zip(*nx.get_edge_attributes(G, 'weight').items())
    nx.draw(G, pos, node_color='#A0CBE2', edgelist=edges, edge_color=weights, width=6,
            edge_cmap=plt.cm.Greys_r, with_labels=True, alpha=1, node_size=2500, font_color='k')
    plt.show()  # display
