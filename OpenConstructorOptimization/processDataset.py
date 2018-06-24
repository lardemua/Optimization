#!/usr/bin/env python

# pcl_ply2pcd 00000000.ply tmp.pcd && pcl_pcd2ply tmp.pcd 00000000a.ply
import argparse
import subprocess

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


#---------------------------------------
#--- Argument parser
#---------------------------------------
ap = argparse.ArgumentParser()
ap.add_argument('directory', metavar='Directory',
                type=str, help='Put the directory of the files')

args = vars(ap.parse_args())

# Work directory
Directory = args['directory']
