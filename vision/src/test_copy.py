#!/usr/bin/env python3

import numpy as np
from numpy.linalg.linalg import norm
from sklearn.cluster import KMeans
import bezier
from wire_modeling.Bezier import Bezier
from wire_modeling.wire_sim import *


# insert wire info here 
L = 1
M = 0.028
Ks = 13.1579
Kb = 6.13
Kd = 5.26
wire_model = WireModel(L,M,Ks,Kb,Kd)



# grasp Object Info -> use a custom ros message to pack this information
gop = np.array([[1.5],[0.75],[0.6]]) # position of grasp object
god = np.array([[0.35],[0.2],[0.2]]) # dimensions of grasp object -> model the grasp object as a box
# orientation of grasp object wrt to world in order (xyz)
rx = 0 # rot about x
ry = 0 # rot about y
rz = 3.14 # rot about z
grasp_object = GraspObject(gop,god,rx,ry,rz)


# dual robot configuration
left_robot_location = np.array([[0],[0.25],[0]])
right_robot_location = np.array([[0],[0.75],[0]])
robot_reach = 0.75
distance_apart = 0.5
robots = DualRobotConfig(left_robot_location, right_robot_location,robot_reach,distance_apart)

# wire sim set up
N = 20
wire_sim_ = WireSim(N,wire_model,grasp_object, robots )

# this will be produced from point cloud data
wire = np.zeros((3,N))
wire[0] = np.ones((1,N))
wire[1] = np.linspace(0,1,N)
wire[2] = 0.75*np.ones((1,N))

sim_result = wire_sim_.simulate(wire)
#wire_sim_.find_optimal_action(sim_result)
print(sim_result)
