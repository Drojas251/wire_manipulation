#!/usr/bin/env python3
import numpy as np
from numpy import linalg as la
from numpy.lib.function_base import vectorize
from mpl_toolkits import mplot3d

import numpy as np
import matplotlib.pyplot as plt
import math
# FLOW of Service 
# Get PC size ~ 
# Pass PC to State estimation to output 3x20 vector representing wire 
# Pass 3x20 vector to sim function. Output is 8x3x20 tensor representing results to 8 actions 
# Pass 8x3x20 to Optimizer function to select an solution and return the Pick and Pull pair that goes with that solution 
# Return Pick and Pull pair 



class DualRobotConfig:
    def __init__(self,left_robot_location, right_robot_location,robot_reach,distance_apart):
        self.r1 = left_robot_location # robot 1 is the robot on the left ( if you are standing behind them) 3x1 vector
        self.r2 = right_robot_location # robot 2 is the robot on the right ( if you are standing behind them) 3x1 vector
        self.robot_reach = robot_reach # the maximum safe reach of the robot- 1x1 scalar
        self.distance_apart = distance_apart # lateral distance between the two robots ( assume then are next to eachother)- 1x1 scalar

    def workspace_volume(self):
        workspace_volume = np.zeros((3,8))

        scale_on_x = 0.75
        scale_on_z = 0.85
        x = scale_on_x*self.robot_reach
        y = self.robot_reach - self.distance_apart
        z = scale_on_z*self.robot_reach

        workspace_volume[:,[0]] = np.array([self.r2[0] + x, self.r2[1] + y, self.r2[2] + z]) # top right forward 
        workspace_volume[:,[1]] = np.array([self.r1[0] + x, self.r1[1] - y, self.r1[2] + z]) # top left forward 
        workspace_volume[:,[2]] = np.array([self.r2[0] + x, self.r2[1] + y, self.r2[2]]) # bottom right forward 
        workspace_volume[:,[3]] = np.array([self.r1[0] + x, self.r1[1] - y, self.r1[2]]) # bottom left forward 

        workspace_volume[:,[4]] = np.array([self.r2[0] - x, self.r2[1] + y, self.r2[2] + z]) # top right back 
        workspace_volume[:,[5]] = np.array([self.r1[0] - x, self.r1[1] - y, self.r1[2] + z]) # top left back 
        workspace_volume[:,[6]] = np.array([self.r2[0] - x, self.r2[1] + y, self.r2[2]]) # bottom right back 
        workspace_volume[:,[7]] = np.array([self.r1[0] - x, self.r1[1] - y, self.r1[2]]) # bottom left back

        return workspace_volume 

    def mid_point_of_reference(self):
        mid_point = np.array([self.r1[0], (self.r1[1] + self.r2[1])/2, 0.6*self.robot_reach])
        return mid_point


class GraspObject:
    def __init__(self,gop,god,rx,ry,rz): 
        self.gop = gop # grasp object position
        self.god = god # grasp object dimension
        self.rx = rx # rotations
        self.ry = ry
        self.rz = rz
        self.normal_axis = np.array([[1],[0],[0]]) # normal axis of the object is always in the +x axis of the object frame 

    def rotation_matix(self):
      rotation_x = np.array([[1,0,0],[0, math.cos(self.rx), -1*math.sin(self.rx)],[0,math.sin(self.rx),math.cos(self.rx)]])
      rotation_y = np.array([[math.cos(self.ry), 0 , math.sin(self.ry)], [0 , 1, 0], [-1*math.sin(self.ry), 0 , math.cos(self.ry)]])
      rotation_z = np.array([[math.cos(self.rz), -1*math.sin(self.rz), 0], [math.sin(self.rz), math.cos(self.rz), 0], [0, 0, 1]])
      ROT = rotation_x@rotation_y@rotation_z
      return ROT

    def grasp_volume_in_local_frame(self):
        # forward is in the +x axis 
        dx = self.god[0]/2
        dy = self.god[1]/2
        dz = self.god[2]/2

        grasp_volume_in_local_frame = np.zeros((3,8))

        grasp_volume_in_local_frame[:,[0]] = np.array([dx, dy, dz]) # top right forward 
        grasp_volume_in_local_frame[:,[1]] = np.array([dx, -dy, dz]) # top left forward 
        grasp_volume_in_local_frame[:,[2]] = np.array([dx, dy, -dz]) # bottom right forward 
        grasp_volume_in_local_frame[:,[3]] = np.array([dx, -dy, -dz]) # bottom left forward 

        grasp_volume_in_local_frame[:,[4]] = np.array([-dx, dy, dz]) # top right back 
        grasp_volume_in_local_frame[:,[5]] = np.array([-dx, -dy, dz]) # top left back 
        grasp_volume_in_local_frame[:,[6]] = np.array([-dx, dy, -dz]) # bottom right back 
        grasp_volume_in_local_frame[:,[7]] = np.array([-dx, -dy, -dz]) # bottom left back 

        return grasp_volume_in_local_frame
    
    def grasp_volume_in_global_frame(self):
        # grasp volume in the global frame
        grasp_volume_in_local_frame = self.grasp_volume_in_local_frame()
        ROT = self.rotation_matix()
        grasp_volume_in_global_frame = np.zeros((3,8))

        for i in range(8):
            grasp_volume_in_global_frame[:,[i]] = self.gop + ROT@grasp_volume_in_local_frame[:,[i]]

        return grasp_volume_in_global_frame

class WireModel:
    def __init__(self,L,M,Ks,Kb,Kd): 
        self.L = L #length of wire
        self.M = M # mass of wire
        self.Ks = Ks # Stifness of structural spring
        self.Kb = Kb # stiffness of bending spring
        self.Kd = Kd # damping

        

    

class WireSim:
  def __init__(self,N,wire_model, grasp_object,robot):
    self.N = N # number of nodes that represents the wire
    self.wire_model = wire_model
    self.m = self.wire_model.M/self.N # individual node mass
    self.l = self.wire_model.L/(self.N - 1) # resting length between nodes
    self.actions = 8
    self.grasp_object = grasp_object
    self.robot = robot
    

  def __force_for_struct_springs(self,wire,V):

    F_s = np.zeros((3,self.N)) # structural spring force vectors

    for i in range(self.N):

        # Forces at ends are equal to zero (Do this to fix the two ends. Revise this and figure out how to not make these zero without breaking wire)
        if i == 0 or i == (self.N - 1):
            F_s[:,[i]] = np.zeros((3,1))

        else:
            F_s[:,[i]] = np.ones((3,1))

            l_prime_1 = la.norm(wire[:,[i+1]] - wire[:,[i]]) # new distance from node i to node (i+1)
            l_prime_2 = la.norm(wire[:,[i]] - wire[:,[i-1]]) # new distance from node i to node (i-1)

            # Calculate spring force on node i from neighboring nodes
            if i == 1:
                F1 = 2*self.wire_model.Ks*((self.l/l_prime_2) - 1)*(wire[:,[i]] - wire[:,[i-1]])
                F2 = -self.wire_model.Ks*((self.l/l_prime_1) - 1)*(wire[:,[i+1]] - wire[:,[i]])

            elif i == (self.N - 2):
                F1 = self.wire_model.Ks*((self.l/l_prime_2) - 1)*(wire[:,[i]] - wire[:,[i-1]])
                F2 = -2*self.wire_model.Ks*((self.l/l_prime_1) - 1)*(wire[:,[i+1]] - wire[:,[i]])

            else:
                F1 = self.wire_model.Ks*((self.l/l_prime_2) - 1)*(wire[:,[i]] - wire[:,[i-1]])
                F2 = -self.wire_model.Ks*((self.l/l_prime_1) - 1)*(wire[:,[i+1]] - wire[:,[i]])


            # INCLUDE DAMPING FORCE
            x1 = wire[:,[i]] - wire[:,[i-1]]
            F1_d = -self.wire_model.Kd*np.matmul(x1*np.transpose(x1), V[:,[i]] - V[:,[i-1]])/(la.norm(x1)**2)

            x2 = wire[:,[i+1]] - wire[:,[i]]
            F2_d = self.wire_model.Kd*np.matmul(x2*np.transpose(x2), V[:,[i+1]] - V[:,[i]])/(la.norm(x2)**2)



            F_s[:,[i]] = F1 + F2  + F2_d + F1_d

    return F_s

  def __force_for_bending_springs(self,wire):    

      F_b = np.zeros((3,self.N)) # Bending spring force vectors
      
      # only considering nodes 2 - 19
      for i in range(1,self.N - 1):

          # Case 1: forces on 2nd and 3rd nodes 
          if i == 1 or i == 2:
              l_prime = la.norm(wire[:,[i+2]] - wire[:,[i]])
              F_b[:,[i]] = -self.wire_model.Kb*((self.l/l_prime) - 1)*wire[:,[i+2]] - wire[:,[i]] 
          # Case 2: forces on 18th and 19th node
          elif i == (self.N - 2) or i == (self.N -3):
              l_prime = la.norm(wire[:,[i]] - wire[:,[i-2]])
              F_b[:,[i]] = self.wire_model.Kb*((self.l/l_prime) - 1)*wire[:,[i]] - wire[:,[i-2]]
          # All other nodes    
          else:
              l_prime_1 = la.norm(wire[:,[i+2]] - wire[:,[i]]) # new distance from node i to node (i+2)
              l_prime_2 = la.norm(wire[:,[i]] - wire[:,[i-2]]) # new distance from node i to node (i-2)

              F1 = -self.wire_model.Kb*((self.l/l_prime_1) - 1)*wire[:,[i+2]] - wire[:,[i]]
              F2 = self.wire_model.Kb*((self.l/l_prime_2) - 1)*wire[:,[i]] - wire[:,[i-2]]

              F_b[:,[i]] = F1 + F2

      return F_b

  def __normal_vec(self,ref_vec):

      # check for any zeros to avoid dividing by zero
      if ref_vec[2] != 0:
          situation = 3
      elif ref_vec[1] != 0:
          situation = 2
      elif ref_vec[0] != 0:
          situation = 1
      # assigns values to two unkowns and solves the third to get normal vector 
      # The equation being solved in the dot product = 0
      if situation == 3:
          a = 1
          b = 1
          c = -(a*ref_vec[0,0] + b*ref_vec[1,0])/ref_vec[2,0]
          
      if situation == 2:
          a = 1
          c = 1
          b = -(a*ref_vec[0,0] + c*ref_vec[2,0])/ref_vec[1,0]

      if situation == 1:
          b = 1
          c = 1
          a = -(b*ref_vec[1,0] + c*ref_vec[2,0])/ref_vec[0,0]    


      norm_vec = np.array([[a],[b],[c]])
      norm_vec = norm_vec/la.norm(norm_vec)   
      return norm_vec 

  def get_normal_vectors(self,wire,grasp_index):

      # from one normal vector, we can find the other 7

      grasp_index = grasp_index - 1 # grasp index is interms of the node number, but we reduce by one to track in python array index
      vectors = np.zeros((3,8))

      ref_vec = wire[:,[grasp_index + 1]] - wire[:,[grasp_index]]
      norm_vec = self.__normal_vec(ref_vec)

      norv1 = np.cross(np.transpose(ref_vec),np.transpose(norm_vec))
      norv1 = norv1/la.norm(norv1)
      vectors[:,[0]] = np.transpose(norv1)

      norv2 = np.cross(np.transpose(norm_vec),np.transpose(ref_vec))
      norv2 = norv2/la.norm(norv2)
      vectors[:,[1]] = np.transpose(norv2)

      norv3 = np.cross(norv2,np.transpose(ref_vec))
      norv3 = norv3/la.norm(norv3)
      vectors[:,[2]] = np.transpose(norv3)

      norv4 = np.cross(np.transpose(ref_vec),norv2)
      norv4 = norv4/la.norm(norv4)
      vectors[:,[3]] = np.transpose(norv4)

      mid1 = vectors[:,[0]] - vectors[:,[2]]
      mid2 = vectors[:,[0]] - vectors[:,[3]]
      mid3 = vectors[:,[1]] - vectors[:,[2]]
      mid4 = vectors[:,[1]] - vectors[:,[3]]

      vectors[:,[4]] = mid1/la.norm(mid1)
      vectors[:,[5]] = mid2/la.norm(mid2)
      vectors[:,[6]] = mid3/la.norm(mid3)
      vectors[:,[7]] = mid4/la.norm(mid4)

      return vectors

  def plot_norm_vecs(self,wire,grasp_index,norm_vecs):
      ax = plt.axes(projection='3d')

      zline = wire[[2],:]
      xline = wire[[1],:]
      yline = wire[[0],:]
      ax.scatter3D(xline, yline, zline, 'gray')

      for i in range(8):
        vec = np.zeros((3,2))
        vec[:,[0]] = wire[:,[grasp_index]]
        vec[:,[1]] = wire[:,[grasp_index]] + norm_vecs[:,[i]]
        # Data for a three-dimensional line
        zline = vec[[2],:].flatten()
        xline = vec[[1],:].flatten()
        yline = vec[[0],:].flatten()
        ax.plot3D(xline, yline, zline, 'gray')


  def simulate(self,wire):
      wire_set = np.zeros((self.actions,3,self.N))
      dt = 0.01
      grasp_index = 10
      F_vect = self.get_normal_vectors(wire,grasp_index)
      #self.plot_norm_vecs(wire,grasp_index,F_vect)
      

      for i in range(self.actions):

          v = np.zeros((3,self.N))
          a_k = np.zeros((3,self.N))
          v_k = np.zeros((3,self.N))

          TOL = 0.1
          stop = 0
          count = 0
          MAX_iterations = 300 
          wire_sim = wire

          while stop == 0:   

              F_s = self.__force_for_struct_springs(wire_sim,v)
              F_b = self.__force_for_bending_springs(wire_sim)
              F_net = F_s #+ F_b
              F_net[2,1:(self.N-1)] = F_net[2,1:(self.N-1)] - self.m*9.81

              F_applied = 0.5*F_vect[:,[i]]
              F_net[:,[grasp_index]] = F_net[:,[grasp_index]] + F_applied

              #Simulate
              a_k_1 = F_net/self.m

              v_k_1 = v_k + 0.5*(dt**2)*(a_k_1 + a_k)
              wire_sim = wire_sim + dt*v_k + 0.25*(dt**2)*(a_k_1 + a_k)

              a_k = a_k_1
              v_k = v_k_1

              wire_sim[:,[0]] = wire[:,[0]]
              wire_sim[:,[self.N-1]] = wire[:,[self.N -1]]

              if count == MAX_iterations:
                  stop = 1
              count = count + 1
          wire_set[[i],:,:] = wire_sim
          

          # plotting
          ax = plt.axes(projection='3d')
          for j in range(8):

            # Data for a three-dimensional line
            zline = wire_set[[j],[2],:].flatten()
            xline = wire_set[[j],[1],:].flatten()
            yline = wire_set[[j],[0],:].flatten()

            ax.scatter3D(xline, yline, zline, 'gray')

          zline = wire[[2],:]
          xline = wire[[1],:]
          yline = wire[[0],:]
          ax.scatter3D(xline, yline, zline, 'gray')

          for i in range(8):
            vec = np.zeros((3,2))
            vec[:,[0]] = wire[:,[grasp_index]]
            vec[:,[1]] = wire[:,[grasp_index]] + F_vect[:,[i]]
            # Data for a three-dimensional line
            zline = vec[[2],:].flatten()
            xline = vec[[1],:].flatten()
            yline = vec[[0],:].flatten()
            ax.plot3D(xline, yline, zline, 'gray')  

          #ax.view_init(60, 35)
          #fig  
          
      self.projections(wire_set)
      return wire_set

  def dot_product(self,u,v):
      dp = sum([x*y for (x, *x2), y in zip(u,v)])
      return dp
  
  def rotation_matix(self,rx,ry,rz):
      rotation_x = np.array([[1,0,0],[0, math.cos(rx), -1*math.sin(rx)],[0,math.sin(rx),math.cos(rx)]])
      rotation_y = np.array([[math.cos(ry), 0 , math.sin(ry)], [0 , 1, 0], [-1*math.sin(ry), 0 , math.cos(ry)]])
      rotation_z = np.array([[math.cos(rz), -1*math.sin(rz), 0], [math.sin(rz), math.cos(rz), 0], [0, 0, 1]])
      ROT = rotation_x@rotation_y@rotation_z
      return ROT
  def homo_trans_matrix(self,translation,ROT):
      HTM = np.array([[ROT[0][0], ROT[0][1], ROT[0][2], translation[0][0]],
                    [ROT[1][0],ROT[1][1],ROT[1][2],translation[1][0]],
                    [ROT[2][0],ROT[2][1],ROT[2][2],translation[2][0]],
                    [0 , 0, 0, 1]])
      return HTM

  def vector_to_homo(self,vector):
      homo_vector = np.array([[vector[0][0]], [vector[1][0]], [vector[2][0]], [1] ])
      return homo_vector

  def homo_to_vector(self,homo):
      vector = np.array([[homo[0][0]], [homo[1][0]], [homo[2][0]] ])
      return vector

  #def projections(self,wire_set):
  #    # Exlude for now
  #    projections = np.zeros((self.actions,3,self.N))
  #    ROT = self.rotation_matix(self.grasp_object.rx, self.grasp_object.ry, self.grasp_object.rz)
  #    grasp_axis = ROT@self.grasp_object.normal_axis
  #
  #    ## getting an error with matrix multiplication
  #    #for i in range(8):
  #    #    for j in range(self.N):
  #    #        projections[[i],:,[j]] = wire_set[[i],:,[j]] - self.dot_product((np.transpose(wire_set[[1],:,[2]]) - self.gop),grasp_axis)*grasp_axis
    
     







          

L = 1
M = 0.028
Ks = 13.1579
Kb = 6.13
Kd = 5.26
wire_model = WireModel(L,M,Ks,Kb,Kd)


# grasp Object Info -> use a custom ros message to pack this information
gop = np.array([[1],[0.25],[0.6]]) # position of grasp object
god = np.array([[0.075],[0.05],[0.05]]) # dimensions of grasp object -> model the grasp object as a box
# orientation of grasp object wrt to world in order (xyz)
rx = 0 # rot about x
ry = 0 # rot about y
rz = 1.5707 # rot about z

grasp_object = GraspObject(gop,god,rx,ry,rz)
ROT = grasp_object.rotation_matix()


# dual robot configuration
left_robot_location = np.array([[0],[0.1],[0]])
right_robot_location = np.array([[0],[-0.1],[0]])
robot_reach = 0.5
distance_apart = 0.3
R1 = DualRobotConfig(left_robot_location, right_robot_location,robot_reach,distance_apart)


# wire sim set up
N = 20
L = 1
M = 0.028
wire_sim_ = WireSim(N,wire_model,grasp_object, R1 )



wire = np.zeros((3,N))
wire[0] = np.ones((1,N))
wire[1] = np.linspace(0,1,N)
wire[2] = 2*np.ones((1,N))

#normal_vectors = p1.get_normal_vectors(test,10)
#sim_result = wire_sim_.simulate(wire)

# TO DO
# port the following functions from MATLAB in the following order
#
# find_optimal_vector.m