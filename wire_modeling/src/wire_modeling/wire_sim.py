#!/usr/bin/env python3
import numpy as np
from numpy import linalg as la
from numpy.lib.function_base import vectorize
from mpl_toolkits import mplot3d

import numpy as np
import matplotlib.pyplot as plt
import math
from wire_modeling.wire_grasp_toolbox import WireGraspToolbox

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
        self.max_arm_thickness = 0.08 # thickness of the arm or gripper in meters

    def workspace_volume(self):
        # get the workspace volume for the two robots
        # depends on where they are in the world, where they are with respect to eachother 
        # and their max reach
        workspace_volume = np.zeros((3,8))

        scale_on_x = 0.75
        scale_on_z = 1.0
        x = scale_on_x*self.robot_reach
        y = self.robot_reach - self.distance_apart
        z = scale_on_z*self.robot_reach

        workspace_volume[:,[0]] = np.array([self.r2[0] + x, self.r2[1] - y, self.r2[2] + z]) # top right forward 
        workspace_volume[:,[1]] = np.array([self.r1[0] + x, self.r1[1] + y, self.r1[2] + z]) # top left forward 
        workspace_volume[:,[2]] = np.array([self.r2[0] + x, self.r2[1] - y, self.r2[2]]) # bottom right forward 
        workspace_volume[:,[3]] = np.array([self.r1[0] + x, self.r1[1] + y, self.r1[2]]) # bottom left forward 

        workspace_volume[:,[4]] = np.array([self.r2[0] - x, self.r2[1] - y, self.r2[2] + z]) # top right back 
        workspace_volume[:,[5]] = np.array([self.r1[0] - x, self.r1[1] + y, self.r1[2] + z]) # top left back 
        workspace_volume[:,[6]] = np.array([self.r2[0] - x, self.r2[1] - y, self.r2[2]]) # bottom right back 
        workspace_volume[:,[7]] = np.array([self.r1[0] - x, self.r1[1] + y, self.r1[2]]) # bottom left back

        return workspace_volume 

    def mid_point_of_reference(self):
        # point of reference that is inbetween both robots
        mid_point = np.array([[self.r1[0][0]], [(self.r1[1][0] + self.r2[1][0])/2], [0.6*self.robot_reach]])
        return mid_point

    def robot_grasp_volume(self):
        # this volume represents the volume taken up by the robot arm near the grasp object
        # the volume around the robot_grasp_volume in its local frame
        # forward is in its +x axis 
        dx = self.max_arm_thickness
        dy = self.max_arm_thickness/2
        dz = self.max_arm_thickness/2

        robot_grasp_volume = np.zeros((3,8))

        robot_grasp_volume[:,[0]] = np.array([[dx], [dy], [dz]]) # top right forward 
        robot_grasp_volume[:,[1]] = np.array([[dx], [-dy], [dz]]) # top left forward 
        robot_grasp_volume[:,[2]] = np.array([[dx], [dy], [-dz]]) # bottom right forward 
        robot_grasp_volume[:,[3]] = np.array([[dx], [-dy], [-dz]]) # bottom left forward 

        robot_grasp_volume[:,[4]] = np.array([[0], [dy], [dz]]) # top right back 
        robot_grasp_volume[:,[5]] = np.array([[0], [-dy], [dz]]) # top left back 
        robot_grasp_volume[:,[6]] = np.array([[0], [dy], [-dz]]) # bottom right back 
        robot_grasp_volume[:,[7]] = np.array([[0], [-dy], [-dz]]) # bottom left back 

        return robot_grasp_volume


class GraspObject:
    def __init__(self,gop,god,rx,ry,rz): 
        self.gop = gop # grasp object position
        self.god = god # grasp object dimension
        self.rx = rx # rotations
        self.ry = ry
        self.rz = rz
        self.normal_axis = np.array([[1],[0],[0]]) # normal axis of the object is always in the +x axis of the object frame 

    def rotation_matrix(self):
      rotation_x = np.array([[1,0,0],[0, math.cos(self.rx), -1*math.sin(self.rx)],[0,math.sin(self.rx),math.cos(self.rx)]])
      rotation_y = np.array([[math.cos(self.ry), 0 , math.sin(self.ry)], [0 , 1, 0], [-1*math.sin(self.ry), 0 , math.cos(self.ry)]])
      rotation_z = np.array([[math.cos(self.rz), -1*math.sin(self.rz), 0], [math.sin(self.rz), math.cos(self.rz), 0], [0, 0, 1]])
      ROT = rotation_x@rotation_y@rotation_z
      return ROT

    def grasp_volume_in_local_frame(self):
        # the volume around the grasp object in its local frame
        # forward is in its +x axis 
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
        # the volume around the grasp object in the global frame
        grasp_volume_in_local_frame = self.grasp_volume_in_local_frame()
        ROT = self.rotation_matrix()
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
  def __init__(self,N,wire_model, grasp_object,robot, curve):
    self.N = N # number of nodes that represents the wire
    self.wire_model = wire_model
    self.m = self.wire_model.M/self.N # individual node mass
    self.l = self.wire_model.L/(self.N - 1) # resting length between nodes
    self.actions = 8
    self.grasp_object = grasp_object
    self.robot = robot
    self.curve = curve
    

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

  def internal_forces(self,wire,V):
      F_s = np.zeros((3,self.N)) # structural spring force vectors
      F_b = np.zeros((3,self.N)) # structural spring force vectors

      len_arr = self.l*np.ones((1,self.N-4))

      ## Structure Springs
      l1 = la.norm(wire[:,3:self.N-1] - wire[:,2:self.N-2],axis = 0)
      l2 = la.norm(wire[:,2:self.N-2] - wire[:,1:self.N-3],axis = 0)

      F1 = self.wire_model.Ks*((len_arr/l2) - 1)*(wire[:,2:self.N-2] - wire[:,1:self.N-3])
      F2 = -self.wire_model.Ks*((len_arr/l1) - 1)*(wire[:,3:self.N-1] - wire[:,2:self.N-2])
      F_s[:,2:self.N-2] = F1 + F2

      # case when i = 1
      l_prime_1 = la.norm(wire[:,[2]] - wire[:,[1]]) # new distance from node i to node (i+1)
      l_prime_2 = la.norm(wire[:,[1]] - wire[:,[0]]) # new distance from node i to node (i-1)
      F1 = 2*self.wire_model.Ks*((self.l/l_prime_2) - 1)*(wire[:,[1]] - wire[:,[0]])
      F2 = -self.wire_model.Ks*((self.l/l_prime_1) - 1)*(wire[:,[2]] - wire[:,[1]])
      F_s[:,[1]] = F1 + F2

      # case when i = N-2
      l_prime_1 = la.norm(wire[:,[self.N-1]] - wire[:,[self.N-2]]) # new distance from node i to node (i+1)
      l_prime_2 = la.norm(wire[:,[self.N-2]] - wire[:,[self.N-3]]) # new distance from node i to node (i-1)
      F1 = self.wire_model.Ks*((self.l/l_prime_2) - 1)*(wire[:,[self.N-2]] - wire[:,[self.N-3]])
      F2 = -2*self.wire_model.Ks*((self.l/l_prime_1) - 1)*(wire[:,[self.N-1]] - wire[:,[self.N-2]])
      F_s[:,[self.N-2]] = F1 + F2

      # damping force 
      # i = 1 - N-2
      """
      x1 = wire[:,1:self.N-1] - wire[:,0:self.N-2]
      F1_d_ = -self.wire_model.Kd*np.matmul(x1@np.transpose(x1), V[:,1:self.N-1] - V[:,0:self.N-2])/(la.norm(x1,axis=0)**2)

      x2 = wire[:,2:self.N] - wire[:,1:self.N-1]
      F2_d_ = self.wire_model.Kd*np.matmul(x2@np.transpose(x2), V[:,2:self.N] - V[:,1:self.N-1])/(la.norm(x2,axis=0)**2)

      F_s[:,1:self.N-1] = F_s[:,1:self.N-1] + F2_d_ + F1_d_
      """

      """

      for i in range(1,self.N-1):
        # INCLUDE DAMPING FORCE
        x1 = wire[:,[i]] - wire[:,[i-1]]
        F1_d = -self.wire_model.Kd*np.matmul(x1*np.transpose(x1), V[:,[i]] - V[:,[i-1]])/(la.norm(x1)**2)

        x2 = wire[:,[i+1]] - wire[:,[i]]
        F2_d = self.wire_model.Kd*np.matmul(x2*np.transpose(x2), V[:,[i+1]] - V[:,[i]])/(la.norm(x2)**2)

        F_s[:,[i]] = F_s[:,[i]] + F2_d + F1_d

      """

      ## Bending Spring 
      # 3 - N-3
      len_arr = self.l*np.ones((1,self.N-6))

      l1 = la.norm(wire[:,5:self.N-1] - wire[:,3:self.N-3],axis = 0)
      l2 = la.norm(wire[:,3:self.N-3] - wire[:,1:self.N-5],axis = 0)

      F2 = self.wire_model.Kb*((len_arr/l2) - 1)*(wire[:,3:self.N-3] - wire[:,1:self.N-5])
      F1 = -self.wire_model.Kb*((len_arr/l1) - 1)*(wire[:,5:self.N-1] - wire[:,3:self.N-3])
      F_b[:,3:self.N-3] = F1 + F2

      # 1:2
      len_arr = self.l*np.ones((1,2))
      l11 = la.norm(wire[:,3:5] - wire[:,1:3],axis = 0)
      F_b[:,1:3] = -self.wire_model.Kb*((len_arr/l11) - 1)*(wire[:,3:5] - wire[:,1:3])

      # N-3 N-2
      l22 = la.norm(wire[:,self.N-3:self.N-1] - wire[:,self.N-5:self.N-3],axis = 0)
      F_b[:,self.N-3:self.N-1] = self.wire_model.Kb*((len_arr/l22) - 1)*(wire[:,self.N-3:self.N-1] - wire[:,self.N-5:self.N-3])

      F = F_s #+ F_b
      return F

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

      element = self.curve.find_point_on_curve_from_node(np.transpose(wire[:,[grasp_index]]))
      tangent = self.curve.get_tangent_vector(element)

      ref_vec = np.array([[tangent[0]], [tangent[1]], [tangent[2]]])

      #ref_vec = wire[:,[grasp_index + 1]] - wire[:,[grasp_index]]

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

  """def plot_norm_vecs(self,wire,grasp_index,norm_vecs):
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
  """


  def simulate(self,wire):
      wire_set = np.zeros((self.actions,3,self.N))
      dt = 0.01
      grasp_index = 10
      F_vect = self.get_normal_vectors(wire,grasp_index)
      
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

              #F_s = self.__force_for_struct_springs(wire_sim,v)
              #F_b = self.__force_for_bending_springs(wire_sim)
              #F_net = F_s #+ F_b

              # compute internal forces
              F_net = self.internal_forces(wire_sim,v)
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

      # get optimal wire config
      optimal_wire_config = self.find_optimal_action(wire_set)

      if (len(optimal_wire_config) >0):
        # find the optimal wire config in the original wire set
        for i in range(self.actions):
            if np.array_equal(optimal_wire_config,wire_set[[i],:,:]):
                optimal = i
        
        # find the direction vector that matches the optimal wire config
        optimal_pull_action = F_vect[:,[optimal]]
        optimal_pick_action = wire[:,[grasp_index]]

        if optimal_pick_action[1] >= self.grasp_object.gop[1]:
            robot_to_grasp_wire = "left"
        else:
            robot_to_grasp_wire ="right"

      else:
          print("STATUS: No Solution Found")
          optimal_pull_action = np.array([])
          optimal_pick_action = np.array([])
          robot_to_grasp_wire = None

      return optimal_pick_action, optimal_pull_action , robot_to_grasp_wire

  def dot_product(self,u,v):
      dp = sum([x*y for (x, *x2), y in zip(u,v)])
      return dp
  
  def rotation_matrix(self,rx,ry,rz):
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
 
  def find_optimal_action(self,wire_set):
      workspace_volume = self.robot.workspace_volume()
      object_volume_local = self.grasp_object.grasp_volume_in_local_frame()
      object_volume_global = self.grasp_object.grasp_volume_in_global_frame()
      point_of_reference = self.robot.mid_point_of_reference()
      ROT = self.grasp_object.rotation_matrix()

      # projections of points onto a plane that is normal to the x axis of the grasp object frame
      projections = np.zeros((self.actions,3,self.N))
      projections_in_gof = np.zeros((self.actions,3,self.N))
      Distance = np.zeros((1,self.actions))

      for j in range(self.actions):
        projections[[j],:,:] , projections_in_gof[[j],:,:] , Distance[0][j] = self.projections(wire_set[[j],:,:].reshape((3,20)))

      # First volume constraint is the object_volume 
      # Second volume constraint is the robot arm grasp volume 
      #     -this volume represents the space the arm will need to take up when grasping the object 
      #     -it does not represent the full volume, but more a subset a the volume closest to the grasp object 
      #     -Represented by a box hinged at a point +10cm in the grasp object x axis, and points towards the point_of_reference 
      # Third Volume constraint is the workspace volume

      # point of reference in the grasp object frame
      offset = object_volume_local[0][0] + 0.05 # the joint of the first and second volume constraints will be offset in +x of grasp object frame
      POR_in_grasp_object_frame = self.homo_to_vector(la.inv(self.homo_trans_matrix(self.grasp_object.gop,ROT))@self.vector_to_homo(point_of_reference))
      POR_in_grasp_object_frame[0][0] = POR_in_grasp_object_frame[0][0] - offset

      rz = math.atan2(POR_in_grasp_object_frame[1][0], POR_in_grasp_object_frame[0][0])
      ry = -math.atan2(POR_in_grasp_object_frame[2][0], POR_in_grasp_object_frame[0][0])
      rx = 0

      if (POR_in_grasp_object_frame[0][0] <0 and POR_in_grasp_object_frame[1][0] <0):
          rz = rz + math.pi
      if (POR_in_grasp_object_frame[0][0] <0 and POR_in_grasp_object_frame[1][0] >= 0):
          rz = rz + math.pi
      
      # translation and rotation of the robot_volume contraint frame WRT the grasp object frame
      ROT2 = self.rotation_matrix(rx,ry,rz)
      translation = np.array([[offset],[0],[0]])
      robot_grasp_volume = self.robot.robot_grasp_volume()

      rgv_global_frame = np.zeros((3,8))

      # robot grasp volume in global_frame
      for i in range(8):
        temp = self.homo_to_vector(self.homo_trans_matrix(translation,ROT2)@self.vector_to_homo(robot_grasp_volume[:,[i]]))
        rgv_global_frame[:,[i]] = self.grasp_object.gop + ROT@temp
        

      # transform the wires to the grasp object frame and the robot arm grasp volume frame

      wire_in_gof = np.zeros((self.actions,3,self.N)) # wires in grasp object frame
      wire_in_rgf = np.zeros((self.actions,3,self.N)) # wires in robot grasp volume frame


      for i in range(self.actions):
          for j in range(self.N):
               # This is a mess. break up for clarity
               # There is a lot of transposeing. wire_in_gof[[i],:,[j]] accepts a 1x3 aparently 
               wire_in_gof[[i],:,[j]] = np.transpose(self.homo_to_vector(la.inv(self.homo_trans_matrix(self.grasp_object.gop,ROT))@self.vector_to_homo(np.transpose(wire_set[[i],:,[j]]))))
               wire_in_rgf[[i],:,[j]] = np.transpose(self.homo_to_vector(la.inv(self.homo_trans_matrix(translation,ROT2))@self.vector_to_homo(np.transpose(wire_in_gof[[i],:,[j]]))))
               

      # object dimensions 
      dx = self.grasp_object.god[0]/2
      dy = self.grasp_object.god[1]/2
      dz = self.grasp_object.god[2]/2

      jj = 0
      conflict = 0
      count = 0

      print("  WIRE CONFIG SOLUTION")

      for i in range(self.actions):
          for j in range(3,self.N-4):
              if( -dx <= wire_in_gof[[jj],[0],[j]] and wire_in_gof[[jj],[0],[j]] <= dx
                and -dy <= wire_in_gof[[jj],[1],[j]] and wire_in_gof[[jj],[1],[j]] <= dy
                and -dz <= wire_in_gof[[jj],[2],[j]] and wire_in_gof[[jj],[2],[j]] <= dz):
                conflict = 1

              """if (robot_grasp_volume[0][4] <= wire_in_rgf[[jj],[0],[j]] and wire_in_rgf[[jj],[0],[j]] <= robot_grasp_volume[0][0]
                and robot_grasp_volume[1][1] <= wire_in_rgf[[jj],[1],[j]] and wire_in_rgf[[jj],[1],[j]] <= robot_grasp_volume[1][0]
                and robot_grasp_volume[2][2] <= wire_in_rgf[[jj],[2],[j]] and wire_in_rgf[[jj],[2],[j]] <= robot_grasp_volume[2][1]):
                conflict = 1"""

              if (workspace_volume[0][0] < wire_set[[jj],[0],[j]] or wire_set[[jj],[0],[j]] < workspace_volume[0][4]
                  or workspace_volume[1][1] < wire_set[[jj],[1],[j]] or wire_set[[jj],[1],[j]] < workspace_volume[1][0]
                  or workspace_volume[2][0] < wire_set[[jj],[2],[j]] or wire_set[[jj],[2],[j]] < workspace_volume[2][2]):
                  conflict = 1

          if(conflict ==1):
            print("     wire" + str(i) + " violated grasp")
            wire_in_gof = np.delete(wire_in_gof,jj,0) 
            wire_in_rgf = np.delete(wire_in_rgf,jj,0) 
            wire_set = np.delete(wire_set,jj,0) 
            Distance = np.delete(Distance,jj,1)
            conflict = 0    
            count = count + 1
          else:
            jj = jj +1 

      # find the wire config with max distance from grasp object
      print("")
      print("      " + str(8 - count) + " Possible Wire Configurations Found")
      if count < 7:
        max_dist = 0
        for j in range(len(Distance[0])):
          if Distance[0][j] >= max_dist:
            max_dist = Distance[0][j]
            max_dist_element = j
        print("     Best Solution Found")
        optimal_wire_config = wire_set[[max_dist_element],:,:]

      elif count == 7:
        print("     Only one valid Solution")
        optimal_wire_config = wire_set
      else:
        print("     No solution")
        optimal_wire_config = np.array([])     
              
      # need to employ one more constraint -> projections and max distance 
      # this will converge on a single wire config 
      # just return the first one for now
      return optimal_wire_config


  def projections(self,wire_set):
      # Exlude for now
      projection_in_global = np.zeros((3,self.N)) # projections of points on grasp object normal plane in global frame
      projection_in_gof = np.zeros((3,self.N)) # projections of points on grasp object normal plane in grasp object frame
      ROT = self.grasp_object.rotation_matrix()
      grasp_axis = ROT@self.grasp_object.normal_axis

      Distance = 0
      
      for i in range(self.N):
        projection_in_global[:,[i]] = wire_set[:,[i]] - np.dot(np.transpose(wire_set[:,[i]] - self.grasp_object.gop), grasp_axis)*grasp_axis
        projection_in_gof[:,[i]] = self.homo_to_vector(la.inv(self.homo_trans_matrix(self.grasp_object.gop,ROT))@self.vector_to_homo(projection_in_global[:,[i]]))

        if i != 0 or i != 1 or i != 2 or i !=(self.N-3) or i !=(self.N -2) or i !=(self.N -1):
          Distance = Distance + (projection_in_gof[0][i]**2 + projection_in_gof[1][i]**2)**0.5

      Distance = Distance/(self.N-6)
    
      return projection_in_global , projection_in_gof , Distance


