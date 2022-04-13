#!/usr/bin/env python3

#Python
import numpy as np
from numpy import linalg as la
from numpy.lib.function_base import vectorize
import math
from colorama import Fore

#Other
from wire_modeling.wire_grasp_toolbox import WireGraspToolbox
import fcl

# FLOW 
# Get PC size ~ 
# Pass PC to State estimation to output 3x20 vector representing wire 
# Pass 3x20 vector to sim function. Output is 8x3x20 tensor representing results to 8 actions 
# Pass 8x3x20 to Optimizer function to select an solution and return the Pick and Pull pair that goes with that solution 
# Return Pick and Pull pair 

class Collisions:
    def __init__(self):
        self = self
        self.env_collision_object = [] # list of collision objs in env
        
    def make_box(self,x,y,z,position):
        # This function makes a box primitve collision object
        box = fcl.Box(x,y,z)
        transform = fcl.Transform(position)
        collision_obj = fcl.CollisionObject(box, transform)
        return collision_obj

    def make_sphere(self,radius, position):
        # This function makes a sphere primitive collision object
        node = fcl.Sphere(radius)
        node_tf = fcl.Transform(position)
        node_obj = fcl.CollisionObject(node, node_tf)
        return node_obj

    def make_box_env_collision_obj(self,x,y,z,position):
        # This function makes a env collision object and appends it to a list
        collision_obj = self.make_box(x,y,z,position)
        self.env_collision_object.append(collision_obj)

    def make_sphere_env_collision_obj(self,radius, position):
        # This function makes a env collision object and appends it to a list
        collision_obj = self.make_sphere(radius, position)
        self.env_collision_object.append(collision_obj)

    def make_wire_collision_object(self,wire):
        # This function makes a collision object for a wire
        radius = 0.01
        wire_collision_object = []
        for i in range(2,17):
            x = wire[0][i]
            y = wire[1][i]
            z = wire[2][i]
            position = np.array([x, y, z])
            obj = self.make_sphere(radius,position)
            wire_collision_object.append(obj)
        return wire_collision_object

    def make_collision_manager(self,obj):
        # This function makes a collision manager for a group of collision objects 
        manager = fcl.DynamicAABBTreeCollisionManager()
        manager.registerObjects(obj)
        manager.setup()
        return manager

    def make_env_collision_manager(self):
        # This function makes a collision manager for the group of env collison objects
        manager = self.make_collision_manager(self.env_collision_object)
        return manager
    
    


class TargetObject:
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


class WireModel:
    def __init__(self,L,M,Ks,Kb,Kd,wire_class): 
        self.L = L #length of wire
        self.M = M # mass of wire
        self.Ks = Ks # Stifness of structural spring
        self.Kb = Kb # stiffness of bending spring
        self.Kd = Kd # damping
        self.wire_class = wire_class # type1 = vertical wire , type2 = horizontal wire


class WireSim:
  def __init__(self,N,wire_model, grasp_object, curve, collisions):
    self.N = N # number of nodes that represents the wire
    self.wire_model = wire_model # wire model with shape 3 x N
    self.grasp_object = grasp_object # grasp object
    self.curve = curve # bezier curve
    self.collisions = collisions # collision object
    self.m = self.wire_model.M/self.N # individual node mass
    self.l = self.wire_model.L/(self.N - 1) # resting length between nodes
    self.actions = 8 # number of pull vectors to solve for 
    self.time_step = 0.001 # time step of the simulation 
    self.max_iterations = 500 # max iterations in a simulation before timeout in enforced 
    

  def internal_forces(self,wire,V):
      # This function computes the internal forces due to bending and structural springs 

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
      # This function computes a normal vector given a tangent vector 

      # check for any zeros to avoid dividing by zero
      if ref_vec[2] != 0:
          situation = 3
      elif ref_vec[1] != 0:
          situation = 2
      elif ref_vec[0] != 0:
          situation = 1

      # assigns values to two unkowns and solves the third to get normal vector 
      # The equation being solved is the dot product = 0
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
      
      # Get Tangent Vector
      element = self.curve.find_point_on_curve_from_node(np.transpose(wire[:,[grasp_index]]))
      tangent = self.curve.get_tangent_vector(element)
      ref_vec = np.array([[tangent[0]], [tangent[1]], [tangent[2]]])

      # Get normal Vector from tangent vector 
      norm_vec = self.__normal_vec(ref_vec)

      # Find the rest of the vectors 
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


  def simulate(self,wire):
      # This function simulates different actions imposed on the wire and returns the best pair of actions

      wire_set = np.zeros((self.actions,3,self.N))
      dt = self.time_step
      grasp_index = self.get_grasp_index(wire)
      normal_vecs = self.get_normal_vectors(wire,grasp_index)
      
      # Simulate to get 8 wire configs
      for i in range(self.actions):
          v = np.zeros((3,self.N))
          a_k = np.zeros((3,self.N))
          v_k = np.zeros((3,self.N))

          TOL = 0.1
          stop = 0
          count = 0
          wire_sim = wire

          while stop == 0:   

              # compute internal forces
              F_net = self.internal_forces(wire_sim,v)
              F_net[2,1:(self.N-1)] = F_net[2,1:(self.N-1)] - self.m*9.81

              F_applied = 0.5*normal_vecs[:,[i]]
              F_net[:,[grasp_index]] = F_net[:,[grasp_index]] + F_applied

              #Simulate
              a_k_1 = F_net/self.m
              v_k_1 = v_k + 0.5*(dt**2)*(a_k_1 + a_k)
              wire_sim = wire_sim + dt*v_k + 0.25*(dt**2)*(a_k_1 + a_k)

              a_k = a_k_1
              v_k = v_k_1

              wire_sim[:,[0]] = wire[:,[0]]
              wire_sim[:,[self.N-1]] = wire[:,[self.N -1]]

              if count == self.max_iterations:
                  stop = 1
              count = count + 1
          wire_set[[i],:,:] = wire_sim

      # *FIND THE BEST SET OF ACTIONS*

      # Find the wire config that best meets the imposed constraints
      optimal_wire_config = self.find_optimal_action(wire_set,normal_vecs)

      if (len(optimal_wire_config) >0):
        # find the optimal wire config in the original wire set
        for i in range(self.actions):
            if np.array_equal(optimal_wire_config,wire_set[[i],:,:]):
                optimal = i
        
        # find pick and pull actions for the returned wire config
        optimal_pull_action = normal_vecs[:,[optimal]]
        optimal_pick_action = wire[:,[grasp_index]]

        # Predict the end effector position at the end of the trajectory to move the wire
        future_ee_point = np.array([optimal_pick_action[0] + optimal_pull_action[0],
                                    optimal_pick_action[1] + optimal_pull_action[1],
                                    optimal_pick_action[2] + optimal_pull_action[2]])

        # Determine which robot should grasp the wire such that the arms do not cross
        if future_ee_point[1] >= self.grasp_object.gop[1]:
            robot_to_grasp_wire = "left"
        else:
            robot_to_grasp_wire ="right"

      else:
          # No solution 
          print(Fore.GREEN + "STATUS:= " + Fore.WHITE + "No Solution Found \n")
          optimal_pull_action = np.array([])
          optimal_pick_action = np.array([])
          robot_to_grasp_wire = None

      return optimal_pick_action, optimal_pull_action , robot_to_grasp_wire

 
  def find_optimal_action(self,wire_set,normal_vecs):
      # This function returns a wire config that meets the imposed contraints

      # Get Distance Score for wire configs projected onto the target object plane
      Distance = np.zeros((1,self.actions))
      for j in range(self.actions):
        Distance[0][j]  = self.projections(wire_set[[j],:,:].reshape((3,20)))

      # *** Check Collisions *** #
      request = fcl.CollisionRequest()
      rdata = fcl.CollisionData(request = request)

      wire_collision_object = [] # Get Collision Objects for wire configs
      for i in range(self.actions):
            collision_object = self.collisions.make_wire_collision_object(wire_set[[i],:,:].reshape((3,20)))
            wire_collision_object.append(collision_object)

      env_collision_manager = self.collisions.make_env_collision_manager() #define collision manager for env

      jj = 0
      count = 0

      # check For Collisions between wire configs and env
      for wire_obj in wire_collision_object:

          wire_collision_manager = self.collisions.make_collision_manager(wire_obj) # collision manager for ith wire config

          rdata = fcl.CollisionData(request = request)
          env_collision_manager.collide(wire_collision_manager, rdata, fcl.defaultCollisionCallback) # check collision

          # reject wire configs that are in collisions with env
          if(rdata.result.is_collision):
            wire_set = np.delete(wire_set,jj,0) 
            Distance = np.delete(Distance,jj,1)
            normal_vecs = np.delete(normal_vecs,jj,1) 
            count = count + 1
                    
          else:
            jj = jj +1


      # find the wire config with max distance from grasp object
      print(Fore.GREEN + "     " + str(8 - count) + " Possible Wire Configurations Found")

      if count < 7:
          max_dist = 0
          for j in range(len(Distance[0])):
              if Distance[0][j] >= max_dist:
                  max_dist = Distance[0][j]
                  max_dist_element = j
          print(Fore.GREEN + "     Best Solution Found Based on Planar Distance \n")
          optimal_wire_config = wire_set[[max_dist_element],:,:]

      elif count == 7:
          print(Fore.GREEN + "     Only one valid Solution \n")
          optimal_wire_config = wire_set
      else:
          print(Fore.YELLOW + "     No solution \n")
          optimal_wire_config = np.array([])     
              
      return optimal_wire_config


  def projections(self,wire_set):
      # This function projects the wire config points onto the object plane and evaluates

      projection_in_global = np.zeros((3,self.N)) # projections of points on grasp object normal plane in global frame
      projection_in_gof = np.zeros((3,self.N)) # projections of points on grasp object normal plane in grasp object frame

      ROT = self.grasp_object.rotation_matrix()
      grasp_axis = ROT@self.grasp_object.normal_axis

      plane_distance = 0
      
      for i in range(self.N):
        projection_in_global[:,[i]] = wire_set[:,[i]] - np.dot(np.transpose(wire_set[:,[i]] - self.grasp_object.gop), grasp_axis)*grasp_axis
        projection_in_gof[:,[i]] = self.homo_to_vector(la.inv(self.homo_trans_matrix(self.grasp_object.gop,ROT))@self.vector_to_homo(projection_in_global[:,[i]]))

        # Exludes the first and last three points
        if i != 0 or i != 1 or i != 2 or i !=(self.N-3) or i !=(self.N -2) or i !=(self.N -1):
          plane_distance = plane_distance + (projection_in_gof[2][i]**2 + projection_in_gof[1][i]**2)**0.5

      plane_distance = plane_distance/(self.N-6) # returns average of all distances from points to center of target object
    
      return plane_distance

  def get_grasp_index(self,wire):
      # Based on the classified wire type from the state estimation block, a grasp point will selected 
      #     type1 = vertical 
      #     type2 = horizontal 

      # wire is mostly verticle 
      if self.wire_model.wire_class == "type1":
          # find the point that is closest to the grasps object z coordinate
          closest_value = np.abs(self.grasp_object.gop[2] - wire[[2],[0]])

          for i in range(self.N):
              diff_z = np.abs(self.grasp_object.gop[2] - wire[[2],[i]])
              if diff_z <= closest_value:
                  matching_element = i
                  closest_value = diff_z

      # wire is mostly horizontal
      elif self.wire_model.wire_class == "type2":
          # find the point that is closest to the grasps object y coordinate
          closest_value = np.abs(self.grasp_object.gop[1] - wire[[1],[0]])

          for i in range(self.N):
              diff_z = np.abs(self.grasp_object.gop[1] - wire[[1],[i]])
              if diff_z <= closest_value:
                  matching_element = i
                  closest_value = diff_z
      
      # want the node next to the node that is closest. 
      matching_element = matching_element + 1
      return matching_element

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



