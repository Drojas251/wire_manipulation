#!/usr/bin/env python3

import numpy as np
from numpy import linalg as la
from numpy.lib.function_base import vectorize






class WireSim:
  def __init__(self,N,Ks,Kb,Kd,L,M):
    self.N = N # number of nodes that represents the wire
    self.Ks = Ks # Stifness of structural spring
    self.Kb = Kb # stiffness of bending spring
    self.Kd = Kd # damping 
    self.L = L #length of wire
    self.M = M # mass of wire
    self.m = self.M/self.N # individual node mass
    self.l = self.L/(self.N - 1) # resting length between nodes
    

  def force_for_struct_springs(self,wire):

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
                F1 = 2*self.Ks*((self.l/l_prime_2) - 1)*(wire[:,[i]] - wire[:,[i-1]])
                F2 = -self.Ks*((self.l/l_prime_1) - 1)*(wire[:,[i+1]] - wire[:,[i]])

            elif i == (self.N - 2):
                F1 = self.Ks*((self.l/l_prime_2) - 1)*(wire[:,[i]] - wire[:,[i-1]])
                F2 = -2*self.Ks*((self.l/l_prime_1) - 1)*(wire[:,[i+1]] - wire[:,[i]])

            else:
                F1 = self.Ks*((self.l/l_prime_2) - 1)*(wire[:,[i]] - wire[:,[i-1]])
                F2 = -self.Ks*((self.l/l_prime_1) - 1)*(wire[:,[i+1]] - wire[:,[i]])


            # INCLUDE DAMPING FORCE

            F_s[:,[i]] = F1 + F2 # + F_damping

    return F_s

  def force_for_bending_springs(self,wire):    

      F_b = np.zeros((3,self.N)) # Bending spring force vectors

      for i in range(1,self.N - 1):
          if i == 1 or i == 2:
              l_prime = la.norm(wire[:,[i+2]] - wire[:,[i]])
              F_b[:,[i]] = -self.Kb*((self.l/l_prime) - 1)*wire[:,[i+2]] - wire[:,[i]]

          elif i == (self.N - 1) or i == (self.N -2):
              l_prime = la.norm(wire[:,[i]] - wire[:,[i-2]])
              F_b[:,[i]] = self.Kb*((self.l/l_prime) - 1)*wire[:,[i]] - wire[:,[i-2]]
          else:
              l_prime_1 = la.norm(wire[:,[i+2]] - wire[:,[i]]) # new distance from node i to node (i+2)
              l_prime_2 = la.norm(wire[:,[i]] - wire[:,[i-2]]) # new distance from node i to node (i-2)

              F1 = -self.Kb*((self.l/l_prime_1) - 1)*wire[:,[i+2]] - wire[:,[i]]
              F2 = self.Kb*((self.l/l_prime_2) - 1)*wire[:,[i]] - wire[:,[i-2]]

              F_b[:,[i]] = F1 + F2

      return F_b

  def normal_vec(self,ref_vec):
      if ref_vec[2] != 0:
          situation = 3
      elif ref_vec[1] != 0:
          situation = 2
      elif ref_vec[0] != 0:
          situation = 1
      
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

      grasp_index = grasp_index - 1 # grasp index is interms of the node number, but we reduce by one to track in python array index
      vectors = np.zeros((3,8))

      ref_vec = wire[:,[grasp_index + 1]] - wire[:,[grasp_index]]
      norm_vec = self.normal_vec(ref_vec)


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
      wire_sim = wire
      v = np.zeros((3,self.N))
      a_k = np.zeros((3,self.N))
      v_k = np.zeros((3,self.N))

      TOL = 0.1
      stop = 0
      count = 0
      MAX_iterations = 700
      dt = 0.01
      grasp_index = 10
      F_vect = self.get_normal_vectors(wire_sim,grasp_index)

      while stop == 0:

          

          F_s = self.force_for_struct_springs(wire_sim)
          F_b = self.force_for_bending_springs(wire_sim)
          F_net = F_s + F_b
          F_net[2,1:(self.N-1)] = F_net[2,1:(self.N-1)] - self.m*9.81

          F_applied = 1.0*F_vect[:,[2]]
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

      return wire_sim
          

  






N = 20
Ks = 13.1579
Kb = 6.13
Kd = 2
L = 1
M = 0.028
p1 = WireSim(N,Ks,Kb,Kd,L,M )

wires = np.zeros((2,3,N)) # equivalent to tensor of size (3,N,2) in matlab
wires[1][2][4] = 23


test = np.zeros((3,N))
test[1][0] = 5

test[0] = np.ones((1,N))
test[1] = np.linspace(0,1,N)
test[2] = 2*np.ones((1,N))
#p1.force_for_struct_springs(test)
p1.force_for_bending_springs(test)

norm_vec = np.array([[1],[2],[3]])
vec = p1.normal_vec(norm_vec)

#normal_vectors = p1.get_normal_vectors(test,10)
sim_result = p1.simulate(test)
print(sim_result)
