#!/usr/bin/env python3

import numpy as np
import math
from numpy import linalg as la
from scipy.spatial.transform import Rotation as R

class WireGraspToolbox():
  def __init__(self,P):
    self = self
    self.P = P
    self.t = np.arange(0, 1, 0.01)

  # Bezier Curve Tools
  def Bernstein(self,n,j,t):
    B = math.factorial(n)/(math.factorial(j)*math.factorial(n-j))*(t**j)*(1-t)**(n-j)
    return B

  def BCurve(self):
    # input should be a N x 3 array where N = # of points 
    # output is a 100 x 3 array of points 
    n = len(self.P)
    curve = np.zeros((len(self.t),3))
    for k in range(len(self.t)):
      curve[[k],:] = np.zeros((1,3))
      for j in range(n):
        curve[[k],:] = curve[[k],:] + self.P[[j],:]*self.Bernstein(n-1,j,self.t[k])
    return curve

  def BCurve_first_derivative(self):

    # input should be a N x 3 array where N = # of points 
    # output is a 100 x 3 array of points
    n = len(self.P)
    Q = np.zeros((len(self.t),3))
    for k in range(len(self.t)):
      Q[[k],:] = np.zeros((1,3))
      for j in range(n-1):
        Q[[k],:] = Q[[k],:] + (n-1)*(self.P[[j+1],:] - self.P[[j],:])*self.Bernstein(n-2,j,self.t[k])
    return Q

  def BCurve_second_derivative(self):

    # input should be a N x 3 array where N = # of points 
    # output is a 100 x 3 array of points
    n = len(self.P)
    Q = np.zeros((len(self.t),3))
    for k in range(len(self.t)):
      Q[[k],:] = np.zeros((1,3))
      for j in range(n-2):
        Q[[k],:] = Q[[k],:] + (n-1)*(n-2)*(self.P[[j+2],:] - 2*self.P[[j+1],:] + self.P[[j],:])*self.Bernstein(n-3,j,self.t[k])
    return Q

  def get_tangent_vector(self,element):
    Q_dot = self.BCurve_first_derivative() # first derivative of B curve

    T = Q_dot[element]/(la.norm(Q_dot[element]))
    return T

  def find_point_on_curve_from_node(self,grasp_point):
    # grasp point is one of 20 points on a defined wire:  1 x 3
    curve = self.BCurve()
    element = 25
    TOL = 0.08

    for i in range(len(self.t)):
      if np.array_equal(curve[[i],:],grasp_point):
        element = i
        break
    if element == 25:
      for i in range(len(self.t)):
        diff = grasp_point - curve[[i],:]
        diff = diff.flatten()
        if np.abs(float(diff[0])) < TOL and np.abs(float(diff[1])) < TOL and np.abs(float(diff[2])) < TOL:
          element = i
          break
    return element


  def TNB_frame(self,element):
    Q_dot = self.BCurve_first_derivative() # first derivative of B curve
    Q_dot_dot = self.BCurve_second_derivative() # second derivative of B curve

    T = Q_dot[element]/(la.norm(Q_dot[element]))
    B = np.cross(Q_dot[element],Q_dot_dot[element])/(la.norm(np.cross(Q_dot[element],Q_dot_dot[element])))
    N = np.cross(B,T)

    Transform = np.matrix([N,B,T]) # The TNB frame expressed in the world

    return Transform


  def ROT(self,rx,ry,rz):
    rotation_x = np.matrix([[1, 0, 0],
                [0,math.cos(rx), -1*math.sin(rx)],
                [0,math.sin(rx),math.cos(rx)]])
    
    rotation_y = np.matrix([[math.cos(ry),0,math.sin(ry)],
                            [0,1,0],
                            [-1*math.sin(ry),0,math.cos(ry)]])
    rotation_z = np.matrix([[math.cos(rz),-1*math.sin(rz), 0],
                            [math.sin(rz), math.cos(rz), 0],
                            [0,0,1]])
    rotm = rotation_x@rotation_y@rotation_z
    return rotm

  # Grasp Correction function
  def grasp_correction(self,Frame, grasp_vector, TOL = 0.01 ,flip=False):
    # grasp_vec = 1 x 3

    # Goal of Function: Want the pull vector and y axis of the gripper frame to be parallel 

    correction = 0.00872665 # 0.5 deg
    status = 1
    iter = 0
    min_value = 1
    MAX_ITER = 1000


    while status == 1 and iter < MAX_ITER:

      gripper_x_axis_pointing = np.dot(Frame[[0],:], np.array([1,0,0])) # dot product of the x axis in the world and TNB frame
      pull_vec_alignment = np.dot(Frame[[1],:],grasp_vector.flatten()) # dot product of the y axis of the TNB frame and pull vector
      compare = 1-np.abs(pull_vec_alignment) 

      # Finds the Next best solution if the Tolerance is not met in 1000 iterations 
      if gripper_x_axis_pointing >= 0:
        if compare < min_value:
          Best_frame = Frame
          min_value = compare

      # If tolerance is met, breake the loop, otherwise rotate about z axis 
      if (gripper_x_axis_pointing >= 0 and compare< TOL):
        status = 0
      else:
        rotm = self.ROT(0,0,correction)
        Frame = rotm@Frame
      iter = iter + 1

      
    # If Tolerance not met in 1000 iterations, find Best frame available 
    if iter >= MAX_ITER:
      Frame = Best_frame
      print("Reached Max Iteration. Going with Best Available Gripper Orientation Found. May not be Optimal")
      print("Min_value ", min_value)

    # Transpose to get correct quaternion. This is required 
    Frame = Frame.getT()

    return Frame

  # Grasp Orientaion Function
  def get_wire_grasp_orientation(self,grasp_point,grasp_vector):
    # get the wire grasp orientation at a grasp point on the wire 
    # t = 100 x 1
    # P = N x 3
    # grasp_point = 1 x 3
    # grasp_vec = 1 x 3
    curve = self.BCurve()
    element = 25
    TOL = 0.08

    for i in range(len(self.t)):
      if np.array_equal(curve[[i],:],grasp_point):
        element = i
        break
    if element == 25:
      for i in range(len(self.t)):
        diff = grasp_point - curve[[i],:]
        diff = diff.flatten()
        if np.abs(float(diff[0])) < TOL and np.abs(float(diff[1])) < TOL and np.abs(float(diff[2])) < TOL:
          element = i
          break

    Frame = self.TNB_frame(element)
    grasp_rotm = self.grasp_correction(Frame,grasp_vector)


    r = R.from_matrix(grasp_rotm)
    grasp_quat = r.as_quat() # x y z w

    return grasp_rotm , grasp_quat


def rotm(rx,ry,rz):
  rotation_x = np.matrix([[1, 0, 0],
                [0,math.cos(rx), -1*math.sin(rx)],
                [0,math.sin(rx),math.cos(rx)]])
    
  rotation_y = np.matrix([[math.cos(ry),0,math.sin(ry)],
                          [0,1,0],
                          [-1*math.sin(ry),0,math.cos(ry)]])
  rotation_z = np.matrix([[math.cos(rz),-1*math.sin(rz), 0],
                          [math.sin(rz), math.cos(rz), 0],
                          [0,0,1]])

  rotm = rotation_x@rotation_y@rotation_z

  return rotm