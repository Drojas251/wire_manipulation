#!/usr/bin/env python3

import numpy as np
import math
from numpy import linalg as la
from scipy.spatial.transform import Rotation as R

class WireGraspToolbox():
  def __init__(self):
    self = self

  # Bezier Curve Tools
  def Bernstein(self,n,j,t):
    B = math.factorial(n)/(math.factorial(j)*math.factorial(n-j))*(t**j)*(1-t)**(n-j)
    return B

  def BCurve(self,t,P):
    # input should be a N x 3 array where N = # of points 
    # output is a 100 x 3 array of points 
    n = len(P)
    curve = np.zeros((len(t),3))
    for k in range(len(t)):
      curve[[k],:] = np.zeros((1,3))
      for j in range(n):
        curve[[k],:] = curve[[k],:] + P[[j],:]*self.Bernstein(n-1,j,t[k])
    return curve

  def BCurve_first_derivative(self,t,P):

    # input should be a N x 3 array where N = # of points 
    # output is a 100 x 3 array of points
    n = len(P)
    Q = np.zeros((len(t),3))
    for k in range(len(t)):
      Q[[k],:] = np.zeros((1,3))
      for j in range(n-1):
        Q[[k],:] = Q[[k],:] + (n-1)*(P[[j+1],:] - P[[j],:])*self.Bernstein(n-2,j,t[k])
    return Q

  def BCurve_second_derivative(self,t,P):

    # input should be a N x 3 array where N = # of points 
    # output is a 100 x 3 array of points
    n = len(P)
    Q = np.zeros((len(t),3))
    for k in range(len(t)):
      Q[[k],:] = np.zeros((1,3))
      for j in range(n-2):
        Q[[k],:] = Q[[k],:] + (n-1)*(n-2)*(P[[j+2],:] - 2*P[[j+1],:] + P[[j],:])*self.Bernstein(n-3,j,t[k])
    return Q

  def TNB_frame(self,t,P,element):
    Q = self.BCurve(t,P) # bezier curve
    Q_dot = self.BCurve_first_derivative(t,P) # first derivative of B curve
    Q_dot_dot = self.BCurve_second_derivative(t,P) # second derivative of B curve

    T = Q_dot[element]/(la.norm(Q_dot[element]))
    B = np.cross(Q_dot[element],Q_dot_dot[element])/(la.norm(np.cross(Q_dot[element],Q_dot_dot[element])))
    N = np.cross(B,T)

    Transform = np.matrix([N,B,T])
    return Transform

  # Rotation Matrices and Euler Angle tools
  def isRotationMatrix(self,R) :
      Rt = np.transpose(R)
      shouldBeIdentity = np.dot(Rt, R)
      I = np.identity(3, dtype = R.dtype)
      n = np.linalg.norm(I - shouldBeIdentity)

      return n < 1e-6
 

  def rotationMatrixToEulerAngles(self,R) :
      assert(self.isRotationMatrix(R))
      sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
      singular = sy < 1e-6

      if  not singular :
          x = math.atan2(R[2,1] , R[2,2])
          y = math.atan2(-R[2,0], sy)
          z = math.atan2(R[1,0], R[0,0])

      else :
          x = math.atan2(-R[1,2], R[1,1])
          y = math.atan2(-R[2,0], sy)
          z = 0

      return np.array([x, y, z])

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
  def grasp_correction(self,Frame, grasp_vector, TOL = 0.02 ,flip=False):
    # grasp_vec = 1 x 3
    correction = 0.0174533 # 1 deg
    status = 1
    iter = 0

    while status == 1 and iter < 500:

      gripper_x_axis_pointing = np.dot(Frame[[0],:], np.array([1,0,0]))
      pull_vec_alignment = np.dot(Frame[[1],:],grasp_vector.flatten())

      if (gripper_x_axis_pointing >= 0 and (1-np.abs(pull_vec_alignment))< TOL):
        status = 0
      else:
        rotm = self.ROT(0,0,correction)
        Frame = rotm@Frame
      iter = iter + 1

    print("pull vec ", pull_vec_alignment)
    print("F ", Frame[[1],:] )
    print("grasp ", grasp_vector)
    print("")
    print("gripper_x_axis_pointing ", gripper_x_axis_pointing)
    print("F ", Frame[[0],:])
    #print("orientation Reached in Step: ", iter)
    #print("Dot product between gripper x axis and global x axis: ", gripper_x_axis_pointing)
    #print("Dot product between gripper y axis and grasp vector: ",pull_vec_alignment)
    return Frame

  # Grasp Orientaion Function
  def get_wire_grasp_orientation(self,t,P,grasp_point,grasp_vector):
    # get the wire grasp orientation at a grasp point on the wire 
    # t = 100 x 1
    # P = N x 3
    # grasp_point = 1 x 3
    # grasp_vec = 1 x 3
    curve = self.BCurve(t,P)
    element = 25
    TOL = 0.08

    for i in range(len(t)):
      if np.array_equal(curve[[i],:],grasp_point):
        element = i
        break
    if element == 25:
      for i in range(len(t)):
        diff = grasp_point - curve[[i],:]
        diff = diff.flatten()
        if np.abs(float(diff[0])) < TOL and np.abs(float(diff[1])) < TOL and np.abs(float(diff[2])) < TOL:
          element = i
          break

    Frame = self.TNB_frame(t,P,element)
    #grasp_rotm = self.grasp_correction(Frame,grasp_vector)
    grasp_rotm = Frame

    r = R.from_matrix(grasp_rotm)
    print("Rot", grasp_rotm)
    grasp_quat = r.as_quat() # x y z w
    print("quat", grasp_quat)


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