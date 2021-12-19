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

    Transform = np.matrix([B,T,N])
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
  def grasp_correction(self,Frame,threshold):
    E = self.rotationMatrixToEulerAngles(Frame)
    print(E)

    if(E[1]<threshold and E[1]> -1*threshold):
      print("Good Orientation")
      rotm = self.ROT(0,0,0)
    elif(E[1] > threshold):
      correction = -1*E[1]
      print("correction= ",correction)
      rotm = self.ROT(0,correction,0)
    elif(E[1]< -1*threshold):
      correction = -1*E[1]
      print("correction= ",correction)
      rotm = self.ROT(0,correction,0)

    new_frame = rotm@Frame
    return new_frame

  # Grasp Orientaion Function
  def get_wire_grasp_orientation(self,t,P,grasp_point,threshold):
    # get the wire grasp orientation at a grasp point on the wire 

    # P = raw control points to make up bezier curve 
    print("grasp_point", grasp_point)

    curve = self.BCurve(t,P)
    element = 25
    TOL = 0.08

    for i in range(len(t)):
      if np.array_equal(curve[[i],:],grasp_point):
        element = i
        print("exactl point found")
        break
    if element == 25:
      for i in range(len(t)):
        diff = grasp_point - curve[[i],:]
        diff = diff.flatten()
        if np.abs(float(diff[0])) < TOL and np.abs(float(diff[1])) < TOL and np.abs(float(diff[2])) < TOL:
          element = i
          print("approximate point found")
          break

    Frame = self.TNB_frame(t,P,element)
    Grasp = self.grasp_correction(Frame,threshold)

    r = R.from_matrix(Grasp)
    q = r.as_quat() # x y z w

    return q


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