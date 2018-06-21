#!/usr/bin/env python

import time
import numpy as np
import math

import scipy.linalg
import control

import rospy
from geometry_msgs.msg import PointStamped, Point, PoseWithCovariance
from truck_gps.msg import Array, Velocity, Acceleration, AttitudeQuaternion

# Store velocity & attitude & acceleration data
DroneVelocity = Point()
DroneAcceleration = Point()
DroneAttitude = Point()


dt = 0.1
q = 0.9

m = 3.1
g = 9.80665
kd = 0.0705

# lqrPub = rospy.Publisher("/LQR_K", Array, queue_size=10)
lqrPub = rospy.Publisher("/LQR_K", PoseWithCovariance, queue_size=10)

def PublishK(K):
    # K = K.tolist()
    
    # msgLQR = Array()
    # msgLQR.header.stamp = rospy.Time.now()
    # msgLQR.data1 = K[0]
    # msgLQR.data2 = K[1]

    K = K.reshape(1,-1)[0]
    Kpad = np.zeros(36)
    Kpad[0:len(K)] = Kpad[0:len(K)] + K
    # print(Kpad)

    msgLQR = PoseWithCovariance()
    msgLQR.covariance = Kpad
    rospy.loginfo(msgLQR)
    lqrPub.publish(msgLQR)   


def Get_LQR_controller():

    A = np.array([[0, 0, 1, 0], \
                  [0, 0, 0, 1], \
                  [0, 0, -kd/m, 0 ], \
                  [0, 0, 0, -kd/m ]])
   
    B = np.array([[0, 0], \
                  [0, 0], \
                  [-g, 0], \
                  [0, -g]])

    Q = q * np.array([[1, 0, 0, 0], \
                      [0, 1, 0, 0], \
                      [0, 0, 10, 0], \
                      [0, 0, 0, 10]])

    R = (1-q) * np.array([[1, 0],\
                          [0, 1]])

    K, S, E =  control.lqr(A, B, Q, R)  # continuous time lqr

    return K





def Drone_velocity_Callback(data):
    global DroneVelocity

    DroneVelocity.x = data.vx
    DroneVelocity.y = data.vy
    DroneVelocity.z = data.vz

def Drone_acceleration_Callback(data):
    global DroneAcceleration

    DroneAcceleration.x = data.ax
    DroneAcceleration.y = data.ay
    DroneAcceleration.z = data.az

def Drone_attitude_Callback(data):
    global DroneAttitude

    roll  = math.atan2(2.0 * (data.q3 * data.q2 + data.q0 * data.q1) , 1.0 - 2.0 * (data.q1 * data.q1 + data.q2 * data.q2))
    pitch = math.asin(2.0 * (data.q2 * data.q0 - data.q3 * data.q1))
    yaw   = math.atan2(2.0 * (data.q3 * data.q0 + data.q1 * data.q2) , - 1.0 + 2.0 * (data.q0 * data.q0 + data.q1 * data.q1))

    DroneAttitude.x = roll
    DroneAttitude.y = pitch
    DroneAttitude.z = yaw


def TimerCallback(event):

    K = Get_LQR_controller()
    PublishK(K)



def main():

    rospy.init_node('LQR_controller', anonymous=True)

    rospy.Subscriber("/dji_sdk/velocity", Velocity, Drone_velocity_Callback , queue_size=2)
    rospy.Subscriber("/dji_sdk/acceleration",Acceleration, Drone_acceleration_Callback, queue_size=2)
    rospy.Subscriber("/dji_sdk/attitude_quaternion", AttitudeQuaternion, Drone_attitude_Callback,  queue_size=2)
    
    dTimeStep = 0.1
    rospy.Timer(rospy.Duration(dTimeStep), TimerCallback)

    rospy.spin()



if __name__ == '__main__':
    main()



    # #~ A = np.array([[1, 0, dt, 0], \
    #               #~ [0, 1, 0, dt], \
    #               #~ [0, 0, 1, 0 ], \
    #               #~ [0, 0, 0, 1 ]])
    # #~ 
    # #~ B = np.array([[dt**2/2, 0], \
    #               #~ [0, dt**2/2], \
    #               #~ [dt, 0], \
    #               #~ [0, dt]])
    # # A = np.array([[0, 0, 1, 0], \
    # #               [0, 0, 0, 1], \
    # #               [0, 0, 0, 0 ], \
    # #               [0, 0, 0, 0 ]])

    # # B = np.array([[0, 0], \
    # #               [0, 0], \
    # #               [1, 0], \
    # #               [0, 1]])

    # A = np.array([[0, 0 ], \
    #               [0, 0 ]])

    # B = np.array([[1, 0], \
    #               [0, 1]])

    
    # Q = q * np.array([[2, 0], \
    #                   [0, 2]])

    # R = (1-q) * np.array([[1, 0],\
    #                       [0, 1]])

    # #~ K, S, E = dlqr(A, B, Q, R)   #  discrete time lqr




    # def dlqr(self, A, B, Q, R):
    # """Solve the discrete time lqr controller.
 
 
    # x[k+1] = A x[k] + B u[k]
     
    # cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
    # """
    # #ref Bertsekas, p.151
 
    # #first, try to solve the ricatti equation
    # X = np.matrix(scipy.linalg.solve_discrete_are(A, B, Q, R))
     
    # #compute the LQR gain
    # K = np.matrix(scipy.linalg.inv(B.T*X*B+R)*(B.T*X*A))
     
    # eigVals, eigVecs = scipy.linalg.eig(A-B*K)
     
    # return K, X, eigVals