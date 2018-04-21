import rospy
import time
import numpy as np
import math
# import control
import scipy.linalg
from geometry_msgs.msg import PointStamped
import control

# Control Variables
dt = 0.1
q = 0.9

# Dynamic system parameters
m = 2.5
g = 9.8
kd = 1

def publish_gain(K):

    msgLQR = PointStamped()
    msgLQR.header.stamp = rospy.Time.now()
    msgLQR.point.x = K
    msgLQR.point.y = 0
    msgLQR.point.z = 0
    lqrPub.publish(msgLQR) 


def get_lqr_gain():

    # A = np.array([[1, 0, dt, 0], \
    #               [0, 1, 0, dt], \
    #               [0, 0, 1, 0 ], \
    #               [0, 0, 0, 1 ]])

    # B = np.array([[dt**2/2, 0], \
    #               [0, dt**2/2], \
    #               [dt, 0], \
    #               [0, dt]])

    # Q = q * np.array([[1, 0, 0, 0], \
    #                   [0, 1, 0, 0], \
    #                   [0, 0, 10, 0 ], \
    #                   [0, 0, 0, 10 ]])

    # R = (1-q) * np.array([[2, 0],\
    #                       [0, 2]])

    A = np.array([[0, 0, 1, 0], \
                  [0, 0, 0, 1], \
                  [0, 0, -1/m*kd, 0 ], \
                  [0, 0, 0, -1/m*kd ]])

    B = np.array([[0, 0], \
                  [0, 0], \
                  [-g, 0], \
                  [0, g]])

    Q = q * np.array([[1, 0, 0, 0], \
                      [0, 1, 0, 0], \
                      [0, 0, 10, 0 ], \
                      [0, 0, 0, 10 ]])

    R = (1-q) * np.array([[2, 0],\
                          [0, 2]])

    # K, S, E = dlqr(A, B, Q, R)   #  discrete time lqr
    K, S, E = control.lqr(A, B, Q, R)  # continueous time lqr

    return K


def dlqr(self, A, B, Q, R):
    """Solve the discrete time lqr controller.
    x[k+1] = A x[k] + B u[k]
    cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
    """
    #ref Bertsekas, p.151
 
    #first, try to solve the ricatti equation
    X = np.matrix(scipy.linalg.solve_discrete_are(A, B, Q, R))
     
    #compute the LQR gain
    K = np.matrix(scipy.linalg.inv(B.T*X*B+R)*(B.T*X*A))
     
    eigVals, eigVecs = scipy.linalg.eig(A-B*K)
     
    return K, X, eigVals


def TimerCallback(event):

    K = get_lqr_gain()
    publish_gain(K)



def main():

    rospy.init_node('LQR_controller', anonymous=True)

    lqrPub = rospy.Publisher("/LQR/K", PointStamped, queue_size=10)

    dTimeStep = 0.1
    rospy.Timer(rospy.Duration(dTimeStep), TimerCallback)


    rospy.spin()



if __name__ == '__main__':

  main()