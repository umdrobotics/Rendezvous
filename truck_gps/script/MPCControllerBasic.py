#!/usr/bin/env python2
# -*- coding: UTF-8 -*-

import numpy as np

class MPCControllerBasic(object):

    # Constructor.
    def __init__(self):
        
        self.A = A
        self.B = B
        self.Q = Q
        self.R = R
        self.M = M
        self.P = P


    def Initialize(self):
        (self.nx,self.nu) = np.shape(self.B)
            
        # Build Ap
        self.Ap = np.zeros((self.nx*self.P,self.nx))
        Ap0 = np.eye(self.nx)
        for i in range(1,P+1):
            Ap0 = np.dot(Ap0,self.A)
            self.Ap[(i-1)*self.nx+(1:self.nx),:] = Ap0
        
        
        # Build Bp
        self.Bp = np.zeros((self.nx*self.P,self.nu*self.M))
        for i in range(1,P+1):
            Bpj = np.zeros((self.nx,self.nu*self.M))
            if i < self.M
                Bpi = self.B
            else
                Bpi = np.dot((np.mat(self.A)**(i-self.M)), self.B)
            
            
            for j in range(min(self.M,i)+1,0,-1) 
                Bpj[:,(j-1)*self.nu+(1:self.nu)] = Bpi
                Bpi = np.dot(self.A,Bpi)
            end
            self.Bp[(i-1)*self.nx+(1:self.nx),:] = Bpj
            
        end
        
        # Initialize Um
        self.Um = np.zeros((self.nu*self.M,1))


    def Predict(self):

        Xp = np.dot(self.Ap,xk) + np.dot(self.Bp,Um)

        return Xp


    def ComputeOptimalInput(self, StateError):

        self.K = np.dot(np.dot(np.dot(np.dot((self.Bp.T,self.Q),self.Bp) + self.R).I,self.Bp.T),self.Q)
        K = self.K    
        
        Umd = np.dot(-self.K,StateError)
        self.Um = self.Um + Umd
        Um = self.Um
        
        Umr = np.reshape(Um, (self.nu, self.M))

        S = np.zeros((self.M,1))
        S[1,:] = 1

        uk = Umr*S

        return uk, Um, Umr, K



    def CorrectPrediction(self,xk):

        ek = self.Xp(1:self.nx) - xk
        H = ones(self.P*self.nx, self.nx)
        Xpc = self.Xp + np.dot(H,ek)

        return Xpc



def RunTest():

    dt = 0.1

    A = [1    0   dt  0  
         0    1   0   dt
         0    0   1   0
         0    0   0   1 ]

    B = [dt^2/2     0
         0          dt^2/2
         dt         0
         0          dt]


    P = 20
    M = 3

    q = 0.7
    Q = q*eye(4*P)
    for i in range(1:P+1):
        Q(4*(i)-1,4*(i)-1) = Q(4*(i)-1,4*(i)-1)*2
        Q(4*(i),4*(i)) = Q(4*(i),4*(i))*2
    
    R = (1-q)*eye(2*M)

    mpc1 = MPCControllerBasic(A,B,Q,R,M,P)
    mpc1.Initialize(mpc1)

    x0 = [0000]
    u0 = [00]
   
    xend = [102000]
    
    Xp = repmat(x0,mpc1.P,1)
    xk = x0

    stateError = Xp - repmat(xend,mpc1.P,1)
    [uk, Um, Umr, K] = mpc1.ComputeOptimalInput(mpc1,stateError)
    
    Xp = mpc1.predict(mpc1, xk, Um)




if __name__ == '__main__':
    RunTest()       

