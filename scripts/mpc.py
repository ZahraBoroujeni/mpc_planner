#!/usr/bin/env python

from sys import path
path.append(r"/home/mi/boroujeni/casadi-py27-np1.9.1-v3.1.0-rc1")
from casadi import *
from casadi.tools import *
from pylab import *
import roslib; roslib.load_manifest('rospy_tutorials')

import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String)
    rospy.init_node('talker', anonymous=True)
    r = rospy.Rate(10) # 10hz
    str = "hello world %s"%rospy.get_time()
    rospy.loginfo(str)
    pub.publish(str)
    r.sleep()
def mpc():
    N = 100   # Control discretization
    T = 10.0    # End time

    # Which QP_solver to use
    qp_solver_class = "ipopt"
    #qp_solver_class = "qpoases"

    # Declare variables (use scalar graph)
    u  = SX.sym("u",4)    # input controls
    x  = SX.sym("x",8)   # states
    p  = SX.sym("p",2)    # parameters
    # System dynamics
    xdot = vertcat(x[2]*cos(x[3]),x[2]*sin(x[3]),u[0],u[1],x[6]*cos(x[7]),x[6]*sin(x[7]),u[2],u[3] )

    L = 0.1*u[0]**2 + 0.01*u[1]**2 + 0.1*u[2]**2 + 0.01*u[3]**2+0.001*(x[0]**2+x[1]**2-2.25)**2+10*((x[0]-1.5)**2 +(x[1]-0)**2)

    f = Function('f',[x,u],[xdot,L])
    U = MX.sym("U",4)
    X0 = MX.sym("X0",8)
    P = MX.sym("P",2)
    M = 10; DT =T/N/M # T/(N*M)
    X = X0
    Q = 0
    for j in range(M):
      k1, k1_q = f(X, U)
      k2, k2_q = f(X + DT/2 * k1, U)
      k3, k3_q = f(X + DT/2 * k2, U)
      k4, k4_q = f(X + DT * k3, U)
      X=X+DT/6*(k1 +2*k2 +2*k3 +k4)
      Q = Q + DT/6*(k1_q + 2*k2_q + 2*k3_q + k4_q)
    F = Function('F', [X0, U], [X, Q],['x0','p'],['xf','qf'])

         
    # Evaluate at a test point
    # Fk = F(x0=[-1.5, 0, 0.4, 1.5,-1, 1.11, 0.0, 0.8],p=[0,0,0,0])
    # print Fk['xf']
    # print Fk['qf']

    # Start with an empty NLP
    w =[]
    w0 = []
    lbw = []
    ubw = []
    J = 0
    g=[]
    lbg = []
    ubg = []
    pi = 3.14
    # "Lift" initial conditions
    X0 = MX.sym('X_0', 8)
    w += [X0]
    lbw += [-3,-1,0,-3.14,-3, 0, 0, -3.14]
    ubw += [3, 3,2,+3.14, 3, 3, 1, +3.14]
    w0 = [-1.5, 0, 0.7, 1.5,-1, 1.11, 0.1, 0.8]

    # # Formulate the NLP
    Xk = MX([-1.5, 0, 0.7, 1.5,-1, 1.11, 0.1, 0.8])
    for k in range(N):
      # New NLP variable for the control
      Uk = MX.sym('U_' + str(k),4)
      w += [Uk]
      w0 += [0,0,0,0]
      lbw += [-0.5, -1, -0.01,-1]
      ubw += [+0.5, +1, +0.01,+1]
      # Integrate till the end of the interval
      Fk = F(x0=Xk, p=Uk)
      Xk_end = Fk['xf']
      J=J+Fk['qf']

      # New NLP variable for state at end of interval
      Xk = MX.sym('X_' + str(k+1), 8)
      w   += [Xk]
      lbw += [-3,-1,0,-3.14,-3, 0, 0, -3.14]
      ubw += [3, 3,2,+3.14, 3, 3, 0.1, +3.14]
      w0  += [0,0,0,0,0,0,0,0]
      theta=atan2(-2*Xk_end[4]/(2*(2.25-Xk_end[4]**2)),1)
      # Add inequality constraint
      g += [Xk_end[0]**2+Xk_end[1]**2,Xk_end[4]**2+Xk_end[5]**2-2.25,((cos(theta)*(Xk_end[0]-Xk_end[4])+sin(theta)*(Xk_end[1]-Xk_end[5]))**2)/((0.3)**2)+((sin(theta)*(Xk_end[0]-Xk_end[4])-cos(theta)*(Xk_end[1]-Xk_end[5]))**2)/(0.25)]
      lbg += [2.2,-0.1,1]
      ubg += [9,0.1,inf] 
      # Add equality constraint
      g   += [Xk_end-Xk]
      lbg += [0,0,0,0,0,0,0,0]
      ubg += [0,0,0,0,0,0,0,0] 
    # Create an NLP solver
    prob = {'f': J, 'x': vertcat(*w), 'g': vertcat(*g)}
    solver = nlpsol('solver', 'ipopt', prob);

    #Solve the NLP
# def slove_mpc():
    sol = solver(x0 = w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)
    w_opt = sol['x'].full().flatten()
    # Plot the solution


    x1_opt = w_opt[12::12]
    x2_opt = w_opt[13::12]

    o1_opt =  w_opt[16::12]
    o2_opt =  w_opt[17::12]

    v=w_opt[14::12]
    print v
    tgrid = [T/N*k for k in range(N)]
    import matplotlib.pyplot as plt
    plt.figure(1)
    plt.clf()
    plt.plot(x1_opt, x2_opt, 'ro')

    plt.axis([-2, 2, 0, 3])

    plt.plot(o1_opt, o2_opt, 'bo')
    # plt.xlabel('t')
    plt.grid()

    plt.figure(2)
    plt.clf()
    plt.plot(tgrid, v, 'ro')
    plt.xlabel('t')
    plt.grid()
    plt.show()     
if __name__ == '__main__':
    try:
      talker()
      mpc()
      slove_mpc()
    except rospy.ROSInterruptException:
        pass
