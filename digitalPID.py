#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt

# PID gains
Kp = 0   # proportional
Kd = 0   # derivative
Ki = 0   # integral


deltaT = 0.01         # in seconds
totalTime = 3         # also in seconds
N = int(totalTime/deltaT)  # total number of discrete time samples
setPoint = 0          # horizontal
# real time samples, used for plotting
times = np.linspace(start=0, stop=totalTime, num=N)

g = 9.81     # m/s/s
m = 3        # kg
l = 0.5      # m
friction = 2  # N/rad/sec
maxIntegral = 4*m*g  # to avoid spectacular overshoots
maxThrust = 4*m*g  # this is maximum thrust for the motor
integralTerm = 0  # integral term for integrating the error, starts at zero

theta = list()  # make space for the angles (will be overwritten)
thetadot = list()  # make space for the first derivative
thetadotdot = list()  # make space of the second derivative
error = list()

theta = [-(np.pi)/4]  # starts diagonally down
thetadot = [0]  # starts static
thetadotdot = [0]  # starts with no acceleration
forces = [0]   # initialize the list of forces with zero
error = [0]


# The main loop computing everything in the simulation
for i in range(1, N):
  error.append(setPoint - theta[i-1])  # computer the error at each step
  # PID calculation
  proportional = Kp*error[i]
  derivative = Kd*(error[i]-error[i-1])/deltaT
  integralTerm += Ki*error[i]*deltaT
  # limit the integral accumulation both up and down
  integralTerm = max(integralTerm, 0)
  integralTerm = min(integralTerm, maxIntegral)
  bigF = proportional + derivative + integralTerm  # commanded force

  #  saturate the force between 0 and maxThrust
  bigF = min(bigF, maxThrust)
  bigF = max(0, bigF)
  forces.append(bigF)  # save the current force for plotting it later

  thetadotdot.append(bigF/(m*l)-np.cos(theta[i-1])*g/l - friction*thetadot[i-1])
  thetadot.append(thetadot[i-1]+thetadotdot[i]*deltaT)
  theta.append(theta[i-1]+thetadot[i]*deltaT)


# Now let's see how it looks

fig = plt.figure(1)

# Fist subplot is for the arm position
plt.subplot(2, 1, 1)
plt.plot(times, theta)

ones = [1] * len(times)  # Replaces "ones" from Matlab
ones = np.array(ones)

# Various useful limits:
plt.plot(times, ones*setPoint, 'g')  # the set point
plt.plot(times, ones*np.pi/2, 'k-')  # top of travel
plt.plot(times, -1*ones*np.pi/2, 'k-')  # bottom of travel
plt.plot(times, ones*np.pi*10/180, 'r')  # 10 degree overshoot
plt.plot(times, -ones*np.pi*10/180, 'r')  # 10 degree undershoot
plt.xlabel('Time [s]')
plt.ylabel('Angular position [rad]')


# Second subplot is for the force applied
plt.subplot(2, 1, 2)
plt.plot(times, forces)
plt.xlabel('Time [s]')
plt.ylabel('Force applied [N]')

fig.tight_layout()
plt.savefig("digitalPIDPlot.png", dpi=500)
plt.show()
