# this file came from the following github https://github.com/byu-controlbook/controlbook_public
# I had to enter in all the values for each of the parameters
# VTOL Parameter File 
import numpy as np

# Physical parameters of the  VTOL known to the controller
mc = 1.0 # kg
mr = 0.25  # kg
Jc = 0.0042  # kg m^2
d = 0.3  # m
ml = 0.25 #kg
mew = 0.1  # kg/s
g = 9.81  # m/s^2
F_wind = 0.0 # wind disturbance force is zero in initial homeworks

# parameters for animation
length = 5.0

# Initial Conditions
z0 = 0.0  # initial lateral position
h0 = 0.0  # initial altitude
theta0 = 0.0 # initial roll angle
zdot0 = 0.0  # initial lateral velocity
hdot0 = 1.0  # initial climb rate
thetadot0 = 0.0  # initial roll rate
target0 = 0.0

# Simulation Parameters
t_start = 0.0 # Start time of simulation
t_end = 20.0  # End time of simulation
Ts = 0.01  # sample time for simulation
t_plot = 0.1 # the plotting and animation is updated at this rate

# saturation limits
fmax = 10.0  # Max Force, N
tau_max = .1

# dirty derivative parameters
# sigma =   # cutoff freq for dirty derivative
# beta =  # dirty derivative gain

# equilibrium force
# Fe =

# mixing matrix
unmixing = np.array([[1.0, 1.0], [d, -d]]) # converts fl and fr (LR) to force and torque (FT)
mixing = np.linalg.inv(unmixing) # converts force and torque (FT) to fl and fr (LR) 

