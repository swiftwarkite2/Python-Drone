# I created this file from the following github https://github.com/byu-controlbook/controlbook_public
import matplotlib.pyplot as plt
import VTOLParam as P
from signalGenerator import signalGenerator
from VTOLAnimation import VTOLAnimation
from dataPlotter import dataPlotter
from VTOLDynamics import Dynamics
import numpy as np
from ctrlPD_IO import ctrlPD

# instantiate satellite, controller, and reference classes
vtol = Dynamics(alpha=0.0)
controller = ctrlPD()
# Zreference controls the characteristics of the horizontal reference signal
zreference = signalGenerator(amplitude=5, frequency=0.11, y_offset = 0)
torque = signalGenerator(amplitude=5, frequency=0.02)
# Href controls the characteristics of the vertical reference signal
force = signalGenerator(amplitude=5, frequency=0.02)
hreference = signalGenerator(amplitude=5, frequency=0.09, y_offset = 0)
noise = signalGenerator(amplitude=0.01)
disturbance = signalGenerator(amplitude=0.01)
# instantiate the simulation plots and animation
dataPlot = dataPlotter()
animation = VTOLAnimation()
reference = signalGenerator(amplitude=30.0*np.pi/180.0, 
                            frequency=0.05)

t = P.t_start  # time starts at t_start
while t < P.t_end:  # main simulation loop
    # Propagate dynamics in between plot samples
    t_next_plot = t + P.t_plot
    # updates control and dynamics at faster simulation rate
    while t < t_next_plot:  
        d = 0
        z_ref = zreference.sin(t)
      # z_ref takes the zreference signal characteristics (amplitude, frequency, y_offset) from above 
      # and creates a sin wave signal with them shown as the green line in the z(m) plot in figure 1. 
      # the Y axis on z(m) plot is equal to the X axis on figure 2

        h_ref = hreference.square(t)
      # h_ref takes the hreference signal characteristics (amplitude, frequency, y_offset) from above 
      # and creates a square wave signal with them shown as the green line in the h(m) plot in figure 
      # 1. the Y axis on h(m) plot is equal to the Y axis on figure 2

        r = np.array([[z_ref],[h_ref]])
        x = vtol.state
        u = controller.update(r,x)
        fr = u[0][0]
        fl = u[1][0]       
        tor = (fr-fl)*P.d
        y = vtol.update([[fr+d],[fl+d]])  # Propagate the dynamics
        t = t + P.Ts  # advance time by Ts
        f = (fr+fl) # fr is force of right rotor, fl is for of left rotor
    # update animation and data plots
    animation.update(vtol.state)
    dataPlot.update(t, vtol.state, z_ref, h_ref, f, tor)
    plt.pause(.3)  

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
