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
zreference = signalGenerator(amplitude=5, frequency=0.11, y_offset = 0)
torque = signalGenerator(amplitude=5, frequency=0.02)

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
        
        #d = disturbance.step(t)
        d = 0
        z_ref = zreference.sin(t)
        h_ref = hreference.square(t)
        #r = reference.square(t)
        r = np.array([[z_ref],[h_ref]])
        x = vtol.state
        #f = force.square(t)
        #tor = torque.square(t)
        #u = controller.update(r,x) #
        #forcet = controller.update(f,x)
        #torquet = controller.update(tor,x)
        #fr = (forcet/2)+(torquet/(2*P.d))
        #fr = (f/2)+(tor/(2*P.d)) #
        #fl = (forcet/2)-(torquet/(2*P.d))
        #fl = (f/2)-(tor/(2*P.d)) #
        #u = np.array([fl, fr]) #
        u = controller.update(r,x)
        fr = u[0][0]
        fl = u[1][0]       
       
        #fr_t = u[0][0]
        #fl_t = u[1][0]
        #tor = u[2][0]
        tor = (fr-fl)*P.d
        #fl = fl_t - tor/(2*P.d)
        #fr = fr_t + tor/(2*P.d)
        #fl = fl_t - tor/4
        #fr = fr_t + tor/4

        y = vtol.update([[fr+d],[fl+d]])  # Propagate the dynamics
        t = t + P.Ts  # advance time by Ts
        f = (fr+fl)
        
    # update animation and data plots
    
    animation.update(vtol.state)
    #dataPlot.update(t, vtol.state, z_ref, h_ref, forcet, torquet)
    dataPlot.update(t, vtol.state, z_ref, h_ref, f, tor)
    #dataPlot.update(t, VTOL.state, z_ref, h_ref, f, tor) #
    plt.pause(.3)  

# Keeps the program from closing until the user presses a button.
print('Press key to close')
plt.waitforbuttonpress()
plt.close()
