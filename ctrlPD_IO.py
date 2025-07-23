# I created this file from the following github https://github.com/byu-controlbook/controlbook_public
import numpy as np
import VTOLParam as P


class ctrlPD:
    def __init__(self):
        # tuning parameters
        tr = 2  # tuned for faster rise time before saturation.
        zeta = 0.707
        # desired natural frequency
        wn = 2.2 / tr
        alpha1 = 2.0 * zeta * wn
        self.alpha1 = alpha1
        alpha0 = wn**2
        self.alpha0 = alpha0
        
        # compute PD gains for h (vertical drone location)
        
        self.kp = alpha0 * (P.mc + P.mr * 2) 
        self.kd = (P.mc + P.mr * 2) * alpha1 

        tr_th = 0.2 # rise time for inner loop #
        zeta_th = 0.707  # inner loop damping ratio 
        M = 10.0  # Time scale separation between loops
        zeta_z = 0.707  # outer loop damping ratio
        # saturation limits
        self.theta_max = 10.0*np.pi/180.0 
        self.tau_max = 10

        # compute PD gains for theta (drone angle)
        wn_th = 2.2 / tr_th
        self.kp_th = (wn_th**2 * (P.Jc + P.mr * 2 * P.d**2)) / P.d
        self.kd_th = (2 * zeta_th * wn_th * (P.Jc + P.mr * 2 * P.d**2)) / P.d 
        # DC gain for inner loop
        DC_th = 1
        
        # PD design for outer loop
        tr_z = M * tr_th  # rise time for outer loop
        wn_z = 2.2 / tr_z
        
        # compute PD gains for z (horizontal drone location)

        self.kp_z = wn_z**2 * -(P.mc + 2 * P.mr)*0.066
        self.kd_z = 2 * zeta_z * wn_z * -(P.mc + 2 * P.mr)*0.066
        
        # DC gain for outer loop
        k_DC_z = DC_th * self.kp_z \
            / ( DC_th * self.kp_z)
        # print control gains to terminal        
        print('k_DC_z', k_DC_z)
        print('kp_th: ', self.kp_th)
        print('kd_th: ', self.kd_th)
        print('kp_z: ', self.kp_z)
        print('kd_z: ', self.kd_z)
        print('kp: ', self.kp)
        print('kd: ', self.kd)

    def update(self, r, x):

        z = x[0][0]
        h = x[1][0]
        theta = x[2][0]
        zdot = x[3][0]
        hdot = x[4][0]
        thetadot = x[5][0]
        z_ref = r[0][0]
        h_ref = r[1][0]

        force = (P.mc+P.mr*2)*P.g #This is the gravitational force, it is the sum of all mass multiplied by the gravitational constant.
        force_tilde = self.kp * (h_ref - h) - self.kd * hdot #h_ref is the vertical reference target signal specified in the sim file. In our case its a square wave.
                                            # h is the actual vertical location of the planar quadrotor at the current moment. Hdot is the actual vertical velocity at the current moment.

        total_force = force + force_tilde 
        total_force = saturate(total_force, 2*P.fmax)
# Outer loop is calculating a reference angle based on the the proportional gain (kp_z) * 
# the difference the horizontal reference signal and the actual horizontal location - the derivative gain (kd_z) * the horizontal velocity
        theta_r = self.kp_z * (z_ref - z) \
            - self.kd_z * zdot
        theta_r = saturate(theta_r, 10)

        # inner loop: outputs the torque applied to the base

        tau = self.kp_th * (theta_r - theta) \
            - self.kd_th * thetadot
        tau = saturate(tau, 10)

        # third level of control equations. The kp and kd calculated first are used to create the total force. The kp_th and kd_th 
        # (inner loop - derived from the thetaddot equation of motion) are used to calculate tau with the help of theta_r 
        # which was calculated using kp_z and kd_z (outer loop - derived from the zddot equation of motion)        
        fr = total_force/2 + tau/(2*P.d)
        fl = total_force/2 - tau/(2*P.d)       
        fr = saturate(fr, 10)
        fl = saturate(fl, 10)
        out = np.array([[fr],[fl]])
        return out

def saturate(u, limit):
    if abs(u) > limit:
        u = limit * np.sign(u)
    return u
