import numpy as np
import VTOLParam as P


class ctrlPD:
    def __init__(self):
        ####################################################
        #       PD Control: Time Design Strategy
        ####################################################
        # tuning parameters
        tr = 2  # tuned for faster rise time before saturation.
        zeta = 0.707
        # desired natural frequency
        wn = 2.2 / tr
        alpha1 = 2.0 * zeta * wn
        self.alpha1 = alpha1
        alpha0 = wn**2
        self.alpha0 = alpha0
        # compute PD gains
        #self.kp = alpha0*(P.m * P.ell**2) / 3.0
        #self.kd = (P.m * P.ell**2) \
        #            / 3.0 * (alpha1 - 3.0 * P.b / (P.m * P.ell**2))
        self.kp = alpha0 * (P.mc + P.mr * 2)
        self.kd = (P.mc + P.mr * 2) * alpha1
        
        
        
        
        tr_th = 0.2 # rise time for inner loop #
        zeta_th = 0.707  # inner loop damping ratio 
        M = 10.0  # Time scale separation between loops
        zeta_z = 0.707  # outer loop damping ratio
        # saturation limits
        self.theta_max = 10.0*np.pi/180.0  #
        self.tau_max = 10
        
        #self.kp = 0.113
        #self.kd = 0.583

            # maximum commanded base angle
        #---------------------------------------------------
        #                    Inner Loop
        #---------------------------------------------------
        # PD design for inner loop
        wn_th = 2.2 / tr_th
        self.kp_th = (wn_th**2 * (P.Jc + P.mr * 2 * P.d**2)) / P.d
        self.kd_th = (2 * zeta_th * wn_th * (P.Jc + P.mr * 2 * P.d**2)) / P.d 
        # DC gain for inner loop
        DC_th = 1
        #---------------------------------------------------
        #                    Outer Loop
        #---------------------------------------------------
        # PD design for outer loop
        tr_z = M * tr_th  # rise time for outer loop
        wn_z = 2.2 / tr_z
        #AA = np.array([
        #    [P.k * DC_th, -P.b * DC_th * wn_phi**2],
        #    [P.b * DC_th, \
        #        P.k * DC_th \
        #            - 2 * zeta_phi * wn_phi * P.b * DC_th]])    
        #bb = np.array([
        #            [-P.k + P.Jp * wn_phi**2],
        #            [-P.b + 2 * P.Jp * zeta_phi * wn_phi]])
        #tmp = np.linalg.inv(AA) @ bb
        #self.kp_phi = tmp[0][0]
        #self.kd_phi = tmp[1][0]
        force = (P.mc+P.mr*2)*P.g #
        self.kp_z = wn_z**2 * (P.mc + 2 * P.mr)/(-force) #
        self.kd_z = 2 * zeta_z * wn_z * (P.mc + 2 * P.mr + P.mew)/(-force) #
       
        
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
        #theta = state[0][0]
        #z = state[1][0]
        #thetadot = state[2][0]
        #zdot = state[3][0]
        z = x[0][0]
        h = x[1][0]
        theta = x[2][0]
        zdot = x[3][0]
        hdot = x[4][0]
        thetadot = x[5][0]
        z_ref = r[0][0]
        h_ref = r[1][0]

        force = (P.mc+P.mr*2)*P.g
        force_tilde = self.kp * (h_ref - h) - self.kd * hdot
        total_force = force + force_tilde
        total_force = saturate(total_force, 2*P.fmax)
        #fr = total_force/2
        #fl = total_force/2

        # outer loop: outputs the reference angle for theta
        # note that book recommends a feed forward term because
        # of poor DC gain on the outer loop which
        # is why we add an addition "phi_r" at the end

        theta_r = self.kp_z * (z_ref - z) \
            - self.kd_z * zdot
        #theta_r = self.kp_z * (z_ref - z) \
        #    - self.kd_z * zdot
        #theta_r = 0.4 * (z_ref - z) \
        #    - 0.1 * zdot
        theta_r = saturate(theta_r, 10)

        # inner loop: outputs the torque applied to the base

        tau = self.kp_th * (theta_r - theta) \
            - self.kd_th * thetadot
        tau = saturate(tau, 10)
        #tau = 0.01 * (theta_r - theta) \
        #    - 0.8 * thetadot
        #tau = saturate(tau, 0.001)

        #tau = self.kp_z * (theta_r - theta) \
        #    - self.kd_z * thetadot
        
        #return saturate(tau, P.tau_max)
        fr = total_force/2 + tau/(2*P.d)
        fl = total_force/2 - tau/(2*P.d)       
        #fr = total_force/2
        #fl = total_force/2   
        fr = saturate(fr, 10)
        fl = saturate(fl, 10)
        #out = np.array([[fr],[fl],[tau]])
        out = np.array([[fr],[fl]])
        return out

def saturate(u, limit):
    if abs(u) > limit:
        u = limit * np.sign(u)
    return u
