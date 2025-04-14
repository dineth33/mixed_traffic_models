from longitudinal_cfmodels import IDM, ACC 
from math import exp

class MTM:
    
    def __init__(self, long_model, s0y, s0y_lat, s0y_b, s0y_lat_b, sens_lat, tau_lat_ovm, sens_dvy,
                 acc_lat_b_max, acc_lat_b_ref, acc_long_b_ref, antic_factor_b):
        """
        Initializes an MTM instance with the specified parameters.
    
        Parameters:
            long_model: The underlying longitudinal car-following model (e.g., ACC).
            s0y (float): Lateral attenuation scale for longitudinal veh-veh interaction [m].
            s0y_lat (float): Lateral attenuation scale for lateral veh-veh interaction [m].
            s0y_b (float): Lateral attenuation scale for longitudinal boundary-veh interaction [m].
            s0y_lat_b (float): Lateral attenuation scale for lateral boundary-veh interaction [m].
            sens_lat (float): Sensitivity (desired lateral speed)/(longitudinal accel) [s].
            tau_lat_ovm (float): Time constant for lateral OVM [s].
            sens_dvy (float): Sensitivity of lateral relative speed, similar to FVDM [s/m].
            acc_lat_b_max, acc_lat_b_ref, acc_long_b_ref, antic_factor_b: Global constants.
        """
        
        self.long_model = long_model
        self.s0y = s0y
        self.s0y_b = s0y_b
        self.s0y_lat = s0y_lat
        self.s0y_lat_b = s0y_lat_b
        self.sens_lat = sens_lat     
        self.tau_lat_ovm = tau_lat_ovm
        self.sens_dvy = sens_dvy
    
        # Define boundary parameters from inputs
        self.acc_lat_int_max = 4 * long_model.b  
        self.acc_lat_b_max = acc_lat_b_max  
        self.acc_lat_b_ref = acc_lat_b_ref
        self.acc_long_b_ref = acc_long_b_ref
        self.antic_factor_b = antic_factor_b
        self.nj = 8  # Number of discrete steps

    ##############################################################
    ## Longitudinal Acceleration 
    ##############################################################

    def calc_acc_long_int(self, dx, dy, vx, vxl, axl, Ll, Wavg):
        """
        Calculate the interaction acceleration for longitudinal direction 
        
        Parameters:
            dx = distance between the leader and follower for horizontal, we input the real gap. 
            dy = distance between the leader and follower for the vertical
            vx = speed of subject vehicle 
            vxl = speed of the leader vehicle 
            Ll = length of the leader 
            Wavg = width average of leader and subject
        
        Returns:  
            float: veh-veh longitudinal acceleration.
        """
        sx = dx 
        sy = abs(dy) - Wavg
        acc_cf_int = self.long_model.calc_acc_int(dx, vx, vxl, axl)
        alpha = min(exp(-dy/self.s0y), 1)
        return alpha * acc_cf_int



        ##############################################################
        ## Lateral Acceleration 
        ##############################################################

    def calc_acc_lat_free(self, vy): 
            
                """
                Calculate the free acceleration for lateral direction 
                
                Parameters:
                    vy: lateral speed of the subject vehicle 
                    tau_lat_OVM : speed adaptation time 
        
                returns: free lateral acceleration 
                
                """
            
                return -vy/self.tau_lat_ovm        

    def calc_acc_lat_int(self, dx, y, yl, vx, vxl, vy, vyl, axl, Lveh, L1, Wveh, Wl, Wroad): 
        
                """
                calculates the desired interaction lateral acceleration
        
                Parameters:
                    dx: distance between the leader and follower for horizontal, we input the real gap. 
                    y: lat middle position of the subject vehicle 
                    yl: lat middle position of the leader vehicle. 
                    vx: longitudinal speed of the subject vehicle 
                    vxl: longitudinal speed of the leading vehicle 
                    vy: lateral speed of the subject vehicle 
                    vyl: lateral speed of the leading vehicle 
                    axl: longitudinal acceleration of the leading vehicle 
                    Lveh: Length of subject vehicle 
                    L1: Length of the leader 
                    Weh: Width of the subject vehicle 
                    W1: Width of the leading vehicle (imo)
                    Wroad: width of the road 
        
                    dy: lateral distance =v[other vehicle]-v [m]
                    sx: determined whether there is an overlap or not (0 for overlap, the gap between vehicles if not)
        
                Returns:
                    calc_acc_lat_int: desired lateral interaction acceleration [m/s^2] (including sign)
                
                """
        
                sx = max(0,dx)
                acc_cf_int = self.long_model.calc_acc_int(sx, vx, vxl, axl)
        
                dy = yl - y
                sign_dy = -1 if dy < 0 else 1
                Wavg = 0.5*(Wveh+Wl)
        
                overlap = (abs(dy) < Wavg)
        
                alpha = -sign_dy*(abs(dy)/Wavg if (overlap) else exp(abs(dy)-Wavg)/self.s0y_lat)  # we have an confusion here to get solved 

                # this part is to consider, when there are narrow gaps from the left side and the right side. 
                if overlap == True:
        
                    sylb_right = 0.5*Wroad - yl - 0.5*Wl; # right gap leader and road boundary 
                    sylb_left = Wroad - sylb_right - Wl # left gap leader and road boundary 
                    too_narrow_right = (sylb_right< Wveh + self.s0y_lat_b)
                    too_narrow_left = (sylb_left < Wveh + self.s0y_lat_b)
        
                    if not (too_narrow_right and too_narrow_left): 
        
                        if (too_narrow_right and (y>yl)): alpha = -1 
                        if (too_narrow_left and (y < yl)): alpha = 1
        
                v0_lat_int = -(self.sens_lat)*alpha*acc_cf_int
        
                if overlap == True: 
                    mult_dv_factor = 1
                else: 
                    mult_dv_factor = max(0,1-self.sens_dvy*sign_dy*(vyl-vy))
        
                acc_lat_int = v0_lat_int / self.tau_lat_ovm*mult_dv_factor # this part is different from the orignal equation 
        
                acc_lat_int = max(-self.acc_lat_int_max, min(self.acc_lat_int_max, acc_lat_int))
        
        
                return acc_lat_int


        ##############################################################
        ## Lateral and longitudinal boundaries 
        ##############################################################

        
    def alpha_long_b_fun(self, sy):
        
            """
            Calculate lateral attenuation factor for longitudinal boundary 
            
            Parameters:
                sy: abs(dy) - Wavg: calculated in calc_acc_long_int
            
            returns: lateral attenuation factor for longitudinal boundary 
                 
            """
        
            if sy > 0: 
                alpha = exp(-sy/self.s0y_b)
            else: 
                alpha = 1 
        
            return alpha 

    def alpha_lat_b_fun(self, sy): 
           
            """
            Calculate lateral attenuation factor for lateral boundary 
            
            Parameters:
                sy: abs(dy) - Wavg: calculated in calc_acc_long_int, we use the calculated value in the boundary function 
            
            returns: lateral attenuation factor for lateral boundary 
                 
            """
        
            if sy > 0: 
                alpha = exp(-sy/self.s0y_lat_b)
            else: 
                alpha = 1 - sy/self.s0y_lat_b
        
            return alpha 


    # def calc_acc_b(self, width_left, width_right, x, y, vx, vy, Wveh):
        
    #         """
    #         Calculate lateral and longitudianl boundary effect for both lateral and longitudinal acceleration 
            
    #         Parameters:
    #            width_left:  function pointer roadAxis-leftBd as a funct of arcLength u
    #            width_right: same for rightBd-roadAxis
    #            x = x position of the subejct 
    #            y = y position of the subject 
    #            vx = longitudinal speed of the subject   
    #            vy = lateral speed of the subject
    #            Wveh = width of the subject vehicle 
            
    #         returns: boundary effect for both lateral and longitudinal acceleration 
                 
    #         """    
            
    #         log = False 
        
    #         Tantic = self.antic_factor_b*(self.long_model.T) 
            
    #         dTantic = 1*Tantic/self.nj
    #         dTantic = 0 ## !!! Test: no boundary anticipation. Reduces lateral wiggling
        
    #         alpha_long_left_max = 0
    #         alpha_long_right_max = 0 
    #         alpha_lat_left_max = 0 
    #         alpha_lat_right_max = 0 
        
    #         v0y_b_left = 0 
    #         v0y_b_right = 0 
        
    #         # loop over spatial anticipations dx_antic=x+vx*TTC: find max interaction
        
    #         for j in range(self.nj):
        
    #             TTC = j*dTantic 
    #             weight = exp(-TTC/Tantic)  # always 1, since we do not have anticipation. 
                
    #             sy_left = width_left*(x+vx*TTC) + y - 0.5*Wveh  # y positive to right    # very big value 
    #             sy_right = width_right*(x+vx*TTC) - y - 0.5*Wveh # y corresp to vehicle  # very big value. 
        
    #             if (j > 0): 
        
    #                 v0y_b_left = max(v0y_b_left, -sy_left / TTC) # become a very big value 
    #                 v0y_b_right = min(v0y_b_right, sy_right/TTC)  # always become 0. 
        
    #             alpha_long_left = self.alpha_long_b_fun(sy_left)*weight
    #             alpha_long_right = self.alpha_long_b_fun(sy_right)*weight
    #             alpha_lat_left = self.alpha_lat_b_fun(sy_left)*weight
    #             alpha_lat_right = self.alpha_lat_b_fun(sy_right)*weight
        
                
    #             alpha_long_left_max = max(alpha_long_left,alpha_long_left_max )
    #             alpha_long_right_max =  max(alpha_long_right,alpha_long_right_max )
    #             alpha_lat_left_max = max(alpha_lat_left,alpha_lat_left_max )
    #             alpha_lat_right_max =  max(alpha_lat_right, alpha_lat_right_max)
                
    #         v0y = v0y_b_left if (abs(v0y_b_left) > abs(v0y_b_right)) else v0y_b_right 
            
    #         acc_long_b = self.acc_long_b_ref*(-alpha_long_left_max -alpha_long_right_max) #  because we need both left and right to a single variable 
    #         acc_lat_b = self.acc_lat_b_ref*(alpha_lat_left_max - alpha_lat_right_max)
    
    #         acc_long_b *= vx/self.long_model.v0 # fits with the equation. 
    #         acc_lat_b *= (0.2 + 0.8*vx)/self.long_model.v0 
    
    #         acc_lat_b_restr = max(-self.acc_lat_b_max, min(self.acc_lat_b_max, acc_lat_b)) # rarely in effect#
    
    #         # return is changed from the original js code, since the main objective is to get acc long b and acc lat b 
    #         return  acc_long_b, acc_lat_b


    def calc_acc_b(self, width_left, width_right, x, y, vx, vy, Wveh):
        
            """
            Calculate lateral and longitudianl boundary effect for both lateral and longitudinal acceleration 
            
            Parameters:
               width_left:  function pointer roadAxis-leftBd as a funct of arcLength u
               width_right: same for rightBd-roadAxis
               x = x position of the subejct 
               y = y position of the subject 
               vx = longitudinal speed of the subject   
               vy = lateral speed of the subject
               Wveh = width of the subject vehicle 
            
            returns: boundary effect for both lateral and longitudinal acceleration 
                 
            """    
            
        
             # ursprüngliche gibt es plus oder minue für lecht und reciht, aber wir haben diese width in einem gleiche Wiese genommen. 
            sy_left = width_left - 0.5*Wveh 
            sy_right = width_right - 0.5*Wveh 
        
            alpha_long_left = self.alpha_long_b_fun(sy_left)
            alpha_long_right = self.alpha_long_b_fun(sy_right)
            alpha_lat_left = self.alpha_lat_b_fun(sy_left)
            alpha_lat_right = self.alpha_lat_b_fun(sy_right)
            
            acc_long_b = self.acc_long_b_ref*(-alpha_long_left -alpha_long_right) # because we need both left and right to a single variable 
            acc_lat_b = self.acc_lat_b_ref*(alpha_lat_left - alpha_lat_right)
    
            acc_long_b *= vx/self.long_model.v0 # fits with the equation. 
            acc_lat_b *= (0.2 + 0.8*vx)/self.long_model.v0 
        
            # return is changed from the original js code, since the main objective is to get acc long b and acc lat b 
            return  acc_long_b, acc_lat_b
                
