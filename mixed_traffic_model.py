import IDM, ACC from longitudinal_cfmodels 


class MTM:
    
        def __init__(self, long_model, s0y, s0y_lat, s0y_b, s0y_lat_b, sens_lat, tau_lat_ovm, sens_dvy):
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
            """
            self.long_model = long_model
            self.s0y = s0y
            self.s0y_b = s0y_b
            self.s0y_lat = s0y_lat
            self.s0y_lat_b = s0y_lat_b
            self.sens_lat = sens_lat     # this part is somewhat different from the equation, the given original explnation 
            self.tau_lat_ovm = tau_lat_ovm
            self.sens_dvy = sens_dvy
    
            # Assuming global constants for boundary parameters are defined elsewhere
            self.acc_lat_int_max = 4 * long_model.b  # Example of linking to the long_model's attribute
            self.acc_lat_b_max = glob_acc_lat_b_max  # Placeholder for global constants
            self.acc_lat_b_ref = glob_acc_lat_b_ref
            self.acc_long_b_ref = glob_acc_long_b_ref
            self.antic_factor_b = glob_antic_factor_b
            self.nj = 8  # Number of discrete steps

        def copy(self, mixed_model):
            """
            Creates a deep copy of the provided MTM instance.
    
            Parameters:
                mixed_model (MTM): The MTM instance to copy.
            """
            # Assuming long_model has its own copy method
            self.long_model = mixed_model.long_model.copy()
            self.s0y = mixed_model.s0y
            self.s0y_b = mixed_model.s0y_b
            self.s0y_lat = mixed_model.s0y_lat
            self.s0y_lat_b = mixed_model.s0y_lat_b
            self.sens_lat = mixed_model.sens_lat
            self.tau_lat_ovm = mixed_model.tau_lat_ovm
            self.sens_dvy = mixed_model.sens_dvy


        ##############################################################
        ## Longitudinal Acceleration 
        ##############################################################

        def calc_acc_long_int(dx, dy, vx, vxl, axl, Ll, Wavg):
            
            """
            Calculate the interaction acceleration for longitudinal direction 
            
            Parameters:
                dx = distance between the leader and follower for horizontal
                dy = distance between the leader and follower for the vertical
                vx = speed of subject vehicle 
                vxl = speed of the leader vehicle 
                Ll = length of the leader 
                Wavg = width average of leader and subject
                long_par_reduct_factor = additional factor needed to impliment the alpha for paralel situation 
        
               
            returns:  veh veh longitudinal acceleration 
            
            """
        
            sx = max(0, dx - Ll)
            sy = abs(dy) - Wavg
        
            acc_cf_int = self.long_model.calc_acc_int(sx, vx, vxl, axl)
            alpha = min(exp(-sy/self.s0y),1)
        
            #if ((dx < Ll)&& (sy> 0)):
            #    alpha = alpha*long_par_reduct_factor
        
            return alpha*acc_cf_int


        def calc_acc_leader_select(dx, dy, vx, vxl, axl, Ll, Wavg):
        
            '''
            Simplified calcAccLongInt to select leaders and followers
            NOTE: Differences to calcAccLongInt:
                  (i) take the maximum of long and lat attenuation because I select
                       partners just on long acceleration!
                  (ii) no special provision for laterally neighboring vehs:
                       global.longParReductFactor
                       (for selecting neighbors, just assume full long force)
            '''
        
            """
            Calculate the interaction acceleration for longitudinal direction 
            
            Parameters:
                dx = distance between the leader and follower for horizontal
                dy = distance between the leader and follower for the vertical
                vx = speed of subject vehicle 
                vxl = speed of the leader vehicle 
                Ll = length of the leader 
                Wavg = width average of leader and subject
               
            returns:  veh veh longitudinal acceleration 
            
            """
        
            sx = max(0, dx - Ll)
            sy = abs(dy) - Wavg
            s0ylarger = max(this.s0y, s0ylarger)
        
            acc_cf_int = self.long_model.calc_acc_int(sx, vx, vxl, axl)
            alpha = min(exp(-sy/self.s0ylarger),1)
        
            return alpha*acc_cf_int
    
        ##############################################################
        ## Lateral Acceleration 
        ##############################################################

        def calc_acc_lat_free(vy): 
            
                """
                Calculate the free acceleration for lateral direction 
                
                Parameters:
                    vy: lateral speed of the subject vehicle 
                    tau_lat_OVM : speed adaptation time 
        
                returns: free lateral acceleration 
                
                """
            
                return -vy/self.tau_lat_OVM        

        def calc_acc_lat_int(x, xl, y, yl, vx, vxl, vy, vyl, axl, Lveh, L1, Weh, Wl, Wroad, logging): 
        
                """
                calculates the desired interaction lateral acceleration
        
                Parameters:
                    x: front position of the subjet vehicle
                    xl: front position of the leading vehicle 
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
        
                    dx: longitudinal distance =u[other vehicle]-u [m]
                    dy: lateral distance =v[other vehicle]-v [m]
                    sx: determined whether there is an overlap or not (0 for overlap, the gap between vehicles if not)
        
                Returns:
                    calc_acc_lat_int: desired lateral interaction acceleration [m/s^2] (including sign)
                
                """
        
                dx = xl - x 
                sx = max(0,dx)
                acc_cf_int = self.long_model.calc_acc_int(sx, vx, vxl, axl)
        
                dy = yl - y
                sign_dl = -1 if dy < 0 else 1
                Wavg = 0.5*(Weh+Wl)
        
                overlap = (abs(dy) < Wavg)
        
                alpha = -sign_dy*(abs(dy)/Wavg if (overlap) else exp(abs(dy)-Wavg)/self.s0y_lat)  # we have an confusion here to get solved 
        
                if overlap == True:
        
                    sylb_right = 0.5*Wroad - yl - 0.5*Wl; # right gap leader and road boundary 
                    sylb_left = Wroad - sylb_right - Wl # left gap leader and road boundary 
                    too_narrow_right = (sylb_right< Wveh + self.s0y_latb)
                    too_narrow_left = (sylb_left < Wveh + self.s0y_latb)
        
                    if not (too_narrow_right and too_narrow_left): 
        
                        if (too_narrow_right and (y>yl)): alpha = -1 
                        if (too_narrow_left and (y < yl)): alpha = 1
        
                v0_lat_int = -(self.sens_lat)*alpha*acc_cf_int
        
                if overlap == True: 
                    mult_dv_factor = 1
                else: 
                    mult_dv_factor = max(0,1-self.sens_dvy*sign_dy*(vyl-vy))
        
                acc_lat_int = v0_lat_int / self.tau_lat_OVM*mult_dv_factor: # this part is different from the orignal equation 
        
                acc_lat_int = max(-self.acc_lat_int_max, min(self.acc_laat_int_max, acc_lat_int))
        
                if logging:
                    print("MTM.calcAccLatInt:",
                          "x=", formd(x),
                          "dx=", formd(dx),
                          "y=", formd(y),
                          "dy=", formd(dy),
                          "vx=", formd(vx),
                          "vy=", formd(vy),
                          "accCFint=", formd(accCFint),
                          "\n                     alpha=", formd(alpha),
                          "mult_dv_factor=", formd(mult_dv_factor),
                          "v0LatInt=", formd(v0LatInt),
                          "accLatInt=", formd(accLatInt))
        
                return acc_lat_int


        ##############################################################
        ## Lateral and longitudinal boundaries 
        ##############################################################

        
        def alpha_long_b_fun(sy):
        
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

        def alpha_lat_b_fun(sy): 
           
            """
            Calculate lateral attenuation factor for lateral boundary 
            
            Parameters:
                sy: abs(dy) - Wavg: calculated in calc_acc_long_int
            
            returns: lateral attenuation factor for lateral boundary 
                 
            """
        
            if sy > 0: 
                alpha = exp(-sy/self.s0y_lat_b)
            else: 
                alpha = 1 - sy/this.s0y_lat_b
        
            return alpha 


        def calc_acc_b(width_left, width_right, x, y, vx, vy, Wveh):
        
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
            
            log = false 
        
            Tantic = self.antic_factor_b*(this.long_model.T)
            dTantic = 0 
        
            alpha_long_left_max = 0
            alpha_long_right_max = 0 
            alpha_lat_left_max = 0 
            alpha_lat_right_max = 0 
        
            v0y_b_left = 0 
            v0y_b_right = 0 
        
            # loop over spatial anticipations dx_antic=x+vx*TTC: find max interaction
        
            for j in range(self.nj):
        
                TTC = j*dTantic 
                weight = exp(-TTC/Tantic)
                sy_left = width_left(x+vx*TTC) + y - 0.5*Wveh  # y positive to right
                sy_right = width_right(x*vx*TTC) -y - 0.5*Wveh # y corresp to vehicle.v
        
                if (j > 0): 
        
                    v0y_b_left = max(v0y_b_left, -sy_left / TTC)
                    v0y_b_right = min(v0y_b_left, sy_right/TTC)
        
                alpha_long_left = self.alpha_long_b_fun(sy_left)*weight
                alpha_long_right = self.alpha_long_b_fun(sy_right)*weight
                alpha_lat_left = self.alpha_lat_b_fun(sy_left)*weight
                alpha_lat_right = self.alpha_lat_b_fun(sy_right)*weight
        
                
                alpha_long_left_max = max(alpha_long_left,alpha_long_left_max )
                alpha_long_right_max =  max(alpha_long_right,alpha_long_right_max )
                alpha_lat_left_max = max(alpha_lat_left,alpha_lat_left_max )
                alpha_lat_right_max =  max(alpha_lat_right, alpha_lat_right_max)
                
                v0y = v0y_b_left if (abs(v0y_b_left) > abs(v0y_b_right)) else v0y_b_right 
                
                acc_long_b = self.acc_long_b_ref*(-alpha_long_left_max -alpha_long_right_max)
                acc_lat_b = self.acc_lat_b_ref*(alpha_lat_left_max - alpha_lat_right_max)
        
                acc_long_b *= vx/v0max
                acc_lat_b *= (0.2 + 0.8*vx/v0max)
        
                acc_lat_b_restr = max(-self.acc_lat_b_max, min(acc_lat_b_max, acc_lat_b)) # rarely in effect#
        
                # return is changed from the original js code, since the main objective is to get acc long b and acc lat b 
                return  acc_long_b, acc_lat_b
                
