from longitudinal_cfmodels import IDM, ACC 
from math import exp
import math

class IAM:
    
    def __init__(self, long_model, s0y, s0b, lat_sens, relax_time, fb, gb, lc_param):
        
        """
        
        Initializes an IAM instance with the specified parameters.
    
        Parameters:
            long_model: The underlying longitudinal car-following model IDM 
            
            s0y (float) : Attenuation width 
            s0b (float) : Boundary attenuation width
            lat_sens (float) :  lateral sensivity (s) 
            relax_time :  SFM transversal relaxation time 
            fb : long. de acc at the boundary. 
            gb : lat. acc at the boundary. 
            
        """
        
        self.long_model = long_model
        self.s0y = s0y
        self.s0b = s0b
        self.lat_sens = lat_sens
        self.relax_time = relax_time
        self.fb = fb      
        self.gb = gb 
        self.lc_param = lc_param

    ##############################################################
    ## Longitudinal Acceleration 
    ##############################################################

    def calc_acc_long_int(self, dx, dy, vx, vxl, axl, Wavg):
        
        """
        Calculate the interaction acceleration for longitudinal direction 
        
        Parameters:
        
            dx = distance between the leader and follower for horizontal, we input the real gap. 
            dy = distance between the leader and follower for the vertical (difference bettween y, not the real gap) 
            vx = speed of subject vehicle 
            vxl = speed of the leader vehicle 
            axl = leader acceleration. 
            Ll = length of the leader 
            Wavg = width average of leader and subject
        
        Returns:  
            float: veh-veh longitudinal acceleration.
        """

        sy = abs(dy) - Wavg
        acc_cf_int = self.long_model.calc_acc_int(dx, vx, vxl, axl)
        alpha = min(exp(-abs(sy)/self.s0y), 1)  ## s0y - attenuation width. 

        if (dx < 0) & (sy > 0):  # for the paralle driving. interaction ignored. 
            alpha = 0 
        
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
            
                return -vy/self.relax_time      

    def calc_acc_lat_int(self, dx, dy, vx, vxl, vy,  axl,  Wavg ): 
        
                """
                calculates the desired interaction lateral acceleration
        
                Parameters:
                    dx: distance between the leader and follower for horizontal, we input the real gap. 
                    dy: distance between the leader and follower for the vertical (difference bettween y, not the real gap)
                    vx: longitudinal speed of the subject vehicle 
                    vxl: longitudinal speed of the leading vehicle 
                    vy: lateral speed of the subject vehicle 
                    axl: longitudinal acceleration of the leading vehicle 
                    Wavg: width averagea leader and follower. 

                other optimizing params. 
                    self.lat_sens - lateral sensivity 
                    self.s0y - attenutaiton width 
                    self.relax_time - SFM transversal relaxation time. 
                
                Returns:
                    calc_acc_lat_int: desired lateral interaction acceleration [m/s^2] (including sign) (only between two vehicles)
                
                """
        
        
                acc_cf_int = self.long_model.calc_acc_int(dx, vx, vxl, axl)
                
                sign_dy = -1 if dy < 0 else 1 
        
                sy = abs(dy) - Wavg 
        
                overlap = (abs(dy) <= Wavg)

                if overlap == True: 
                    
                    acc_lat_int_w = self.lat_sens*acc_cf_int*(dy/Wavg) ### lat_sens new parameter 
                    
                else: 
                    
                    acc_lat_int_w = self.lat_sens*sign_dy*exp(-sy/self.s0y)  #### s0y attenuation width 

                acc_lat_int = (acc_lat_int_w - vy)/self.relax_time #### relax_time new parameter
                    
                return acc_lat_int


        ##############################################################
        ## Lateral and longitudinal boundaries 
        ##############################################################

    def calc_acc_b(self, width_left, width_right, vx, vxo, Wveh):
        
            """
            Calculate lateral and longitudianl boundary effect for both lateral and longitudinal acceleration 
            
            Parameters:
               width_left:  abs(y - )
               width_right: same for rightBd-roadAxis
               vx = longitudinal speed of the subject  
               vxo = desired long speed from IDM 
               Wveh = width of the subject vehicle 

            other optimizing params. 
                self.fb - long. de-acc at the boundary. 
                self.gb - lat. acc at the boundary.
                self.sob - boundary attenuation width       
            
            returns: boundary effect for both lateral and longitudinal acceleration 
                 
            """    
        
            sr = abs(width_left  - 0.5*(Wveh))
            sl = abs(width_right - 0.5*(Wveh))

            alpha_r = min(exp(-sr/self.s0b), 1)  ## new parameter s0b. boundary attenuation width. 
            alpha_l = min(exp(-sl/self.s0b), 1)

            acc_long_b_r = -alpha_r*self.fb*(vx/vxo) ### new parameter self.fb - long. de-acc at the boundary. 
            acc_long_b_l = -alpha_l*self.fb*(vx/vxo)

            acc_lat_b_r =   -alpha_r*self.gb  ###  the gap for the right side have an impact to the left side acc increase. 
            acc_lat_b_l =   alpha_l*self.gb   ### note that we use + for the right side

            acc_long_b = acc_long_b_r +  acc_long_b_l

            acc_lat_b = acc_lat_b_r +  acc_lat_b_l

            return  acc_long_b, acc_lat_b



    
        ##############################################################
        ## floor fields 
        ##############################################################

    def calc_ff(self, width_ff):

        '''
        This calcualte the ff for the vehilce to use in lateral acceleration. 
        '''

        
        ff = self.lc_param*math.cos(2*math.pi*width_ff/14.5)


        return ff 
        


                
