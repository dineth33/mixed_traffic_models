import math 

## IDM and ACC 

class IDM:
    
    """
    class for the Intelligent Driver Model (IDM) 
    
    Attributes:
        v0: Desired speed in meters per second (m/s).
        T : Desired time gap in seconds (s).
        s0 : Minimum gap in meters (m).
        a : Maximum acceleration in meters per second squared (m/s^2).
        b : Comfortable deceleration in meters per second squared (m/s^2).
        speedlimit: Speed limit affecting the IDM (initially set to 1000, indicating no restriction).
        bmax : Maximum possible deceleration value.
    """

    def __init__(self, v0, T, s0, a, b):
        """
        Initializes an IDM instance with the provided parameters.

        Parameters:
            v0: Desired speed in meters per second (m/s).
            T: Desired time gap in seconds (s).
            s0: Minimum gap in meters (m).
            a: Maximum acceleration in meters per second squared (m/s^2).
            b: Comfortable deceleration in meters per second squared (m/s^2).
        """
        self.v0 = v0
        self.T = T
        self.s0 = s0
        self.a = a
        self.b = b
        self.speedlimit = 1000  # initially set to 1000, indicating no restriction
        self.bmax = 9  # maximum possible deceleration value


    def copy(self, long_model):
        """
        Copies the attributes from another IDM instance (long_model) to this instance.

        Parameters:
            long_model (IDM): The IDM instance from which to copy the attributes.
        """
        self.v0 = long_model.v0
        self.T = long_model.T
        self.s0 = long_model.s0
        self.a = long_model.a
        self.b = long_model.b
        self.speedlimit = long_model.speedlimit
        self.bmax = long_model.bmax
        

    def calc_acc_free(self, v):
        """
        Calculate the free acceleration for the IDM.

        Parameters:
            v : speed of the subject vehicle in meters per second (m/s).

        Returns:
            acc_free : The calculated free acceleration in meters per second squared (m/s^2).
        """
        v0eff = max(0.01, min(self.v0, self.speedlimit))
        
        if v < v0eff:
            acc_free = self.a * (1 - (v / v0eff) ** 4)
        else:
            acc_free = self.a * (1 - v / v0eff)
        return acc_free


    
    def calc_acc_int(self, s, v, vl, al):
        """
        Calculate the IDM interaction acceleration.

        Parameters:
            s : The actual gap to the leading vehicle in meters (m).
            v : The actual speed of the vehicle in meters per second (m/s).
            vl : The speed of the leading vehicle in meters per second (m/s).
            al : The acceleration of the leading vehicle in meters per second squared (m/s^2).
                      (This parameter is ignored in the current IDM implementation.)

        Returns:
            act_int: The calculated interaction acceleration in meters per second squared (m/s^2).
        """
        sstar = self.s0 + max(0, v * self.T + 0.5 * v * (v - vl) / math.sqrt(self.a * self.b))
        acc_int = -self.a * (sstar / max(s, 0.1 * self.s0)) ** 2
        
        return max(-self.bmax, acc_int)





class ACC:
    
    def __init__(self, v0, T, s0, a, b):
        """
        Initializes an ACC instance with the parameters.

        Parameters:
            v0 : Desired speed in meters per second (m/s).
            T : Desired time gap in seconds (s).
            s0 : Minimum gap in meters (m).
            a: Maximum acceleration in meters per second squared (m/s^2).
            b: Comfortable deceleration in meters per second squared (m/s^2).
        """
        self.v0 = v0
        self.T = T
        self.s0 = s0
        self.a = a
        self.b = b
        self.cool = 0.99  # cooler reactions if the gap is too small
        self.speedlimit = 1000  # effective speed limit
        self.bmax = 9  # maximum deceleration

    def copy(self, long_model):
        """
        Copies the attributes from another ACC instance (long_model) to this instance.

        Parameters:
            long_model (ACC): The ACC instance from which to copy the attributes.
        """
        self.v0 = long_model.v0
        self.T = long_model.T
        self.s0 = long_model.s0
        self.a = long_model.a
        self.b = long_model.b
        self.cool = long_model.cool  # Assume cool factor should be copied as well
        self.speedlimit = long_model.speedlimit
        self.bmax = long_model.bmax

    def calc_acc_free(self, v):
        """
        Calculate the free acceleration for the ACC, which is the same as for IDM.

        Parameters:
            v (float): The actual speed of the vehicle in meters per second (m/s).

        Returns:
            acc_free: The calculated free acceleration in meters per second squared (m/s^2).
        """
        v0eff = max(0.01, min(self.v0, self.speedlimit))
        if v < v0eff:
            acc_free = self.a * (1 - (v / v0eff) ** 4)
        else:
            acc_free = self.a * (1 - v / v0eff)
        return acc_free

    def calc_acc_int(self, s, v, vl, al=0):
        """
        Calculate the ACC interaction acceleration.

        Parameters:
            s : The actual gap to the leading vehicle in meters (m).
            v : The actual speed of the vehicle in meters per second (m/s).
            vl: The speed of the leading vehicle in meters per second (m/s).
            al: The acceleration of the leading vehicle in meters per second squared (m/s^2), defaults to 0.

        Returns:
            float: The calculated interaction acceleration in meters per second squared (m/s^2).
        """

        # Determine the effective desired speed
        v0eff = min(self.v0, self.speedlimit)

        # Calculate the desired gap
        sstar = self.s0 + max(0, v * self.T + 0.5 * v * (v - vl) / math.sqrt(self.a * self.b))
        
        # Calculate the free acceleration
        acc_free = self.calc_acc_free(v)

        # Calculate the IDM interaction acceleration
        acc_int_idm = -self.a * (sstar / max(s, 0.1 * self.s0)) ** 2
        acc_idm = acc_free + acc_int_idm

        # Calculate the CAH (Collision Avoidance Heuristic) acceleration
        acc_cah = (v * v * al / (vl * vl - 2 * s * al)) if (vl * (v - vl) < -2 * s * al) else al - (v - vl) ** 2 / (2 * max(s, 0.1)) * (1 if v > vl else 0)
        acc_cah = min(acc_cah, self.a)

        # Combine IDM and CAH accelerations
        acc_mix = acc_idm if acc_idm > acc_cah else acc_cah + self.b * math.tanh((acc_idm - acc_cah) / self.b)

        # Calculate the final ACC acceleration
        acc_acc = self.cool * acc_mix + (1 - self.cool) * acc_idm
        acc_int = max(-self.bmax, acc_acc - acc_free)

        return acc_int


