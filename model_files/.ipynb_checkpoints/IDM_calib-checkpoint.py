import numpy as np
import math
from scipy.optimize import minimize


class IDM:
    def __init__(self,v0,T,s0,a,b):
        self.v0=v0
        self.T=T
        self.s0=s0
        self.a=a
        self.b=b

        self.speed_limit = 1000
        self.bmax=9
    
    #free acceleration equation
    '''
    @param v: actual speed (m/s)
    @return : free acceleration (m/s**2)
    '''

    
    def calcAccFree(self,v):

        # determine valid local v0

        v0eff = np.maximum(0.01,np.minimum(self.v0,self.speed_limit))

        accFree = self.a*(1-math.pow(v/v0eff,4)) if v<v0eff else self.a*(1-(v/v0eff))

        return  accFree

    # interaction Acceleration equation

    '''
    @param s:     actual gap [m]
    @param v:     actual speed [m/s]
    @param vl:    leading speed [m/s]
    @return:  acceleration [m/s^2]
    '''

    def calcAccInt(self,s,v,vl):

        sstar = self.s0 + np.maximum(0,v*self.T + 0.5*v*(v-vl)/np.sqrt(self.a*self.b))

        accInt = -self.a*math.pow(sstar/np.maximum(s,0.1*self.s0),2)

        #return np.maximum(-self.bmax,accInt)
        return accInt

    
    # Final longitudinal acceleration equation

    def calcAccLong(self,s,v,vl):
        accLong=np.maximum(-self.bmax, self.calcAccFree(v)+self.calcAccInt(s,v,vl) )
        #if self.v0>6.8 and self.v0<7.0:
        if False:
            sstar = self.s0 + np.maximum(0,v*self.T + 0.5*v*(v-vl)/np.sqrt(self.a*self.b))
            print(f' calcAcclong: s=',s,' sstar=',sstar,' accIDM=',accLong)
            
            
        return accLong

def sim(x, data):
  
    v0 = x[0]
    T = x[1]
    s0 = x[2]
    a = x[3]
    b = x[4]
    
    count=0
    GAP_MIN=0.4
    data=data.reset_index()
    dt=0.2
    #print(f'simulate CF pair with v0={v0},T={T},s0={s0},a={a},b={b}')
    
    CF=IDM(v0,T,s0,a,b)    # IDM model as defined above
    
    # convert pandas dataframe to arrays since left-assignment faster

    #x1=np.empty(len(data), dtype=float)
    v1=np.empty(len(data), dtype=float)
    gap1=np.empty(len(data), dtype=float)
    acc1=np.empty(len(data), dtype=float)

    # export data gaps to numpy array to  limit gap to values >=GAP_VAL
    # !!! by reference, also original data['gap[m]'] affected by mainpul gapData
    
    gapData=data['gap[m]'].to_numpy()
    for i in range(0,len(data)):
        gapData[i]=max(GAP_MIN,gapData[i])
                       
    #print(f'FCdata=',FCdata)
    #print(f'gapData=',gapData)

    # initialisation
    
    v1[0]=data.loc[0,'vx[m/s]'] 
    gap1[0]=gapData[0] #max(GAP_MIN,data.loc[0,'gap[m]'])
    acc1[0]=CF.calcAccLong(gap1[0],v1[0],data.loc[0,'lead_vx']) ################ cannot we take teh acceleeraiton for the first one by the dataset itself 

    # simulation
    
    for i in range(1,len(data)):
        #if i<1181:
        if False:
            print(f'\nsim: time step i=',i,' gap1[i-1]=',gap1[i-1],' v1[i-1]=',v1[i-1])

        v1[i]=v1[i-1]+acc1[i-1]*dt 
        
        v_lead=0.5*(data.loc[i-1,'lead_vx'] + data.loc[i,'lead_vx'])
        
        gap1[i]=gap1[i-1]+(v_lead-0.5*(v1[i]+v1[i-1]))*dt
        
        # if estimated speed negative, assume a stop and no further decel
        # (before possible gap reset because gap reset is dominating the actions)
        
        if v1[i]<-1e-6:  # then acc1 strictly<0
            v1[i]=0
            gap1[i]=gap1[i-1]+v_lead*dt -(-0.5*v1[i-1]**2/acc1[i-1])

        # reset gap if new leader
        if data.loc[i,'leadID']!=data.loc[i-1,'leadID']:
            #v1[i]=data.loc[i,'vx[m/s]']  #!! No speed reset!
            gap1[i]=gapData[i] #data.loc[i,'gap[m]']
            #print(f' new leader, i={i}: reset gap1[i]={gap1[i]}, unchanged v={v1[i]}')

         # reset for change in follower gap if new follower
        if data.loc[i,'subj']!=data.loc[i-1,'subj']:
            count+=1
            v1[i]=data.loc[i,'vx[m/s]']  #!! No speed reset!
            gap1[i]=gapData[i] #data.loc[i,'gap[m]']
            #print(f' new leader, i={i}: reset gap1[i]={gap1[i]}, unchanged v={v1[i]}')

        


        acc1[i]=CF.calcAccLong(gap1[i],v1[i],data.loc[i,'lead_vx'])

        if False:
        #if i<10:
            print(f'i={i} sLast={gap1[i-1]} vLast={v1[i-1]} vlLast={data.loc[i-1,"lead_vx"]} acc1[i-1]={acc1[i-1]}')

    # re-convert arrays to dataframe to be consistent
    # (this one-shot conversion is fast)
    
    data['v1']=v1.tolist()
    data['gap1']=gap1.tolist()
    data['acc1']=acc1.tolist()

    sse= sum((data['gap1']-data['gap[m]'])**2)
    sse1= sum((gapData-gap1)**2)
    #avg_error=sum(abs(data['gap1']-data['gap[m]']))/len(data) Ankit: measure MAD
    avg_error=np.sqrt(sse/len(data))
    #print(f'simulated CF pair with v0={v0},T={T},s0={s0},a={a},b={b}, SSE={sse}, sse1={sse1}')
    #print(f'simulated CF pair {data["subj"].unique()} with v0={v0},T={T},s0={s0},a={a},b={b}, SSE={sse}',count)
    return  avg_error

def denormparams(x1):
    x0=[0,0,0,0,0]
    x0[0]=np.minimum(np.maximum((x1[0]*400+3),3),400)
    x0[1]=np.minimum(np.maximum((x1[1]*10+0.1),0.1),10)
    x0[2]=np.minimum(np.maximum((x1[2]*20+0.4),0.4),20)
    x0[3]=np.minimum(np.maximum((x1[3]*9+0.1),0.1),9)
    x0[4]=np.minimum(np.maximum((x1[4]*9+0.1),0.1),9)

    return x0

def calib_pooled(x,data):
    x0=denormparams(x)
    v0=np.minimum(np.maximum(x0[0],3),400)
    T=np.minimum(np.maximum(x0[1],0.01),10)
    s0=np.minimum(np.maximum(x0[2],0.4),20)
    a=np.minimum(np.maximum(x0[3],0.1),9)
    b=np.minimum(np.maximum(x0[4],0.1),9)
    return sim(v0,T,s0,a,b,data)[0]


def optim(x, data):
    
    try:
        # Optim function to be used in parallelization using Nelder-Mead method
        res = minimize(sim, x, (data), method='Nelder-Mead')
        return res.x
    except OverflowError as e:
        print(f"OverflowError encountered: {e}. Skipping this iteration.")
        return None  # You can choose what to return if an error occurs.
