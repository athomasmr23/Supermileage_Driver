# -*- coding: utf-8 -*-
"""
Created on Sat Feb 18 10:57:13 2017

@author: Aaron
"""
import scipy
from scipy import interpolate
import math

def torque(RPM):
    #return -3E-7*math.pow(RPM,2)+0.0018*RPM-1.6718 #N*m, torque as a function of RPM at full throttle
    return -4e-15*math.pow(RPM,4)+8e-11*math.pow(RPM,3)-6e-7*math.pow(RPM,2)+0.0019*RPM
                          
def BSFC(RPM):
    #return 12.5*math.pow(kW,3)-74.583*math.pow(kW,2)+131.33*kW+255.6 #brake specific fuel consumption as a function of kW
    return 4e-5*math.pow(RPM,2)-0.3458*RPM+1180.9
                         
def grade(CurLapDist,gradeDistance,gradeDegrees): #take sind of output for forces of grade
    temp=scipy.interpolate.pchip_interpolate(gradeDistance,gradeDegrees,CurLapDist)
    return float(temp)

def SimNodeToNode(EngineOn,x1,v1,a1,StartT,EndT,startEng):
    #----------------Engine, transmission, and fuel Parameters-----------------
    
    RPMMax = 5000
    fuelDensity = 691.92 #g/L density of isooctane fuel
    
    ratio = (60/13)*(60/16)
    r = [ratio, ratio/1.37]#/1.37] #transmission gear ratio
    vShift = 10*0.44704 #speed at which the hub shifts gears 
    eff = .9 #transmission efficiency
    clutchEngage = 1800 #RPM when clutch engages
    
    gramsPerStart = 0.07349 #grams used to start the engine
    startTime = 4.0 #seconds, cranking time to start the engine 
    
    #----------------------------Vehicle Parameters----------------------------
    m = 115.172 #kg, vehicle mass
    k = 0.36 #rear weight bias
    h = .3 #m, center of gravity height
    pr = 1.0 #rear power bias
    l = 1.47 #m, wheelbase
    crr = 0.005577 #rolling resistance coefficienct
    cd =  .386718 #.09*1.5 #drag coefficient
    A = .608 #m^2, frontal area
    cpx = 0 #m, horizontal location of of center of pressure, with the cg being (0,0)
    cpy = 0 #m, vertical location of of center of pressure, with the cg being (0,0)
    mu = 0.8 #friction coefficient of tires
    LDR = 0 #lift to drag ratio
    d = .464 #m, wheel diameter
    
    #------------------------Environment Parameters----------------------------
    rho = 1.162 #kg/m^2,air density average for marshall MI for June
    g = 9.804 #m/s^s, gravitational acceleration
    
    #--------------------Simulation Parameters---------------------------------
    dt = .1 #seconds, timestep for simulation, smaller means better resolution, but slower computation time
    nPre = 6 #digits of preision for return
    t = []
    for te in range(int(StartT),int((EndT/dt))):
        t.append(round(te*dt,8))
    
    #----------------------Track Variables-------------------------------------
    lapDistance = 2570.5      #m, length of one lap
    totalDistance = lapDistance * 6
    gradeDegrees = [-0.00120873685628604, -0.00149795359527380,
                    -0.00175680943619625, -0.00190480228574180,
                    -0.00259249356266746, -0.000897653618127095,
                     0.000698423444691644, 0.00985107554879941,
                     0.0104933487982877,   0.00949959502756691,
                     0.00956020813438021,  0.00939886399449551,
                    -0.0109041227784944,  -0.0601995006291671,
                    -0.0715213868288333,  -0.0756675294879319,
                    -0.0705252260348547,  -0.0570125839260598,
                    -0.0209633822544938,  -0.000600138421448646,
                     0.000317134303300928,-0.000501602298742393]
    gradeDistance = [ 105.908856000000,  211.717128000000,
                      317.549784000000,  453.563736000000,
                      574.660776000000,  798.764976000000,
                     1012.60656000000,  1195.15128000000,
                     1316.27270400000,  1482.47100000000,
                     1633.58779200000,  1784.70458400000,
                     1860.45348000000,  1875.67519200000,
                     1890.86337600000,  1906.06070400000,
                     1921.29156000000,  1936.56508800000,
                     1951.82947200000,  2149.90375200000,
                     2351.73621600000,  2570.49117600000]
          
    #--------------------Kinematic Veriables-----------------------------------
    x = [] #meters, Position
    v = [] #m/s, Velocity
    a = [] #m/s^2, Acceleration
    GramFuel = []
    
    x.append(x1)
    a.append(a1)
    if v1 > 0:
        v.append(v1)
    else:
        v.append(0.000001)
    
    #---------------------------Simulation-------------------------------------
    Nf = m*g*(1-k)
    Nr = m*g*k   
    
    for i in range(0,len(t)):
        if x[i] < lapDistance:
            currentLapDistance = x[i]
        else:
            currentLapDistance = x[i]%lapDistance
        fGrade = m*g*math.sin(math.radians(grade(currentLapDistance,gradeDistance,gradeDegrees))) 
        Frrf = crr*Nf
        Frrr = crr*Nr
        fDrag = 0.5*A*cd*rho*math.pow(v[i],2)
        
        if EngineOn:

            if v[i] < vShift: 
               RPM = ((v[i]*60.0)/(math.pi*d))*r[0]
               RPM = min(RPM, RPMMax)
               RPM = max(RPM, clutchEngage)
               ft = torque(RPM) * eff * r[0]
               #print(RPM, ", ", ft)
            else:
               RPM = ((v[i]*60)/(math.pi*d))*r[1]
               RPM = min(RPM, RPMMax)
               RPM = max(RPM, clutchEngage)
               ft = torque(RPM) * eff * r[1]
               print(RPM)

           
            Fxf = ft*(1-pr)/(.5*d)
            Fxr = ft*pr/(.5*d)
            FxfMax = mu*Nf
            FxrMax = mu*Nr
            Ftf = min(FxfMax,Fxf)
            Ftr = min(FxrMax, Fxr)
            if RPM >= RPMMax and fGrade <0:
               a.append(-fGrade/m)
            elif RPM >= RPMMax:
               a.append(0)
            else:
               a.append((Ftf+Ftr-Frrf-Frrr-fDrag-fGrade)/m)
            v.append(v[i]+a[i+1]*dt)
            x.append(x[i]+v[i+1]*dt+a[i+1]*math.pow(dt,2))
            Nr = m*g*k+(Ftf+Ftr-Frrf-Frrr)*(h/l)+fDrag*LDR*((cpx/l)-k)
            Nf = m*g-fDrag*LDR-Nr
            power = torque(RPM)*RPM/9.5488/1000.0 #power used in kW
            gramsPerSecond = BSFC(RPM)*power/3600
            if startEng:
                GramFuel.append((gramsPerSecond*dt) + gramsPerStart)
            else:
                GramFuel.append(gramsPerSecond*dt)
        
        if not EngineOn:
           ft = 0
           Fxf = ft*(1-pr)/(.5*d)
           Fxr = ft*pr/(.5*d)
           FxfMax = mu*Nf
           FxrMax = mu*Nr
           Ftf = min(FxfMax,Fxf)
           Ftr = min(FxrMax, Fxr)
           a.append((Ftf+Ftr-Frrf-Frrr-fDrag-fGrade)/m)
           v.append(v[i]+a[i+1]*dt)
           x.append(x[i]+v[i+1]*dt+a[i+1]*math.pow(dt,2))
           Nr = m*g*k+(Ftf+Ftr-Frrf-Frrr)*(h/l)+fDrag*LDR*((cpx/l)-k)
           Nf = m*g-fDrag*LDR-Nr
    return [round(sum(GramFuel),nPre),round(x[-1],nPre),round(v[-1],nPre),round(a[-1],nPre)]
        
#StartDis = 0
#q = SimNodeToNode(False,StartDis,2,0,0,1)
#print(q)
#
#mpg=(q[1]-StartDis)*0.000621371/((q[0]/691.92)*0.264172)
#print(mpg)