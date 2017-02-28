# -*- coding: utf-8 -*-
"""
Created on Thu Feb 16 18:38:02 2017

@author: Aaron
"""

import heapq as hq
import CarSimDefSimple as Sim

OpenSet = []
hq.heapify(OpenSet)
ClosedSet = []
totalDistance = 1000#2570.5 * 6
totalTime = 2300 #2300 #in sec

class Gnode:
    def __init__(self,TimeSec,DistM):
        self.Name = str(TimeSec)+str(DistM)
        self.TimeSec = TimeSec
        self.DistM = DistM
        self.CostWeightF = 0 #grams of fuel
        self.CostToPointG = 0 #grams of fuel
        self.HeuristicCostH = 0 #grams of fuel
        self.Parent = None
        self.Neighbors = []
        self.EngineOn = False
        self.Velocity = 0 #m/s
        self.Accel = 0
        
    def calcF(self):
        self.CostWeightF = self.CostToPointG + self.HeuristicCostH
        
def addNeighbors(node):
    Tstep = 1
    if node.TimeSec + Tstep <= totalTime:
        timeStep = node.TimeSec + Tstep
    elif node.TimeSec + Tstep > totalTime:
        timeStep = totalTime - node.TimeSec
    
    #get the starteng status
    if node.EngineOn:
        startEng = False
    else:
        startEng = True
        
    wot = Sim.SimNodeToNode(True,node.DistM,node.Velocity,node.Accel,node.TimeSec,timeStep,startEng)
    coast = Sim.SimNodeToNode(False,node.DistM,node.Velocity,node.Accel,node.TimeSec,timeStep,False)
    if wot[1] > node.DistM: 
        temp = Gnode(node.TimeSec + Tstep, wot[1])
        temp.CostToPointG = node.CostToPointG + wot[0]
        temp.Parent = node
        temp.Velocity = wot[2]
        temp.Accel = wot[3]
        temp.EngineOn = True
        temp.HeuristicCostH = get_heuristic(wot[1], totalDistance)
        temp.calcF()
        hq.heappush(OpenSet, (temp.CostWeightF, temp))
           
    if coast[1] > node.DistM:
        temp = Gnode(node.TimeSec + Tstep, coast[1])
        temp.CostToPointG = node.CostToPointG+coast[0]
        temp.Parent = node
        temp.Velocity = coast[2]
        temp.Accel = coast[3]
        temp.EngineOn = False
        temp.HeuristicCostH = get_heuristic(coast[1], totalDistance)
        temp.calcF()
        hq.heappush(OpenSet, (temp.CostWeightF, temp))

def get_heuristic(DistM,EndDistM):
    return (1/1062859.27) * 691.92 * (EndDistM-DistM)
    #use world record of 2500 mpg prorated over the distance
    #use Isooctance density to get grams, 691.92 g/L
    
def get_path(node):
    path = [node]
    while node.Parent.DistM > 0:
        node = node.Parent
        path.append(node)
    node = node.Parent
    path.append(node)
    path.reverse()     
    return path

def gramtompg(grams, meters):
    return ((meters/1000)/(grams/691.92)) * 2.35215 #take km/L to US mpg
    
#The guts
#make open set a list of list with main item Fcost and second node object
#put create start node place in openset
#gen neighbors and assign Weights
#pop start into closed
#pop next lowest
#
def SolveThis():
    TDtup = []
    start = Gnode(0,0)
    start.HeuristicCostH = get_heuristic(start.DistM,totalDistance)
    start.calcF
    hq.heappush(OpenSet, (start.CostWeightF, start))
    while len(OpenSet):
        f, node = hq.heappop(OpenSet)
        print(node.DistM)
        TDtup.append((node.TimeSec, node.DistM, node.CostToPointG))
        if node.DistM >= totalDistance:
            path = get_path(node)
            return path, TDtup
        addNeighbors(node)
        
path, TD = SolveThis()
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.mlab import griddata
from matplotlib import cm
import matplotlib.pyplot as plt
import numpy as np

T,D,f = zip(*TD)
cm = plt.cm.get_cmap('coolwarm')
fig = plt.figure()
sc = plt.scatter(T,D,c=f,cmap=cm)
plt.colorbar(sc)
plt.show()