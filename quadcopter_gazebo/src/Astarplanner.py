#!/usr/bin/env python3
import numpy as np
import math
from nav_msgs.msg import OccupancyGrid


class Astar:
    def __init__(self,resolution,robotradius):
        
        self.map = np.ones((120,120))*(-1)
        self.resolution = resolution
        self.rr = robotradius
        self.Rd = robotradius + 0.25
        self.collisioncostmap = np.zeros((self.map.shape))
        self.krep = 5


    def generategridmap(self,mapmsg):
        self.map = np.asarray(mapmsg.data)

        self.map = np.reshape(self.map,(math.sqrt(self.map.shape),math.sqrt(self.map.shape)))

    

    def updatecollisioncostmap(self):

        for i in range(self.map.shape[0]):
            for j in range(self.map.shape[1]):
                if self.map[i,j]==100:
                    for k in range(math.ceil(2*self.Rd/self.resolution)):
                        for l in range(math.ceil(2*self.Rd/self.resolution)):
                            currentobstaclecell = np.array([i,j])
                            othercell = np.array([max(0,i-math.ceil(self.Rd/self.resolution))+k,max(0,j-math.ceil(self.Rd/self.resolution))+l])
                            distancebtwcells = self.resolution*np.linalg.norm(currentobstaclecell-othercell)
                            if distancebtwcells < self.rr:
                                self.collisioncostmap[othercell[0], othercell[1]] = max(self.collisioncostmap[othercell[0],\
                                othercell[1]],math.inf)
                            elif distancebtwcells < self.Rd:
                                self.collisioncostmap[othercell[0], othercell[1]] = max(self.collisioncostmap[othercell[0], othercell[1]],\
                                self.krep*((self.Rd - distancebtwcells)/(self.Rd - self.rr))**2)

        

    #def endpointsingridframe(self, goal):



    def astarplanner(self, startpose, goal, mapmsg):
        self.generategridmap(mapmsg)
        self.updatecollisioncostmap()

        self.fcost = np.ones([self.map.shape])*math.inf  # Astar f costmap initialisation : f = g + h
        self.gcost = np.ones([self.map.shape])*math.inf  # Astar g costmap initialisation : g the distance from start to current node
        iterationnodes = []
        self.gcost[0,self.map.shape/2-1] = 0
        self.fcost[0,self.map.shape/2-1] = self.gcost[0,self.map.shape/2-1] + np.linalg.norm(np.array([0,self.map.shape/2-1]),goal) \
                                                                                            #euclidean distance heuristic
        iterationnodes = [np.array([0,self.map.shape/2-1],self.fcost[0,self.map.shape/2-1])]

        while len(iterationnodes)!=0 :

            currentnode = iterationnodes.index(min())

