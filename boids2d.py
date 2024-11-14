#! /usr/bin/env python3

# Author: Liam Hainsworth
# Date Created: March 12, 2021
#
# Description: A basic 2D implementation of the boids flocking algorithm with capability for
#       real-time adjustment of algorithm parameters through a series of sliders. https://en.wikipedia.org/wiki/Boids
#
# Requires tkinter to be installed
#

import tkinter as tk
from tkinter import ttk
import random
import math

# Object representing a single boid.
#
# Each object several main components:
# - a position vector representing their position on the screen
# - a heading vector representing the direction they are travelling with a magnitude of
#   their velocity
# - a set of parameter values governing their interaction with other boid objects, namely
#   the weights of the separation, alignment and coherence operations as well as the distance
#   at which they are capable of sensing other boid objects
class boid2D():
    def __init__(self, pos, idnum, \
                 senserange_ = 1, separate_ = 1, align_ = 1, cohere_ = 1, velocity_ = 1):
        global senserange
        senserange = senserange_
        global separate
        separate = separate_
        global align
        align = align_
        global cohere
        cohere = cohere_
        global velocity
        velocity = velocity_
        global noise
        noise = 1
        self.pos = pos
        self.idnum = idnum
        phi = random.random() * 360.0
        self.heading = [math.cos(phi)*velocity,
                        math.sin(phi)*velocity]
        global bounce
        bounce = False
        global root_sep
        root_sep = True
        
    # Find any boids within sensing range
    def findnear(self, boidarr):
        global senserange
        subarr = []
        for boid in boidarr:
            # Rough (but fast) check
            if math.fabs(boid.pos[0] - self.pos[0]) < senserange and \
               math.fabs(boid.pos[1] - self.pos[1]) < senserange:
                # Actual check
                if math.sqrt((boid.pos[0]-self.pos[0])**2 + \
                           (boid.pos[1]-self.pos[1])**2) < senserange:
                    if self.idnum != boid.idnum:
                        subarr.append(boid)
        return subarr
    
    # Adjusts the heading of a boid using the properties of nearby boids
    #
    # Heading adjustment uses the follwing factors:
    #   anticrowd is the average of the square root of how far into the sensing range other boids are
    #   meanpos is the mean position of other boids in sensing range
    #   meanhead is the mean heading of other boids in sensing range
    # The heading of the boid is then adjusted using these:
    #   away from the anticrowd vector (multiplied by the separate scaling factor)
    #   towards the meanpos vector (multiplied by the cohere scaling factor)
    #   towards the meanhead vector (multiplied by the align scaling factor)
    #
    # In the more general sense of the boids algorithm, the anticrowd vector serves the purpose of
    # maintaining separation between boids, the meanpos vector maintains coherence of a group of boids
    # and the meanhead vector maintains the alignment of a group's headings
    def adjustheading(self, guideboids):
        global separate, align, cohere, velocity, senserange, root_sep
        if len(guideboids) == 0:
            pass
        else:
            meanpos = [0,0]
            meanhead = [0,0]
            anticrowd = [0,0]
            for boid in guideboids:
                for i in range(2):
                    meanpos[i] = meanpos[i] + boid.pos[i]
                    meanhead[i] = meanhead[i] + boid.heading[i]
                    if root_sep:
                        if boid.pos[i] < self.pos[i]:
                            anticrowd[i] = anticrowd[i] - \
                                math.sqrt(senserange-math.fabs(self.pos[i]-boid.pos[i]))
                        else:
                            anticrowd[i] = anticrowd[i] + \
                                math.sqrt(senserange-math.fabs(boid.pos[i]-self.pos[i]))
                    else:
                        anticrowd[i] = anticrowd[i] - \
                                (senserange-math.fabs(self.pos[i]-boid.pos[i]))**2

            # Convert from sums to mean values
            for i in range(2):
                meanpos[i] = meanpos[i]/len(guideboids)
                meanhead[i] = meanhead[i]/len(guideboids)
                anticrowd[i] = anticrowd[i]/len(guideboids)

            # Adjust heading using calculated values
            for i in range(2):
                self.heading[i] = self.heading[i] - \
                    separate * anticrowd[i] + \
                    cohere * (meanpos[i] - self.pos[i]) + \
                    align * meanhead[i]                    

        # Normalize heading vector and adjust to set velocity (prevent acceleration)
        magnitude = math.sqrt(self.heading[0]**2 + self.heading[1]**2)
        if magnitude == 0:
            self.heading[0] = 0
            self.heading[1] = 0
        else:
            for i in range(2):
                self.heading[i] = self.heading[i]*velocity/magnitude   

    # Move based on heading
    def movetick(self, bound, multiplier = 1, addnoise = False):
        for i in range(2):
            self.pos[i] = self.pos[i] + self.heading[i]
            # add 
            if addnoise:
                global noise
                self.heading[i] = self.heading[i] + \
                    random.random()*noise - noise/2
            global bounce
            if bounce:
                if self.pos[i] > bound or self.pos[i] < 0:
                    self.heading[i] = -self.heading[i]
            else:
                if self.pos[i] > bound:
                    self.pos[i] = self.pos[i] - bound
                elif self.pos[i] < 0:
                    self.pos[i] = bound - self.pos[i]

class boidrender():
    def __init__(self, bounds = 1000, boids = 50, interval = 20):
        self.win = tk.Tk()
        self.win.title("Boids2D")
        self.interval = interval
        self.bounds = bounds
        self.canvas = tk.Canvas(self.win, width = bounds, height = bounds, background="#ffffff")
        self.defaultparams = {"senserange":50, "separate":4.5, "align":2, "cohere":0.7, \
                              "velocity":5, "noise":1}
        self.boidarr = [boid2D([random.random()*bounds, random.random()*bounds], i, \
                               self.defaultparams["senserange"], \
                               self.defaultparams["separate"], \
                               self.defaultparams["align"], \
                               self.defaultparams["cohere"], \
                               self.defaultparams["velocity"]) for i in range(boids)]
        global showsight
        showsight = False

    # Sets up the window environment
    def setup(self):
        self.canvas.grid(rowspan = 6)
        global cohere, align, separate, noise, velocity, senserange

        # Coherence slider
        self.sc = tk.Scale(self.win, label = str("Coherence Weight"),\
                      to = 5.0, resolution = 0.01, command = self.setcohere,\
                      tickinterval = 1, length = 250)
        self.sc.set(cohere)
        self.sc.grid(column=1, row=1, sticky=tk.W)

        # Separation slider
        global root_sep
        if root_sep:
            sepres = 0.1
            sepmax = 100.0
            sepint = 20
        else:
            sepres = 0.01
            sepmax = 1
            sepint = 0.1
        self.ss = tk.Scale(self.win, label = "Separation Weight",\
                      to = 100.0, resolution = sepres, command = self.setseparate,\
                      tickinterval = 20, length = 250)
        self.ss.set(separate)
        self.ss.grid(column=2, row=0, sticky=tk.W)

        # Alignment slider
        self.sa = tk.Scale(self.win, label = "Alignment Weight",\
                      to = 40.0, resolution = 0.1, command = self.setalign,\
                      tickinterval = 10, length = 250)
        self.sa.set(align)
        self.sa.grid(column=1, row=0, sticky=tk.W)

        # Noise slider
        self.sn = tk.Scale(self.win, label = "Noise Amount",\
                      to = 40.0, resolution = 0.1, command = self.setnoise,\
                      tickinterval = 10, length = 250)
        self.sn.set(noise)
        self.sn.grid(column=1,row=2, sticky=tk.W)

        # Velocity slider
        self.sv = tk.Scale(self.win, label = "Velocity",\
                      to = 100.0, resolution = 0.1, command = self.setvelocity,\
                      tickinterval = 20, length = 250)
        self.sv.set(velocity)
        self.sv.grid(column=2,row=1, sticky=tk.W)

        # Sensing range slider
        self.sr = tk.Scale(self.win, label = "Sensing Range",\
                      to = 100.0, resolution = 0.1, command = self.setrange,\
                      tickinterval = 20, length = 250)
        self.sr.set(senserange)
        self.sr.grid(column=2,row=2, sticky=tk.W)

        # Checkbox to show sensing ranges
        self.sb = tk.Checkbutton(command = self.updatesight, text="Show Sensing Ranges")
        self.sb.grid(column=2,row=3, sticky=tk.NW)

        # Checkbox to make any individual boid bounce rather than teleport at an edge
        self.bb = tk.Checkbutton(command = self.updatebounce, text="Edge Bounce (Unimpressive)")
        self.bb.grid(column=1,row=3, sticky=tk.NW)

        # Button to reset slider values
        self.rb = tk.Button(command = self.reset, text="Reset Paramaters to defaults")
        self.rb.grid(column=1,row=4,columnspan=2, sticky=tk.NW+tk.E)

    # Start simulation
    def start(self):
        self.setup()
        self.update()
        self.win.mainloop()

    # Reset slider values
    def reset(self):
        self.sc.set(self.defaultparams["cohere"])
        self.ss.set(self.defaultparams["separate"])
        self.sa.set(self.defaultparams["align"])
        self.sn.set(self.defaultparams["noise"])
        self.sv.set(self.defaultparams["velocity"])
        self.sr.set(self.defaultparams["senserange"])
        
    # Update method for sensing range
    def updatesight(self):
        global showsight
        showsight = not(showsight)
#        print(showsight)

    # Update method for edge bouncing 
    def updatebounce(self):
        global bounce
        bounce = not(bounce)
#        print(bounce)

    # Update method for coherence slider
    def setcohere(self, newval):
        global cohere
        cohere = float(newval)

    # Update method for separation slider
    def setseparate(self, newval):
        global separate
        separate = float(newval)

    # Update method for alignment slider
    def setalign(self, newval):
        global align
        align = float(newval)

    # Update method for noise slider
    def setnoise(self, newval):
        global noise
        noise = float(newval)

    # Update method for velocity slider
    def setvelocity(self, newval):
        global velocity
        velocity = float(newval)

    # Update method for sensing range slider
    def setrange(self, newval):
        global senserange
        senserange = float(newval)

    # Redraw canvas on new tick
    def update(self):
        # clear canvas
        self.canvas.delete('all')
        
        global showsight,senserange
        for boid in self.boidarr:
            # draw individual boid
            self.canvas.create_oval(boid.pos[0]-5, boid.pos[1]-5, boid.pos[0]+5, boid.pos[1]+5)
            if showsight:
                # draw boid sensing range
                self.canvas.create_oval(boid.pos[0]-senserange, \
                                        boid.pos[1]-senserange, \
                                        boid.pos[0]+senserange, \
                                        boid.pos[1]+senserange, \
                                        outline="#ff0000")
            # update boid headings
            boid.adjustheading(boid.findnear(self.boidarr))
        # update boid positions
        for boid in self.boidarr:
            boid.movetick(self.bounds, 0.5, True)
        # set up next tick
        self.win.after(self.interval, self.update)

renderer = boidrender()
renderer.start()
        
            
            
