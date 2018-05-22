# Final Project for CompSci Intro to Programming
#   by zoddisa

# Moon is stationary at position [0,0]
# Display:  Main window is oriented such that 'down' is towards [0,0].

from graphics import *
from math import sqrt, acos, asin, log, e, pi, sin, cos, hypot, copysign
from decimal import *
from time import sleep

class LEM:

    def __init__(self, xpos, ypos, xvel, yvel, phi, rot, rotationalAcceleration,
                 fuel, drymass, gravity, thrust, engineFuelUse, radiusMoon):
        """This creates a lunar module object, which handles the nongraphical
        mathematical representation of the lander."""

        self.xpos = xpos
        self.ypos = ypos
        self.xvel = xvel
        self.yvel = yvel
        self.velocity = sqrt(self.xvel*self.xvel + self.yvel*self.yvel)

        self.phi = phi                                      #phi is the angle of rotation of the craft on the coordinate system, 0 is in standard position
        hypotenuse = sqrt(self.xpos*self.xpos + self.ypos*self.ypos)
        adjacent = self.xpos
        self.theta = self.phi - acos(adjacent/hypotenuse)   #theta is angle of craft with respect to ground, ground is at -pi/2
        self.rot = rot
        self.rotAcc = rotationalAcceleration

        self.throttle = 0       #range is 0.0 to 1.0
        self.fuel = fuel        #mass of fuel
        self.fueled = 1         #becomes 0 if fuel is empty
        self.dryMass = drymass  #mass of vehicle without fuel
        self.engineOn = 0       #0 or 1

        self.isp = thrust/engineFuelUse
        self.thrust = thrust
        self.engineFuelUse = engineFuelUse
        self.deltaV = self.isp*log((self.fuel + self.dryMass)/self.dryMass,e)

        self.radiusMoon = radiusMoon
        self.gravConst = (gravity*radiusMoon*radiusMoon)    #grav is surf gravity, gconst such that inverse square law applies
        self.altitude = (self.xpos*self.xpos + self.ypos*self.ypos) - self.radiusMoon

    def refresh(self, timeUpdate):
        """This will calculate time seconds into the future
        and update the variables of the LEM accordingly."""

        # calculates accelerations
        thrustAcc = self.throttle * self.thrust * self.engineOn * self.fueled / (self.fuel + self.dryMass)

        xgrav = (-self.xpos * self.gravConst) / pow(hypot(self.xpos, self.ypos),3)
        ygrav = (-self.ypos * self.gravConst) / pow(hypot(self.xpos, self.ypos),3)

        xthrust = thrustAcc * cos(self.phi)
        ythrust = thrustAcc * sin(self.phi)

        xacc = xgrav + xthrust
        yacc = ygrav + ythrust

        #updates position
        self.xpos = self.xpos + self.xvel*timeUpdate + .5*xacc*pow(timeUpdate,2)
        self.ypos = self.ypos + self.yvel*timeUpdate + .5*yacc*pow(timeUpdate,2)
        #print(sqrt(self.xpos*self.xpos + self.ypos*self.ypos))

        #updates velocity
        self.xvel = self.xvel + xacc*timeUpdate
        self.yvel = self.yvel + yacc*timeUpdate
        self.velocity = sqrt(self.xvel*self.xvel + self.yvel*self.yvel)

        #updates rotation
        self.phi = self.phi + self.rot
        self.rot = self.rot + (0.005)*pi*self.rotAcc
        self.rotAcc = 0

        #updates fuel & deltav
        self.fuel = self.fuel - timeUpdate*self.engineFuelUse*self.engineOn*self.throttle*self.fueled
        self.deltaV = self.isp*log((self.fuel + self.dryMass)/self.dryMass,e)

        if self.fuel <= 0:
            self.fueled = 0

        #updates altitude
        self.altitude = hypot(self.xpos, self.ypos) - self.radiusMoon

    #def getXVel(self):
    #    return self.xvel

    #def getYVel(self):
    #    return self.yvel

    def getDisplayVariables(self):
        """Returns a lot of variables at once, and calculates a few more."""
        hypotenuse = sqrt(self.xpos*self.xpos + self.ypos*self.ypos)
        adjacent = self.ypos
        self.omega = pi*(1/2)  - acos(adjacent/hypotenuse)  #omega is the angle of the position of the craft relative to the origin
        self.theta = self.phi + acos(adjacent/hypotenuse)   #gives angle of craft with respect to ground

        #linear transformation of x/y to h/v coordinate system
        A = cos(self.omega)
        B = sin(self.omega)
        hvel = self.xvel * B - self.yvel * A
        vvel = self.xvel * A + self.yvel * B

        return self.fueled, self.omega, self.theta, self.throttle, self.engineOn, self.xpos, self.ypos, self.xvel, self.yvel, hvel, vvel, self.fuel, self.deltaV, self.altitude, self.velocity

    def keyProcess(self, key):
        """Processes keypresses and updates LEM accordingly"""
        if key == 'a':
            self.rotAcc = 1
        elif key == 'd':
            self.rotAcc = -1
        elif key == 'e':
            self.engineOn = 1
        elif key == 'q':
            self.engineOn = 0
        elif key == 'w':
            if self.throttle < .9:
                self.throttle = self.throttle + .2
            else:
                self.throttle = 1   #here to avoid floating point errors
        elif key == 's':
            if self.throttle > .1:
                self.throttle = self.throttle - .2
            else:
                self.throttle = 0   #here to avoid floating point errors

class LanderGraphics:

    def __init__(self, win, theta, altitude, hvel, vvel):     #theta is rotation of craft with respect to ground
        """Creates a graphical representation of the Lunar Module
        oriented to the ground according to theta (in radians).
        Also draws throttle handle."""

        self.win = win
        self.theta = theta
        self.omega = 0
        self.throttle = 0
        self.engineOn = 0
        self.fueled = 1
        self.altitude = altitude
        self.lowAltitude = False
        if self.altitude <= 500:
            self.lowAltitude = True
        self.hvel = hvel
        self.vvel = vvel

        #draws landscape objects out of view
        horizon = Rectangle(Point(-1,-1),Point(-2,-2))
        horizon.draw(self.win)
        self.horizon = horizon
        mid = Rectangle(Point(-1,-1),Point(-2,-2))
        mid.draw(self.win)
        self.mid = mid
        land = Rectangle(Point(-1,-1),Point(-2,-2))
        land.draw(self.win)
        self.land = land

        #draws handle of throttle in 0 position
        self.handle = Rectangle(Point(26,.8), Point(27,1.2))
        self.handle.setFill('black')
        self.handle.draw(self.win)

        #draws lander graphical representation
        # r = 2.5 = 5m
        # thus on display window, each coordinate point is equivalent to 2 meters.
        r = 2.236
        t = 0.463
        px = r*cos(self.theta + t) + 12.5
        py = r*sin(self.theta + t) + 12.5
        p1 = Point(px, py)
        px = r*cos(self.theta + -t) + 12.5
        py = r*sin(self.theta + -t) + 12.5
        p14 = Point(px,py)

        r = 2.236
        t = 1.107
        px = r*cos(self.theta + t) + 12.5
        py = r*sin(self.theta + t) + 12.5
        p2 = Point(px, py)
        px = r*cos(self.theta + -t) + 12.5
        py = r*sin(self.theta + -t) + 12.5
        p13 = Point(px,py)

        r = 2
        t = 1.571
        px = r*cos(self.theta + t) + 12.5
        py = r*sin(self.theta + t) + 12.5
        p3 = Point(px, py)
        px = r*cos(self.theta + -t) + 12.5
        py = r*sin(self.theta + -t) + 12.5
        p12 = Point(px,py)

        r = 1.118
        t = 2.034
        px = r*cos(self.theta + t) + 12.5
        py = r*sin(self.theta + t) + 12.5
        p4 = Point(px, py)
        px = r*cos(self.theta + -t) + 12.5
        py = r*sin(self.theta + -t) + 12.5
        p11 = Point(px,py)

        r = 2.062
        t = 1.816
        px = r*cos(self.theta + t) + 12.5
        py = r*sin(self.theta + t) + 12.5
        p5 = Point(px, py)
        px = r*cos(self.theta + -t) + 12.5
        py = r*sin(self.theta + -t) + 12.5
        p10 = Point(px,py)

        r = 5
        t = 2.214
        px = r*cos(self.theta + t) + 12.5
        py = r*sin(self.theta + t) + 12.5
        p6 = Point(px, py)
        px = r*cos(self.theta + -t) + 12.5
        py = r*sin(self.theta + -t) + 12.5
        p9 = Point(px,py)

        r = 2.5
        t = 2.14
        px = r*cos(self.theta + t) + 12.5
        py = r*sin(self.theta + t) + 12.5
        p7 = Point(px, py)
        px = r*cos(self.theta + -t) + 12.5
        py = r*sin(self.theta + -t) + 12.5
        p8 = Point(px,py)

        self.lander = Polygon(p1,p2,p3,p4,p5,p6,p7,p8,p9,p10,p11,p12,p13,p14)
        self.lander.setFill('grey')
        self.lander.setOutline('grey')
        self.lander.draw(self.win)

        #draws exhaust plume

        r = 2.236
        t = 2.678
        px = r*cos(self.theta + t) + 12.5
        py = r*sin(self.theta + t) + 12.5
        p1 = Point(px, py)
        px = r*cos(self.theta + -t) + 12.5
        py = r*sin(self.theta + -t) + 12.5
        p3 = Point(px,py)

        r = 3*self.throttle*self.engineOn
        t = 3.142
        px = r*cos(self.theta + t) + 12.5
        py = r*sin(self.theta + t) + 12.5
        p2 = Point(px, py)

        self.exhaust = Polygon(p1,p2,p3)
        self.exhaust.setFill('yellow')
        self.exhaust.draw(self.win)

        #draws prograde vector
        alpha = acos(self.hvel/hypot(self.hvel,self.vvel))
        alpha = copysign(alpha,self.vvel)
        r = 4
        
        px = r * cos(alpha) + 12.5
        py = r * sin(alpha) + 12.5
        p1 = Point(px,py)

        r = 1
        px = r * cos(alpha+(pi*(1/32))) + 12.5
        py = r * sin(alpha+(pi*(1/32))) + 12.5
        p2 = Point(px,py)

        px = r * cos(alpha-(pi*(1/32))) + 12.5
        py = r * sin(alpha-(pi*(1/32))) + 12.5
        p3 = Point(px,py)

        self.vector = Polygon(p1,p2,p3)
        self.vector.draw(self.win)

    def newPos(self, theta, throttle, engineOn, fueled, altitude, hvel, vvel):
        """Reorients the Lunar Module according to theta and
        displays exhaust according to throttle, if engine is on.
        Theta is defined as the angle of the craft with respect
        to the ground. Also updates throttle handle."""

        self.theta = theta
        self.throttle = throttle
        self.engineOn = engineOn
        self.altitude = altitude
        self.fueled = fueled
        self.hvel = hvel
        self.vvel = vvel

        #draws handle of throttle in appropriate position
        p1 = 26
        p2 = 6*throttle + 0.8
        p3 = 27
        p4 = 6*throttle + 1.2
        self.handle.undraw()
        handle = Rectangle(Point(p1,p2), Point(p3,p4))
        handle.setFill('black')
        self.handle.undraw()
        handle.draw(self.win)
        self.handle = handle

        if self.lowAltitude == False:
            if self.altitude <= 500:
                self.lowAltitude = True
        else:
            #draws horizon
            horizonY = 11 - 11*self.altitude/500
            horizon = Rectangle(Point(0,0),Point(25,horizonY))
            horizon.setFill('darkgrey')
            horizon.setOutline('darkgrey')
            self.horizon.undraw()
            horizon.draw(self.win)
            self.horizon = horizon

            #draws mid 'horizon'
            midY = 10.5 - 10.5*self.altitude/200
            mid = Rectangle(Point(0,0),Point(25,midY))
            mid.setFill('lightgrey')
            mid.setOutline('lightgrey')
            self.mid.undraw()
            mid.draw(self.win)
            self.mid = mid

            #draws land
            landY = 9.5 - 9.5*((self.altitude-2.5)/25)
            land = Rectangle(Point(0,0),Point(25,landY))
            land.setFill('whitesmoke')
            land.setOutline('whitesmoke')
            self.land.undraw()
            land.draw(self.win)
            self.land = land

        #draws lander graphical representation
        # r = 2.5 = 5m
        # thus on display window, each coordinate point is equivalent to 2 meters.

        r = 2.236
        t = 0.463
        px = r*cos(self.theta + t) + 12.5
        py = r*sin(self.theta + t) + 12.5
        p1 = Point(px, py)
        px = r*cos(self.theta + -t) + 12.5
        py = r*sin(self.theta + -t) + 12.5
        p14 = Point(px,py)

        r = 2.236
        t = 1.107
        px = r*cos(self.theta + t) + 12.5
        py = r*sin(self.theta + t) + 12.5
        p2 = Point(px, py)
        px = r*cos(self.theta + -t) + 12.5
        py = r*sin(self.theta + -t) + 12.5
        p13 = Point(px,py)

        r = 2
        t = 1.571
        px = r*cos(self.theta + t) + 12.5
        py = r*sin(self.theta + t) + 12.5
        p3 = Point(px, py)
        px = r*cos(self.theta + -t) + 12.5
        py = r*sin(self.theta + -t) + 12.5
        p12 = Point(px,py)

        r = 1.118
        t = 2.034
        px = r*cos(self.theta + t) + 12.5
        py = r*sin(self.theta + t) + 12.5
        p4 = Point(px, py)
        px = r*cos(self.theta + -t) + 12.5
        py = r*sin(self.theta + -t) + 12.5
        p11 = Point(px,py)

        r = 2.062
        t = 1.816
        px = r*cos(self.theta + t) + 12.5
        py = r*sin(self.theta + t) + 12.5
        p5 = Point(px, py)
        px = r*cos(self.theta + -t) + 12.5
        py = r*sin(self.theta + -t) + 12.5
        p10 = Point(px,py)

        r = 5
        t = 2.214
        px = r*cos(self.theta + t) + 12.5
        py = r*sin(self.theta + t) + 12.5
        p6 = Point(px, py)
        px = r*cos(self.theta + -t) + 12.5
        py = r*sin(self.theta + -t) + 12.5
        p9 = Point(px,py)

        r = 2.5
        t = 2.14
        px = r*cos(self.theta + t) + 12.5
        py = r*sin(self.theta + t) + 12.5
        p7 = Point(px, py)
        px = r*cos(self.theta + -t) + 12.5
        py = r*sin(self.theta + -t) + 12.5
        p8 = Point(px,py)

        self.lander.undraw()
        self.lander = Polygon(p1,p2,p3,p4,p5,p6,p7,p8,p9,p10,p11,p12,p13,p14)
        self.lander.setFill('grey')
        self.lander.setOutline('grey')
        self.lander.draw(self.win)

        #draws exhaust plume

        r = 1.803
        t = 2.554
        px = r*cos(self.theta + t) + 12.5
        py = r*sin(self.theta + t) + 12.5
        p1 = Point(px, py)
        px = r*cos(self.theta + -t) + 12.5
        py = r*sin(self.theta + -t) + 12.5
        p3 = Point(px,py)

        r = 3 * self.throttle * self.engineOn * self.fueled + 1.5
        t = 3.142
        px = r*cos(self.theta + t) + 12.5
        py = r*sin(self.theta + t) + 12.5
        p2 = Point(px, py)

        self.exhaust.undraw()
        self.exhaust = Polygon(p1,p2,p3)
        self.exhaust.setFill('yellow')
        self.exhaust.draw(self.win)

        #draws prograde vector
        alpha = acos(self.hvel/hypot(self.hvel,self.vvel))
        alpha = copysign(alpha,self.vvel)
        r = 1
        px = r * cos(alpha) + 12.5
        py = r * sin(alpha) + 12.5
        p1 = Point(px,py)

        r = .5
        px = r * cos(alpha+pi*(1/2)) + 12.5
        py = r * sin(alpha+pi*(1/2)) + 12.5
        p2 = Point(px,py)

        px = r * cos(alpha-pi*(1/2)) + 12.5
        py = r * sin(alpha-pi*(1/2)) + 12.5
        p3 = Point(px,py)

        self.vector.undraw()
        self.vector = Polygon(p1,p2,p3)
        self.vector.draw(self.win)

class Trajectory:

    def __init__(self, win, xpos, ypos, xvel, yvel, gravConst, radiusMoon, timeUpdate):
        """Sets up Trajectory object and draws predicted trajectory
        for the next timeUpdate*100 seconds. Also draws the position
        of the lander on the map as a yellow circle."""

        self.gravConst = gravConst
        self.timeUpdate = timeUpdate
        self.win = win

        xp1 = xpos
        yp1 = ypos
        xv1 = xvel
        yv1 = yvel

        self.radiusMoon = radiusMoon

        self.segment = []
        sentinel = True
        while sentinel == True:
            for i in range(100):

                xgrav = (-xp1 * self.gravConst) / pow(hypot(xp1,yp1),3)
                ygrav = (-yp1 * self.gravConst) / pow(hypot(xp1,yp1),3)

                xp2 = xp1 + xv1*self.timeUpdate + .5*xgrav*pow(self.timeUpdate,2)
                yp2 = yp1 + yv1*self.timeUpdate + .5*ygrav*pow(self.timeUpdate,2)

                if hypot(xp2,yp2) <= self.radiusMoon:
                    sentinel = False
                    break

                xv2 = xv1 + xgrav*self.timeUpdate
                yv2 = yv1 + ygrav*self.timeUpdate

                self.segment.append(Line(Point(xp1, yp1), Point(xp2, yp2)))
                self.segment[i].setFill('blue')
                self.segment[i].draw(self.win)
                xp1 = xp2
                yp1 = yp2
                xv1 = xv2
                yv1 = yv2
                if i == 99:
                    sentinel = False

        self.landerMap = Circle(Point(xpos, ypos), 0.05*self.radiusMoon)
        self.landerMap.setFill('yellow')
        self.landerMap.setOutline('yellow')
        self.landerMap.draw(self.win)

    def newPos(self, xpos, ypos, xvel, yvel):
        """Draws the predicted trajectory for the next timeUpdate*100 seconds.
           Also updates the position of the lander on the map."""

        xp1 = xpos
        yp1 = ypos
        xv1 = xvel
        yv1 = yvel


        for i in range(len(self.segment)):
            self.segment[i].undraw()

        sentinel = True
        while sentinel == True:
            for i in range(100):

                xgrav = (-xp1 * self.gravConst) / pow(hypot(xp1,yp1),3)
                ygrav = (-yp1 * self.gravConst) / pow(hypot(xp1,yp1),3)

                xp2 = xp1 + xv1*self.timeUpdate + .5*xgrav*pow(self.timeUpdate,2)
                yp2 = yp1 + yv1*self.timeUpdate + .5*ygrav*pow(self.timeUpdate,2)

                if hypot(xp2,yp2) <= self.radiusMoon:
                    sentinel = False
                    break

                xv2 = xv1 + xgrav*self.timeUpdate
                yv2 = yv1 + ygrav*self.timeUpdate

                newSegment = Line(Point(xp1, yp1), Point(xp2, yp2))
                newSegment.draw(self.win)
                newSegment.setFill('blue')
                self.segment.append(newSegment)

                xp1 = xp2
                yp1 = yp2
                xv1 = xv2
                yv1 = yv2
                if i == 99:
                    sentinel = False

        self.landerMap.undraw()
        self.landerMap = Circle(Point(xpos, ypos), 0.05*self.radiusMoon)
        self.landerMap.setFill('yellow')
        self.landerMap.setOutline('yellow')
        self.landerMap.draw(self.win)

class Prograde:
    """Displays an arrow to indicate the prograde vector (direction you're travelling)."""

    def __init__(self,win,hvel,vvel):
        self.hvel = hvel
        self.vvel = vvel
        self.win = win

        alpha = acos(self.hvel/hypot(self.hvel,self.vvel))
        r = 4
        px = r * cos(-alpha) + 12.5
        py = r * sin(-alpha) + 12.5
        p1 = Point(px,py)

        r = 1
        px = r * cos(-alpha+pi*(1/32)) + 12.5
        py = r * sin(-alpha+pi*(1/32)) + 12.5
        p2 = Point(px,py)

        px = r * cos(-alpha-pi*(1/32)) + 12.5
        py = r * sin(-alpha-pi*(1/32)) + 12.5
        p3 = Point(px,py)

        self.vector = Polygon(p1,p2,p3)
        self.vector.setFill = 'white'
        self.vector.setOutline = 'white'
        self.vector.draw(self.win)

    def refresh(self,hvel,vvel):
        """Updates prograde vector appropriately."""
        self.hvel = hvel
        self.vvel = vvel

        alpha = acos(self.hvel/hypot(self.hvel,self.vvel))
        print(alpha)
        r = 7
        px = r * cos(-alpha) + 12.5
        py = r * sin(-alpha) + 12.5
        p1 = Point(px,py)

        r = 1
        px = r * cos(-alpha+pi*(1/32)) + 12.5
        py = r * sin(-alpha+pi*(1/32)) + 12.5
        p2 = Point(px,py)

        px = r * cos(-alpha-pi*(1/32)) + 12.5
        py = r * sin(-alpha-pi*(1/32)) + 12.5
        p3 = Point(px,py)


        self.vector.undraw()
        self.vector = Polygon(p1,p2,p3)
        self.vector.setFill = 'white'
        self.vector.setOutline = 'white'
        self.vector.draw(self.win)
#        self.vector = vector

def collisionCheck(numAlt, numVel):
    if numAlt <= 2.5:
        if numVel <= 3.0:
            return True, True
        else:
            return False, True
    else:
        return False, False


def main():

    ##############################################
    #  universal constants and derived numbers   #
    ##############################################

    #these are defined so as to be able to set thing differently for gameplay considerations.
    #you can update these to realistic Apollo / Moon values for SCIENCE!

    g = 1.5                                     #m/s^2 towards origin at surface of moon
    radiusMoon = 10000.0                        #m in radius
    gravConst = g*(radiusMoon*radiusMoon)
    initAltitude = 1000.0                       #m
    initYPos = radiusMoon + initAltitude
    initGrav = gravConst/pow(initYPos,2)
    initXVel = sqrt(initGrav*initYPos)          #sets lander in circular orbit
    dryMass = 10000.0                           #kg
    initFuelMass = 4000.0                       #kg
    secFuel = 150.0                             #how many seconds of fuel at full thrust you have
    engineFuelUse = initFuelMass / secFuel      #how many kg of fuel used per second at full throttle
    thrust = 40000                              #thrust in newtons
    isp = thrust/engineFuelUse * 9.81           #specific impulse
    timeUpdate = 0.1                                  #time graniness i.e. update interval


    ##############################################
    # window and static graphical elements setup #
    ##############################################

    mapV = GraphWin("Map View", 300, 300)
    mapV.setCoords(-2*radiusMoon, -2*radiusMoon, 2*radiusMoon, 2*radiusMoon)
    mapV.setBackground('black')

    mapMoon = Circle(Point(0, 0), radiusMoon)
    mapMoon.setFill('darkgrey')
    mapMoon.draw(mapV)

    mapCrater = Circle(Point(.25*radiusMoon, .3*radiusMoon), 1/4*radiusMoon)
    mapCrater.setFill('grey')
    mapCrater.setOutline('grey')
    mapCrater.draw(mapV)
    mapCrater2 = Circle(Point(-.4*radiusMoon, -.1*radiusMoon), 1/6*radiusMoon)
    mapCrater2.setFill('grey')
    mapCrater2.setOutline('grey')
    mapCrater2.draw(mapV)
    mapCrater3 = Circle(Point(.12*radiusMoon, -.4*radiusMoon), 1/6*radiusMoon)
    mapCrater3.setFill('grey')
    mapCrater3.setOutline('grey')
    mapCrater3.draw(mapV)

    win = GraphWin("Lunar Module Simulator", 875, 625) #set for 25 pixels per coordinate point
    win.setCoords(0,0,35,25)
    win.setBackground('black')

    winControls = Rectangle(Point(25,0),Point(35,25))
    winControls.setFill('lightgrey')
    winControls.setOutline('dimgrey')
    winControls.setWidth(2)
    winControls.draw(win)

    winSeparator = Line(Point(25,15.5),Point(35,15.5))
    winSeparator.setFill('dimgrey')
    winSeparator.setWidth(2)
    winSeparator.draw(win)

    winThrottle = Line(Point(26.5,1), Point(26.5,7))
    winThrottle.setWidth(2)
    winThrottle.draw(win)

    winInstructions = Text(Point(30,11.5), "e.....fire main engine\nq......cut main engine\na...rot. anticlockwise\nd.......rot. clockwise\nw........inc. throttle\ns........dec. throttle")
    winReadme = Text(Point(30, 20), "Lunar Module Simulator\n\nTry to land safely on \nthe surface of the    \nmoon with a relative  \nvelocity of less than \n3 meters per second!  \n\n(note: mapview behind \nthis window)         ")

    for i in [winInstructions, winReadme]:
        i.setSize(11)
        i.setFace('courier')
        i.setFill('black')
        i.draw(win)


    winHVel = Text(Point(29, 6.5), "hvel..")
    winVVel = Text(Point(29, 5.5), "vvel..")
    winFuel = Text(Point(29, 4.5), "fuel..")
    winDeltaV = Text(Point(29, 3.5), "dv....")
    winAlt = Text(Point(29, 2.5), "alt...")
    winVel = Text(Point(29, 1.5), "vel...")

    ##############################################
    #     nonstatic graphical elements setup     #
    ##############################################

    disHVel = Text(Point(32.5, 6.5), "{0:5.1f} m/s".format(initXVel))
    disVVel = Text(Point(32.5, 5.5), "{0:5.1f} m/s".format(0))
    disFuel = Text(Point(32.5, 4.5), "{0:5.1f} kg ".format(initFuelMass))
    disDeltaV = Text(Point(32.5, 3.5), "{0:5.1f} m/s".format(isp*9.81*log((initFuelMass + dryMass)/dryMass,e)))
    disAlt = Text(Point(32.5, 2.5), "{0:5.1f} m  ".format(initAltitude))
    disVel = Text(Point(32.5, 1.5), "{0:5.1f} m/s".format(initXVel))


    for i in [winHVel, winVVel, winFuel, winDeltaV, winAlt, winVel, disHVel, disVVel, disFuel, disDeltaV, disAlt, disVel]:
        i.setFace('courier')
        i.draw(win)

    gameOver=Text(Point(13,20), "")
    gameOver.setFace('courier')
    gameOver.setSize(16)
    gameOver.setStyle('bold')
    gameOver.setFill('green')
    gameOver.draw(win)

    ##############################################
    #           fancy animation objects          #
    ##############################################

    #initializes lander math object
    lander = LEM(0, initYPos, initXVel, 0, pi, 0, 0, initFuelMass, dryMass, g, thrust, engineFuelUse, radiusMoon)

    #initializes lander, landscape, prograde vector, and throttle graphics
    landerFX = LanderGraphics(win, (pi/2), initAltitude, initXVel, 0)

    #initializes prograde arrow
    #arrow = Prograde(win, initXVel, 0)

    #initializes trajectory in mapview window
    trajectory = Trajectory(mapV, 0, initYPos, initXVel, 0, gravConst, radiusMoon, 1)

    ##############################################
    #                 game loop                  #
    ##############################################

    collision_happened = False
    landing_happened = False
    t = 0

    while not collision_happened:       #sentinel loop ends game if contact with moon is made

        key = win.checkKey()            #if a key has been pressed, record it
        lander.keyProcess(key)          #channels key input to LEM object
        lander.refresh(timeUpdate)      #calculates timeUpdate seconds into the future and places LEM object in appropriate position

        #gets a bunch of variables from the LEM object
        fueled, omega, theta, throttle, engineOn, xpos, ypos, xvel, yvel, numHVel, numVVel, numFuel, numDeltaV, numAlt, numVel = lander.getDisplayVariables()

        #updates HUD
        disHVel.setText("{0:5.1f} m/s".format(numHVel))
        disVVel.setText("{0:5.1f} m/s".format(numVVel))
        disFuel.setText("{0:5.1f} kg ".format(numFuel))
        disDeltaV.setText("{0:5.1f} m/s".format(numDeltaV))
        disAlt.setText("{0:5.1f} m  ".format(numAlt))
        disVel.setText("{0:5.1f} m/s".format(numVel))

        #makes sure program runs no faster than timeUpdate seconds per timeUpdate seconds
        sleep(timeUpdate)

        #updates lander graphics (lander polygon, exhaust triangle, landscape, throttle handle, prograde vector)
        landerFX.newPos(theta, throttle, engineOn, fueled, numAlt, numHVel, numVVel)

        #updates prograde arrow
        #arrow.refresh(numHVel,numVVel)

        #as trajectory is expensive, only run it once per second.
        if t < 1:
           t = t + timeUpdate
        else:
           trajectory.newPos(xpos, ypos, xvel, yvel)
           t = 0

        #checks to see if lander has made contact with ground (set to 2.5 m)
        #if contact has been made and velocity is less than or equal to 3m/s, sets landing flag
        landing_happened, collision_happened = collisionCheck(numAlt, numVel)

    #sets endgame text
    if landing_happened == True:
        gameOver.setText("You Landed!")
    else:
        gameOver.setText("You Crashed!")




main()
