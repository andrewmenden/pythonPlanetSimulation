import pygame
import math
from numpy import *
import os
import time

class Projection:
    def __init__(self, rx, ry, screen: pygame.Surface, width, height, sensitivity=0.1):
        self.rx = float(rx)
        self.ry = float(ry)
        self.screen = screen
        self.width = width
        self.height = height
        self.sensitivity = sensitivity
        self.mouseDown = False

    def updateRotationA(self):
        if (self.mouseDown == False):
            self.mouseDown = True
            pygame.mouse.get_rel()
        if (pygame.mouse.get_pressed()[0]):
            relPos = pygame.mouse.get_rel()
            self.rx -= relPos[1]*self.sensitivity
            self.ry -= relPos[0]*self.sensitivity
        else:
            self.mouseDown = False
    def updateRotationB(self, offsetX, offsetY):
        self.rx += offsetY*self.sensitivity
        self.ry += offsetX*self.sensitivity
    def updateRotationC(self, offset2D):
        self.rx += offset2D[1]*self.sensitivity
        self.ry += offset2D[0]*self.sensitivity

    def centerCoordsA(self,x,y):
        return [float(x+self.width/2),float(y+self.height/2)]
    def centerCoordsB(self, pos2D):
        return [float(pos2D[0]+self.width/2),float(pos2D[1]+self.height/2)]

    def projectTo2DA(self,x,y,z):
        x = float(x)
        y = float(y)
        z = float(z)

        return [math.cos(self.ry)*x - z*math.sin(self.ry),
                -x*math.sin(self.rx)*math.sin(self.ry)+y*math.cos(self.rx)-z*math.sin(self.rx)*math.cos(self.ry)]
    def projectTo2DB(self,pos3D):
        x = float(pos3D[0])
        y = float(pos3D[1])
        z = float(pos3D[2])

        return [math.cos(self.ry)*x - z*math.sin(self.ry),
                -x*math.sin(self.rx)*math.sin(self.ry)+y*math.cos(self.rx)-z*math.sin(self.rx)*math.cos(self.ry)]

    def drawCircleA(self, x, y, z, radius, color="white", offset: list=None):
        coord2D = self.projectTo2DA(x,y,z)
        radius = float(radius)
        if (offset == None):
            offset = [self.width/2, self.height/2]
        coord2D[0] += offset[0]
        coord2D[1] += offset[1]
        return pygame.draw.circle(self.screen,color,coord2D,radius)
    def drawCircleB(self, pos3D, radius, color="white"):
        return self.drawCircleA(pos3D[0],pos3D[1],pos3D[2], radius,color)            

    def drawAALineA(self, x1, y1, z1, x2, y2, z2, color="white", offset: list=None):
        start = self.projectTo2DA(x1,y1,z1)
        end = self.projectTo2DA(x2,y2,z2)

        if (offset == None):
            offset = [self.width/2, self.height/2]
        start[0] += offset[0]
        start[1] += offset[1]
        end[0] += offset[0]
        end[1] += offset[1]

        return pygame.draw.aaline(self.screen, color, start, end)
    def drawAALineB(self, pos3DStart, pos3DEnd, color="white", offset: list=None):
        start = self.projectTo2DB(pos3DStart)
        end = self.projectTo2DB(pos3DEnd)

        if (offset == None):
            offset = [self.width/2, self.height/2]
        start[0] += offset[0]
        start[1] += offset[1]
        end[0] += offset[0]
        end[1] += offset[1]

        return pygame.draw.aaline(self.screen, color, start, end)

    def drawAALines(self, listPos3D: list[list[float]], color="white", closed=False, offset: list=None):
        listPos2D = []
        if (offset == None):
            offset = [self.width/2, self.height/2]
        for i in range(len(listPos3D)):
            coord2D = self.projectTo2DB(listPos3D[i])
            coord2D[0] += offset[0]
            coord2D[1] += offset[1]
            listPos2D.append(self.projectTo2DB(listPos3D[i]))
        
        pygame.draw.aalines(self.screen,color,closed,listPos2D)

    def drawAxis(self, length, colorX, colorY, colorZ):
        self.drawAALineA(-length, 0, 0, length, 0, 0, color=colorX)
        self.drawAALineA(0, -length, 0, 0, length, 0, color=colorY)
        self.drawAALineA(0, 0, -length, 0, 0, length, color=colorZ)

class Planet:
    def __init__(self, mass, position, velocity, radius, G = 1, color="white"):
        self.mass = mass
        self.position = array(position)
        self.velocity = array(velocity)
        self.radius = radius
        self.G = G
        self.color = color

    def distanceToPlanet(self, planet):
        sum = 0
        for i in range(len(self.position)):
            temp = (self.position[i]-planet.position[i])
            sum += temp*temp # might be faster
        return math.sqrt(sum)

    def gravityToBody(self, planet):
        # a = G*planet.mass/r^2
        r = self.distanceToPlanet(planet)
        if (r == 0):
            r = 1
        magnitude = self.G*planet.mass/(r*r)

        v = planet.position-self.position
        v /= r

        #collisions here
        if r<10:
            print("COLLISION")

        return v*magnitude

    def plotPlanet(self, projection: Projection):
        projection.drawCircleA(self.position[0],self.position[1], self.position[2],self.radius, self.color)

    def printPlanet(self):
        print(f'\tmass: {self.mass}\n\tposition: {self.position}\n\tvelocity: {self.velocity}')

class PlanetSystem:
    def __init__(self, count):
        self.count = count
        self.planets = None
    
    def createPlanetsA(self, masses, positions, velocities, radii, G, colors):
        self.planets = []
        for i in range(self.count):
            planet = Planet(masses[i], positions[i], velocities[i], radii[i], G, colors[i])
            self.planets.append(planet)
    
    def createPlanetsB(self, G):
        self.planets = []
        for i in range(self.count):
            mass = random.random()*10.0+5
            position = [random.random()*500.0-250.0,
                        random.random()*500.0-250.0,
                        random.random()*500.0-250.0]
            velocity = [random.random()*1.0-0.5,
                        random.random()*1.0-0.5,
                        random.random()*1.0-0.5]
            planet = Planet(mass,position,velocity,mass,G)
            self.planets.append(planet)
    
    def getNetGravity(self, index):
        gravity = array([0.0,0.0,0.0])

        for i in range((len(self.planets)-1)):
            if (i >= index):
                i+=1
            gravity += self.planets[index].gravityToBody(self.planets[i])
        return gravity
    
    def getNetGravities(self):
        g = []
        for i in range(len(self.planets)):
            g.append(self.getNetGravity(i))
    
    def updatePlanetSystem(self, dt):
        for i in range(len(self.planets)):
            g = self.getNetGravity(i)
            self.planets[i].velocity += g*dt
            self.planets[i].position += self.planets[i].velocity

    def plotPlanetSystem(self, projection: Projection):
        for i in range(len(self.planets)):
            self.planets[i].plotPlanet(projection)

    def printPlanetSystem(self, dt):
        for i in range(len(self.planets)):
            print(f'Planet #{i}: ')
            self.planets[i].printPlanet()

def startApp():
    width = 1920
    height = 1080

    pygame.init()
    screen = pygame.display.set_mode((width, height))
    clock = pygame.time.Clock()
    running = True

    projection = Projection(0.0, 0.0, screen, width, height, .001)
    dt = .01
    planets = PlanetSystem(3)

    # good initial condition
    planets.createPlanetsA([5.409172655140732,10.36905376170167,8.369804416751979],[[219.96239253, 245.13260104,  46.49834914],[-17.86950234, -136.39216907,  246.87221765],[-249.02377696, 140.24912708, 242.98671764]], [[0.35574953, -0.16320471, -0.25959253],[0.45991148, 0.39407383, 0.1106445 ],[-0.48664203, -0.41484845, 0.27831551]],[5,10,8], 5000, ["red","white","blue"])
    
    planets.printPlanetSystem(dt)
    rDown = False

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        projection.updateRotationA()

        screen.fill("black")
        projection.drawAxis(500,"red","green","blue")

        # tickSpacing = 500/10
        # for i in range(10):
        #     projection.drawAALineB([tickSpacing*i,])    

        planets.updatePlanetSystem(dt)
        planets.plotPlanetSystem(projection)

        if (pygame.key.get_pressed()[pygame.key.key_code("r")]):
            if (rDown == False):
                # for windows only
                os.system("cls")
                planets = PlanetSystem(3)
                planets.createPlanetsB(5000)
                planets.printPlanetSystem(dt)
                rDown = True
        else:
            if (rDown == True):
                rDown = False
        pygame.display.flip()

        clock.tick(144)  # limits FPS to 144

    pygame.quit()

startApp()
