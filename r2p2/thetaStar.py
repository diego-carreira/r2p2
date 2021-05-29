""" This module implements the A* path planning algorithm.

Two variants are included: grid-based, and mesh-based.

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.
This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with
this program. If not, see <http://www.gnu.org/licenses/>.
"""

__author__ = "Mario Cobos Maestre"
__authors__ = ["Mario Cobos Maestre"]
__contact__ = "mario.cobos@edu.uah.es"
__copyright__ = "Copyright 2019, UAH"
__credits__ = ["Mario Cobos Maestre"]
__date__ = "2019/03/29"
__deprecated__ = False
__email__ =  "mario.cobos@edu.uah.es"
__license__ = "GPLv3"
__maintainer__ = "Mario Cobos Maestre"
__status__ = "Development"
__version__ = "0.0.1"


"""
    Code modified from https://gist.github.com/jamiees2/5531924
"""

import path_planning as pp

def children(point,grid):
    """
        Calculates the children of a given node over a grid.
        Inputs:
            - point: node for which to calculate children.
            - grid: grid over which to calculate children.
        Outputs:
            - list of children for the given node.
    """
    
    x,y = point.grid_point
    if x > 0 and x < len(grid) - 1:
        if y > 0 and y < len(grid[0]) - 1:
            links = [grid[d[0]][d[1]] for d in\
                     [(x-1, y),(x,y - 1),(x,y + 1),(x+1,y),\
                      (x-1, y-1), (x-1, y+1), (x+1, y-1),\
                      (x+1, y+1)]]
        elif y > 0:
            links = [grid[d[0]][d[1]] for d in\
                     [(x-1, y),(x,y - 1),(x+1,y),\
                      (x-1, y-1), (x+1, y-1)]]
        else:
            links = [grid[d[0]][d[1]] for d in\
                     [(x-1, y),(x,y + 1),(x+1,y),\
                      (x-1, y+1), (x+1, y+1)]]
    elif x > 0:
        if y > 0 and y < len(grid[0]) - 1:
            links = [grid[d[0]][d[1]] for d in\
                     [(x-1, y),(x,y - 1),(x,y + 1),\
                      (x-1, y-1), (x-1, y+1)]]
        elif y > 0:
            links = [grid[d[0]][d[1]] for d in\
                     [(x-1, y),(x,y - 1),(x-1, y-1)]]
        else:
            links = [grid[d[0]][d[1]] for d in\
                     [(x-1, y), (x,y + 1), (x-1, y+1)]]
    else:
        if y > 0 and y < len(grid[0]) - 1:
            links = [grid[d[0]][d[1]] for d in\
                     [(x+1, y),(x,y - 1),(x,y + 1),\
                      (x+1, y-1), (x+1, y+1)]]
        elif y > 0:
            links = [grid[d[0]][d[1]] for d in\
                     [(x+1, y),(x,y - 1),(x+1, y-1)]]
        else:
            links = [grid[d[0]][d[1]] for d in\
                     [(x+1, y), (x,y + 1), (x+1, y+1)]]
    return [link for link in links if link.value != 9]

def lineaVision(start, goal, grid):
    

    inicioX= start.point[0]
    inicioY= start.point[1]
    metaX=goal.point[0]
    metaY=goal.point[1]

    distanciaY= metaY-inicioY
    distanciaX= metaX-inicioX
    f=0
    if distanciaY<0:
        distanciaY=0-distanciaY
        start.point[1]=-1
    else:
        start.point[1]=1
    if distanciaX <0:
        distanciaX=0-distanciaX
        start.point[0]=-1
    else:
        start.point[0]=1
    
    if distanciaX >= distanciaY:
        while inicioX!=metaX:
            f=f+distanciaY
            if f>= distanciaX:
                if grid[inicioX+((start.point[0]-1)/2)][inicioY+((start.point[1]-1)/2)]:
                    return False
                
                inicioY=inicioY+start.point[1]
                f=f-distanciaX

            if f!=0 and grid[inicioX+((start.point[0]-1)/2)][inicioY+((start.point[1]-1)/2)]:
                return False
            
            if distanciaY==0 and grid[inicioX+((start.point[0]-1)/2)][inicioY] and grid[inicioX+((start.point[0]-1)/2)][inicioY-1]  :
                return False
            
            inicioX=inicioX+start.point[0]
    
    else:
        while inicioY != metaY:
            f=f+distanciaX
            if f>= distanciaY:
                if grid[inicioX+((start.point[0]-1)/2)][inicioY+((start.point[1]-1)/2)]:
                    return False
                
                inicioX=inicioX+start.point[0]
                f=f-distanciaY

            if f!=0 and grid[inicioX+((start.point[0]-1)/2)][inicioY+((start.point[1]-1)/2)]:
                return False
            
            if distanciaX==0 and grid[inicioX][inicioY+((start.point[1]-1)/2)] and grid[inicioX-1][inicioY+((start.point[1]-1)/2)] :
                return False
            
            inicioY=inicioY+start.point[1]
    
    return True

def thetaStar(start, goal, grid, heur='naive'):
    
    #The open and closed sets
    openset = set()
    closedset = set()
    #Current point is the starting point
    current = start
    #Add the starting point to the open set
    openset.add(current)
    #While the open set is not empty
    while openset:
        #Find the item in the open set with the lowest G + H score
        current = min(openset, key=lambda o:o.G + o.H)
        #If it is the item we want, retrace the path and return it
        if current == goal:
            path = []
            while current.parent:
                path.append(current)
                current = current.parent
            path.append(current)
            return path[::-1]
        #Remove the item from the open set
        openset.remove(current)
        #Add it to the closed set
        closedset.add(current)
        #Loop through the node's children/siblings
        for node in children(current,grid):
            #If it is already in the closed set, skip it
            if node in closedset:
                continue
            #Otherwise if it is already in the open set
            if node in openset:
                if lineaVision(current.parent,node,grid):
                    nuevoPadre=current.parent.G+current.parent.move_cost(node)
                    if nuevoPadre < node.G:
                        node.G=nuevoPadre
                        node.parent=current.newParent
                
                else:
                    nuevo= current.G+current.move_cost(node)
                    if node.G > nuevo:
                        node.G=nuevo
                        node.parent=current
                
            else:
                #If it isn't in the open set, calculate the G and H score for the node
                node.G = current.G + current.move_cost(node)
                node.H = pp.heuristic[heur](node, goal)
                #Set the parent to our current item
                node.parent = current
                #Add it to the set
                openset.add(node)
    #Throw an exception if there is no path
    raise ValueError('No Path Found')

pp.register_search_method('Theta*', thetaStar)

def thetaStar_mesh(start, goal, grid, heur='naive'):
    
    #The open and closed sets
    openset = set()
    closedset = set()
    #Current point is the starting point
    current = start
    #Add the starting point to the open set
    openset.add(current)
    #While the open set is not empty
    while openset:
        #Find the item in the open set with the lowest G + H score
        current = min(openset, key=lambda o:o.G + o.H)
        #If it is the item we want, retrace the path and return it
        if current == goal:
            path = []
            while current.parent:
                path.append(current)
                current = current.parent
            path.append(current)
            return path[::-1]
        #Remove the item from the open set
        openset.remove(current)
        #Add it to the closed set
        closedset.add(current)
        #Loop through the node's children/siblings
        for node in children(current,grid):
            #If it is already in the closed set, skip it
            if node in closedset:
                continue
            #Otherwise if it is already in the open set
            if node in openset:
                if lineaVision(current.parent,node,grid):
                    nuevoPadre=current.parent.G+current.parent.move_cost(node)
                    if nuevoPadre < node.G:
                        node.G=nuevoPadre
                        node.parent=current.newParent
                
                else:
                    nuevo= current.G+current.move_cost(node)
                    if node.G > nuevo:
                        node.G=nuevo
                        node.parent=current
                
            else:
                #If it isn't in the open set, calculate the G and H score for the node
                node.G = current.G + current.move_cost(node)
                node.H = pp.heuristic[heur](node, goal)
                #Set the parent to our current item
                node.parent = current
                #Add it to the set
                openset.add(node)
    #Throw an exception if there is no path
    raise ValueError('No Path Found')

pp.register_search_method('Theta* mesh', thetaStar_mesh)




