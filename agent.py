#!/usr/bin/env python
# python_example.py
# Author: Ben Goodrich
#
# This is a direct port to python of the shared library example from
# ALE provided in doc/examples/sharedLibraryInterfaceExample.cpp
import sys
from random import randrange
from ale_python_interface import ALEInterface
import numpy as np
import time

ale = ALEInterface()

# Get & Set the desired settings
ale.setInt(b'random_seed', 123)

# Set USE_SDL to true to display the screen. ALE must be compilied
# with SDL enabled for this to work. On OSX, pygame init is used to
# proxy-call SDL_main.
USE_SDL = True
if USE_SDL:
    if sys.platform == 'darwin':
        import pygame
        pygame.init()
        ale.setBool('sound', False) # Sound doesn't work on OSX
    elif sys.platform.startswith('linux'):
        ale.setBool('sound', True)
    ale.setBool('display_screen', True)
    ale.setBool('color_averaging', True)

# Load the ROM file
ale.loadROM("space_invaders.bin")

#Assings actions to the appropriate name
(idle, shoot, right, left, rightshoot, leftshoot) = ale.getMinimalActionSet()
# Get the list of legal actions
legal_actions = ale.getLegalActionSet()

(w,h) = ale.getScreenDims()
screen_data = np.empty(w*h, dtype=np.uint8)
ale.getScreen(screen_data)

#Write screen_data matrix to a file
def printState():
    f = open("out.txt", "w")
    index = 0
    for x in screen_data:
        if x > 0:
            f.write(str(1))
        else:
            f.write(str(x))
        index += 1
        if index == w:
            f.write("\n")
            index = 0

#Print screen_data matrix
"""
for x in range (0, h):
    for y in range (0, w):
    screen_data[x*y]
    print "\n"
"""

def bottomRowOfInvadors():
    row = 0
    colomun = 0
    bottomLeft = (184 * 160) + 21
    index = bottomLeft

    while True:
        if screen_data[index] == 20:
            break
        index += 1
        colomun += 1
        if colomun == 116:
            colomun = 0
            row += 1
            index = bottomLeft - (row * 160)
    return (185 - row)

def findMin(array):
    index = 0
    min = array[0]
    while index < 10:
        if min[0] > (array[index])[0]:
            min = array[index]
        index += 1
    return min

def xPositionOfBottomInvadors():
    bottomRow = bottomRowOfInvadors()
    distances = []
    row = 0
    index = ((bottomRow - 1) * 160) - row * 160
    distance = 0
    while row < 10:
        if (screen_data[index] == 20):
            distances.append((distance, (bottomRow - row)))
            row += 1
            index = ((bottomRow - 1) * 160) - row * 160
            distance = 0
        index += 1
        distance += 1
    return findMin(distances)

def invadorsPositions():
    position = []
    (distance, row) = xPositionOfBottomInvadors()
    invadorsDistances = distance
    index = ((row - 1) * 160) + distance
    max = ((row - 1) * 160) + 138
    while index < max:
        if screen_data[index] == 20:
            position.append((invadorsDistances, invadorsDistances + 10))
        index += 16
        invadorsDistances += 16
    return position

def agentPosition():
    rowStart = (194 * 160)
    startIndex = rowStart + 22
    endIndex = rowStart + 138
    min = -1
    max = -1

    while startIndex < (rowStart + 138):
        if (screen_data[startIndex] == 196) or (screen_data[startIndex] == 194):
            min = startIndex - rowStart
            break
        startIndex += 1

    while (rowStart + 22) < endIndex:
        if (screen_data[endIndex] == 196) or (screen_data[endIndex] == 194):
            max = endIndex - rowStart
            break
        endIndex -= 1

    return (min,max)

def numberOfInvadors(list):
    if not list:
        return -1
    else:
        return len(list)

def movementDirection():
    startIndex = 31*160 + 23
    leftIndex = 0
    while leftIndex < 154:
        if (screen_data[startIndex] == 20):
            return "Directions changed to right\n"
        leftIndex += 1
        startIndex += 160
    startIndex = 31*160 + 136
    rightIndex = 0
    while rightIndex < 154:
        if (screen_data[startIndex] == 20):
            return "Directions changed to left\n"
        rightIndex += 1
        startIndex += 160
    return "No change"

def getObstaclePositions():
    initialPosition = (174 * 160)
    positions = []
    lowRow = (174 * 160) + 42
    highRow = (157 * 160) + 42
    obstacle = -1
    while (lowRow <= (174*160 + 114)):
        current = lowRow
        hasObstacle = False
        while (current > highRow):
            if screen_data[current] == 52:
                hasObstacle = True
                break
            current -= 160

        if hasObstacle:
            if obstacle == -1:
                obstacle = lowRow - initialPosition
        else:
            if obstacle != -1:
                positions.append((obstacle, lowRow - initialPosition))
                obstacle = -1

        lowRow += 1
        highRow += 1

    return positions

def eleminateUnreachableInvadors(invadors, obstacles):
    newList = []
    for (l,r) in invadors:
        limits = False
        object = False
        for (lo,lr) in obstacles:
            if (l < lo and r > lo) or (l < lr and r > lr):
                object = True
                break
        if r >= 37 and l <= 120:
            limits = True
        if (limits and object):
            newList.append((l,r))
    return newList

def getClosestAlien(positions, agentPos):
    minDist = 1000
    index = -1
    for (currentMin,currentMax) in positions:
        currentMid = (currentMin + currentMax) / 2
        if abs(currentMid - agentPos) < minDist:
            minDist = abs(currentAlien - agentPos)
            index = positions.index(currentAlien)
    return positions[index]

def goTowardsAlien(indexAlien, agentPos):
    mid = (indexAlien[0] + indexAlien[1]) / 2 
    if (mid > agentPos):
        return right
    if (mid < agentPos):
        return left
    return shoot

def isOnLeftEdge():
    rowStart = (194 * 160) + 34
    if (screen_data[rowStart] == 196) or (screen_data[rowStart] == 194):
        return True
    return False

def isOnRightEdge():
    rowStart = (194 * 160) + 122
    if (screen_data[rowStart] == 196) or (screen_data[rowStart] == 194):
        return True
    return False

def isUnderObstacle(list, agentPos):
    (al, ar) = agentPos
    minShootingPoint = (al + ar)/2 - 1
    maxShootingPoint = (al + ar)/2 + 3
    for (l,r) in list:
        if minShootingPoint < r and maxShootingPoint > l:
            return True
    return False

#Get frame number
#ale.getFrameNumber()

games = []
# Play 10 episodes
for episode in range(10):
    total_reward = 0
    while not ale.game_over():
        ale.getScreen(screen_data)

        agentPos = agentPosition()
        obstacles = getObstaclePositions()
        invadors = invadorsPositions()
        position = eleminateUnreachableInvadors(invadors, obstacles)

        if isUnderObstacle(obstacles, agentPos):
            legal_actions = [left, right]
        elif isOnLeftEdge():
            legal_actions = [right, rightshoot]
        elif isOnRightEdge():
            legal_actions = [left, leftshoot]
        else:
            legal_actions = ale.getMinimalActionSet()

        a = legal_actions[randrange(len(legal_actions))]

        #if ale.getFrameNumber() > 130:
            #printState()

        #time.sleep(1)                      #slow frame rate to 1 sec/frame

        #print movementDirection(),         #returns the movement direction

        #print getObstaclePositions()       #returns obstacles position
        #print numberOfInvadors(position)   #returns number of invadors in the bottom line
        #print position
        #print agentPos()                    #returns agent's position

        # Apply an action and get the resulting reward
        reward = ale.act(a);
        total_reward += reward
    print('Episode %d ended with score: %d' % (episode, total_reward))
    ale.reset_game()

"""
f = open("randomWithObstacleAvoidance.txt", "w")

for num in games:
     f.write(str(num) + "\n")
"""
