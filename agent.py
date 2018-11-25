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
def printState(index):
    f = open("out" + str(index) + ".txt", "w")
    index = 0
    for x in screen_data:
        f.write(str(x) + " ")
        index += 1
        if index == w:
            f.write("\n")
            index = 0

#Usually lowestRow = 184
#21 and 115 - very old
#37 and 78 - old
def bottomRowOfInvadors(l,r):
    row = 0
    colomun = 0
    bottomLeft = (184 * 160) + l
    index = bottomLeft

    while True:
        if (screen_data[index] == 20) or (screen_data[index] == 18):
            break
        index += 1
        colomun += 1
        if row > 116:
            #special case (there is only one invador on the game)
            break
        if colomun >= r:
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

def xPositionOfBottomInvadors(l,r):
    bottomRow = bottomRowOfInvadors(l,r)
    distances = []
    row = 0
    index = ((bottomRow - 1) * 160) - row * 160
    distance = 0
    while row < 10:
        if index > 33598:
            return (-1, -1)
        if (screen_data[index] == 20) or (screen_data[index] == 18):
            distances.append((distance, (bottomRow - row)))
            row += 1
            index = ((bottomRow - 1) * 160) - row * 160
            distance = 0
        index += 1
        distance += 1
    return findMin(distances)

def invadorsPositions(l,r):
    position = []
    (distance, row) = xPositionOfBottomInvadors(l,r)
    if distance == -1 and row == -1:
        return []
    invadorsDistances = distance
    index = ((row - 1) * 160) + distance
    max = ((row - 1) * 160) + 138
    while index < max:
        if (screen_data[index] == 20) or (screen_data[index] == 18):
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

def movementDirection(movement):
    startIndex = 31*160 + 23
    leftIndex = 0
    while leftIndex < 154:
        if (screen_data[startIndex] == 20) or (screen_data[startIndex] == 18):
            return "right"
        leftIndex += 1
        startIndex += 160
    startIndex = 31*160 + 136
    rightIndex = 0
    while rightIndex < 154:
        if (screen_data[startIndex] == 20) or (screen_data[startIndex] == 18):
            return "left"
        rightIndex += 1
        startIndex += 160
    return movement


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

def eleminateUnreachableInvadors(invadors):
    newList = []
    for (l,r) in invadors:
        if r >= 37 and l <= 120:
            newList.append((l,r))
    return newList

def getClosestAlien(positions, agent):
    if positions == []:
        return -1
    (l,r) = agent
    agentPos = (l+r)/2
    minDist = 1000
    index = -1
    for (currentMin,currentMax) in positions:
        currentMid = (currentMin + currentMax) / 2
        if abs(currentMid - agentPos) < minDist:
            minDist = abs(currentMid - agentPos)
            index = (currentMin,currentMax)
    return index

def trackBullets(agent):
    bullets = []
    (l,r) = agent
    colomun = 0
    while colomun < (10 + r - l):
        bottomIndex = ((194*160) + l - 5 + colomun)
        row = 0
        while row < 20:
            if (screen_data[bottomIndex] == 4):
                bullets.append(colomun + l - 5)
                break
            row += 1
            bottomIndex -= 160
        colomun += 1
    return bullets

def goTowardsAlien(indexAlien, agent, movement):
    (l,r) = agent
    agentPos = (l+r)/2
    if indexAlien == -1:
        return idle
    (al,ar) = indexAlien
    mid = (al + ar) / 2
    if movement == "right":
        if agentPos >= mid and agentPos <= ar:
            return shoot
        elif agentPos <= mid:
            return right
        elif agentPos >= ar:
            return left
    else:
        if agentPos > al:
            return left
        elif agentPos <= mid and agentPos >= al:
            return leftshoot
        elif agentPos <= al:
            return right

def applyAction(action):
    return ale.act(action)

# Play 10 episodes
for episode in range(10):

    total_reward = 0
    isOneOnOne = False
    movement = "right"

    while not ale.game_over():

        ale.getScreen(screen_data)

        if not isOneOnOne:
            movement = movementDirection(movement)
            agentPos = agentPosition()
            invadors = invadorsPositions(37,78)
            position = eleminateUnreachableInvadors(invadors)
            bullets = trackBullets(agentPosition())

            if bullets:
                (l,r) = agentPos
                mid = (l+r)/2
                if bullets[0] < mid:
                    total_reward += applyAction(right)
                    continue
                elif bullets[0] > mid:
                    total_reward += applyAction(left)
                    continue
                else:
                    if movement == "right":
                        total_reward += applyAction(right)
                        continue
                    else:
                        total_reward += applyAction(left)
                        continue

            a = goTowardsAlien(getClosestAlien(position, agentPos), agentPos, movement)

            if a == idle:
                isOneOnOne = True #we are left with only one invador in the game
            else:
                total_reward += applyAction(a)
        else:
            # --------------------------- Strategy for the last invador -----------------------
            lastInvadorPosition = invadorsPositions(21,115) #Get the position of the last invador

            #Check if you killed the last invador (If you win, the game will crash if you don't do this)
            if lastInvadorPosition:
                (invadorLeft, invadorRight) = lastInvadorPosition[0]
            else:
                total_reward += applyAction(idle)
                isOneOnOne = False      #Makes sure that you will NOT start the new game in one-to-one mode
                continue

            (agentLeft, agentRight) = agentPosition()
            movement = movementDirection(movement)

            #If you want to change the strategy for the last invador, change from this point on
            if invadorLeft >= agentRight:
                if movement == "left":
                    if (agentLeft + agentRight)/2 > (invadorLeft - 5):
                        total_reward += applyAction(shoot)
            elif invadorRight <= agentLeft:
                if movement == "right":
                    if (agentLeft + agentRight)/2 < (invadorRight + 5):
                        total_reward += applyAction(shoot)
            else:
                 total_reward += applyAction(idle)
            # ---------------------------------------------------------------------------------
            #Makes sure that it will return an action even if you don't catch all the cases
            total_reward += applyAction(idle)

    print('Episode %d ended with score: %d' % (episode, total_reward))
    ale.reset_game()
