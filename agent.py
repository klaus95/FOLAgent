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
import Queue

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
            break #special case (there is only one invador on the game)
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
            return (-1, -1) #special case (win's game)
        if (screen_data[index] == 20) or (screen_data[index] == 18):
            distances.append((distance, (bottomRow - row)))
            row += 1
            index = ((bottomRow - 1) * 160) - row * 160
            distance = 0
        index += 1
        distance += 1
    return findMin(distances), bottomRow

def invadorsPositions(l,r):
    position = []
    positionOfBottomInvadors = xPositionOfBottomInvadors(l,r)
    (dis, row) = positionOfBottomInvadors

    if row == -1:
        return ([], -1) #special case (win's game)
    else:
        (distance, row), bottomRow = positionOfBottomInvadors

    invadorsDistances = distance
    index = ((row - 1) * 160) + distance
    max = ((row - 1) * 160) + 138
    while index < max:
        if (screen_data[index] == 20) or (screen_data[index] == 18):
            position.append((invadorsDistances, invadorsDistances + 10))
        index += 16
        invadorsDistances += 16
    return position, bottomRow

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

def goTowardsAlien(indexAlien, agent, movement, displacement):
    (l,r) = agent
    agentPos = (l+r)/2
    if indexAlien == -1:
        return idle
    (al,ar) = indexAlien
    mid = (al + ar) / 2
    if movement == "right":
        displacement -= 1
        if agentPos - displacement >= mid and agentPos - displacement <= ar:
            return shoot
        elif agentPos - displacement <= mid:
            return right
        elif agentPos - displacement >= ar:
            return left
    else:
        displacement += 1
        if agentPos + displacement <= mid and agentPos + displacement >= al:
            return shoot
        elif agentPos + displacement >= mid:
            return left
        elif agentPos + displacement <= al:
            return right


def avgFrameChange(frames, position):
    maxChange = 1
    currChange = 1
    valChange = 0
    if not frames or len(frames) == 1 or not position:
        return 0,0
    for i in range(1, len(frames)):
        if not frames[i] or not frames[i - 1]:
            continue
        if not frames[i][0] or not frames[i-1][0]:
            continue
        if not frames[i][0][0] or not frames[i - 1][0][0]:
            continue
        if (frames[i][0][0] == frames[i -1][0][0]):
            currChange += 1
        else:
            if (abs(frames[i][0][0] - frames[i - 1][0][0]) > valChange):
                valChange = abs(frames[i][0][0] - frames[i - 1][0][0])
            if currChange > maxChange:
                maxChange = currChange
            currChange = 1
    if valChange < 10:
        return valChange, maxChange
    else:
        return 0, 0

# Play 10 episodes
for episode in range(10):

    total_reward = 0
    isOneOnOne = False
    movement = "right"
    frames = []
    maxValChange = 0
    maxChange = 0
    rateChange = 0.0

    while not ale.game_over():

        ale.getScreen(screen_data)

        movement = movementDirection(movement)
        agentPos = agentPosition()

        if isOneOnOne:
            invadors, bottomRow = invadorsPositions(21,115)
        else:
            invadors, bottomRow = invadorsPositions(37,78)

        if bottomRow == -1:
            total_reward += ale.act(idle)

            isOneOnOne = False
            movement = "right"
            frames = []
            maxValChange = 0
            maxChange = 0
            rateChange = 0.0

            continue

        distanceFromBottomRow = 186 - bottomRow
        position = eleminateUnreachableInvadors(invadors)

        if not position:
            maxValChange = 0
            maxChange = 0
            rateChange = 0.0

        if len(frames) >= 10:
            frames.pop(0)
        frames.append(position)

        currentValChange, currentMaxChange = avgFrameChange(frames, position)

        if (currentValChange > maxValChange):
            maxValChange = currentValChange
            maxChange = currentMaxChange
            rateChange = maxValChange * 0.1 / maxChange

        bullets = trackBullets(agentPosition())
        if bullets:
            (l,r) = agentPos
            mid = (l+r)/2
            if bullets[0] < mid:
                total_reward += ale.act(right)
                continue
            elif bullets[0] > mid:
                total_reward += ale.act(left)
                continue
            else:
                if movement == "right":
                    total_reward += ale.act(right)
                    continue
                else:
                    total_reward += ale.act(left)
                    continue

        a = goTowardsAlien(getClosestAlien(position, agentPos), agentPos, movement, distanceFromBottomRow * rateChange)

        if a == idle:
            isOneOnOne = True

        total_reward += ale.act(a)

    print('Episode %d ended with score: %d' % (episode, total_reward))
    ale.reset_game()
