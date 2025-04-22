from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note
from collections import deque

# robot is the instance of the robot that will allow us to call its methods and to define events with the @event decorator.
robot = Create3(Bluetooth("WALL-E"))  # Will connect to the first robot found.

# === FLAG VARIABLES
HAS_COLLIDED = False
HAS_ARRIVED = False

# === MAZE DICTIONARY
N_X_CELLS = 3 # Size of maze (x dimension)
N_Y_CELLS = 3 # Size of maze (y dimension)
CELL_DIM = 50





# === PROXIMITY TOLERANCES
WALL_THRESHOLD = 80

# ==========================================================
# FAIL SAFE MECHANISMS

# EITHER BUTTON
@event(robot.when_touched, [True, True])  # User buttons: [(.), (..)]
async def when_either_button_touched(robot):
    global HAS_COLLIDED
    HAS_COLLIDED = True
    await robot.set_lights_rgb(255,0,0)
    await robot.set_wheel_speeds(0,0)

# EITHER BUMPER
@event(robot.when_bumped, [True, True])  # [left, right]
async def when_either_bumped(robot):
    global HAS_COLLIDED
    HAS_COLLIDED = True
    await robot.set_lights_rgb(255,0,0)
    await robot.set_wheel_speeds(0,0)
    

# ==========================================================
# Helper Functions

def createMazeDict(nXCells, nYCells, cellDim):
    mazeDict = {}
    for x in range(nXCells):
        for y in range(nYCells):
            otherDict = {}
            otherDict["position"] = (x*cellDim, y*cellDim)
            otherDict["neighbors"] = []
            otherDict["visited"] = False
            otherDict["cost"] = 0
            mazeDict[(x,y)] = otherDict
    return mazeDict
    

def addAllNeighbors(mazeDict,nXCells, nYCells):
    for x in range(nXCells):
        for y in range(nYCells):
            neighbors = []
            if y > 0:
                neighbors.append((x, y-1))
            if y < nYCells - 1:
                neighbors.append((x, y+1))
            if x > 0:
                neighbors.append((x-1, y))
            if x < nXCells - 1:
                neighbors.append((x+1, y))
            mazeDict[(x, y)]['neighbors'] = neighbors
    return mazeDict


def getRobotOrientation(heading):
    NEmid = (0+90)/2
    NWmid = (90+180)/2
    SWmid = (180+270)/2
    SEmid = (360+270)/2
    heading = heading%360
    if NEmid < heading <=NWmid:
        return "N"
    if NWmid < heading <= SWmid:
        return "W"
    if SWmid < heading <= SEmid:
        return "S"
    if SEmid < heading <=360 or 0 <=heading <=NEmid:
        return "E"

def getPotentialNeighbors(currentCell, orientation):
    (x, y) = currentCell
    potentialNeighbors = []
    L = (x-1, y)
    F = (x, y+1)
    R = (x+1, y)
    B = (x, y-1)
    if orientation == "N":
        potentialNeighbors = [L, F, R, B]
    if orientation == "E":
        potentialNeighbors += [F, R, B, L]
    if orientation == "S":
        potentialNeighbors += [R, B, L, F]
    if orientation == "W":
        potentialNeighbors += [B, L,F, R]
    return potentialNeighbors

def isValidCell(cellIndices, nXCells, nYCells):
    (x, y) = cellIndices
    if 0 <= x <= (nXCells - 1) and 0 <= y <= (nXCells - 1):
        return True
    else:
        return False
    
def getWallConfiguration(IR0, IR3, IR6, threshold):
    proximity0 = 4095/(IR0+1)
    proximity3 = 4095/(IR3+1)
    proximity6 = 4095/(IR6+1)
    return [proximity0 < threshold, proximity3 < threshold, proximity6 < threshold]


def getNavigableNeighbors(wallsAroundCell, potentialNeighbors, prevCell, nXCells, nYCells):
    listPossible = []
    for i in range(len(wallsAroundCell)):
        if not wallsAroundCell[i]:
            listPossible.append(i)
    if prevCell is not None:
        indexPrev = potentialNeighbors.index(prevCell)
        listPossible.append(indexPrev)
    output = []
    for index in listPossible:
        if potentialNeighbors[index][0] < nXCells and potentialNeighbors[index][1] < nYCells:
            output.append(potentialNeighbors[index])
    return output

def updateMazeNeighbors(mazeDict, currentCell, navNeighbors):
    for cell in mazeDict:
        if currentCell in mazeDict[cell]["neighbors"]:
            if cell not in navNeighbors:
                mazeDict[cell]["neighbors"].remove(currentCell)
    mazeDict[currentCell]["neighbors"] = navNeighbors
    return mazeDict
    
    
def getNextCell(mazeDict, currentCell):
    if not mazeDict[currentCell]["cost"]:
        return None
    neighborList = []
    for cell in mazeDict[currentCell]["neighbors"]:
        neighborList.append((mazeDict[cell]["cost"], cell))
    low = 10
    changed = False
    for element in neighborList:
        if not mazeDict[element[1]]["visited"]:
            if element[0] < low:
                changed = True
                low = element[0]
                bestCell = element[1]
    if changed:
        return bestCell
    else:
        return neighborList[0][1]

def checkCellArrived(currentCell, destination):
    if currentCell == destination:
        return True
    return False

def printMazeGrid(mazeDict, nXCells, nYCells, attribute):
    for y in range(nYCells - 1, -1, -1):
        row = '| '
        for x in range(nXCells):
            cell_value = mazeDict[(x, y)][attribute]
            row += '{} | '.format(cell_value)
        print(row[:-1])

def updateMazeCost(mazeDict, start, goal):
    for (i,j) in mazeDict.keys():
        mazeDict[(i,j)]["flooded"] = False

    queue = deque([goal])
    mazeDict[goal]['cost'] = 0
    mazeDict[goal]['flooded'] = True

    while queue:
        current = queue.popleft()
        current_cost = mazeDict[current]['cost']

        for neighbor in mazeDict[current]['neighbors']:
            if not mazeDict[neighbor]['flooded']:
                mazeDict[neighbor]['flooded'] = True
                mazeDict[neighbor]['cost'] = current_cost + 1
                queue.append(neighbor)

    return mazeDict

# === BUILD MAZE DICTIONARY

MAZE_DICT = createMazeDict(N_X_CELLS, N_Y_CELLS, CELL_DIM)
MAZE_DICT = addAllNeighbors(MAZE_DICT, N_X_CELLS, N_Y_CELLS)

# === DEFINING ORIGIN AND DESTINATION
PREV_CELL = None
START = (0, 0)
CURR_CELL = START
DESTINATION = (2, 2)
MAZE_DICT[CURR_CELL]["visited"] = True

# ==========================================================
# EXPLORATION AND NAVIGATION

# === EXPLORE MAZE
async def navigateToNextCell(robot, nextMove, orientation):
    global MAZE_DICT, PREV_CELL, CURR_CELL, CELL_DIM
    #below here is modifications

    """
    NEEDED TO MOVE BELOW CODE TO NAVIGATEMAZE -BUILT THIS FUNCTION FIRST ACCIDENTALLY
    global N_X_CELLS, N_Y_CELLS, CELL_DIM, WALL_THRESHOLD #3, 3, 50, 80 defined above
    
    #1) getting the current position of the robot
    heading = await robot.get_position()#how to get this variable??? I think might have to define it in the play?
    orientation = getRobotOrientation(heading)

    #getting possible neighbors 
    neighborList = getPotentialNeighbors(CURR_CELL, orientation)
    walls = getWallConfiguration(IR0, IR3, IR6, WALL_THRESHOLD)
    #i implemented this by just doing the readings in the play and then passing them in here
    navNeighbors = getNavigableNeighbors(walls, neighborList, PREV_CELL, N_X_CELLS, N_Y_CELLS)
    MAZE_DICT = updateMazeNeighbors(MAZE_DICT, CURR_CELL)
    nextMove = getNextCell(MAZE_DICT, CURR_CELL)
    """
    neighborList = getPotentialNeighbors(CURR_CELL, orientation)

    
    #under this is implementing turn part + move
    directions = ("left", "front", "right", "back")
    for i, move in enumerate(neighborList):
        if move == nextMove:
            direction = directions[i]
    if direction == "left":
        await robot.turn_left(90)
    if direction == "front":
        pass
        #await robot.turn_right(90)
    if direction == "right":
        await robot.turn_right(90)
    if direction == "back":
        await robot.turn_right(180)
    await robot.move(CELL_DIM)
  
    
    PREV_CELL = CURR_CELL
    CURR_CELL = nextMove
    

@event(robot.when_play)
async def navigateMaze(robot):
    global HAS_COLLIDED, HAS_ARRIVED
    global PREV_CELL, CURR_CELL, START, DESTINATION
    
    #below code is copy pasted from above
    global MAZE_DICT, N_X_CELLS, N_Y_CELLS, CELL_DIM, WALL_THRESHOLD
    
    #1) getting the current position of the robot
    pos = await robot.get_position()
    heading = pos.heading
    orientation = getRobotOrientation(heading)#this was also used in navToNextCell
    readings = (await robot.get_ir_proximity()).sensors
    IR0 = readings[0]
    IR3 = readings[3]
    IR6 = readings[6]
    #getting possible neighbors 
    neighborList = getPotentialNeighbors(CURR_CELL, orientation)
    walls = getWallConfiguration(IR0, IR3, IR6, WALL_THRESHOLD)
    navNeighbors = getNavigableNeighbors(walls, neighborList, PREV_CELL, N_X_CELLS, N_Y_CELLS)
    MAZE_DICT = updateMazeNeighbors(MAZE_DICT, CURR_CELL, navNeighbors)
    MAZE_DICT = updateMazeCost(MAZE_DICT, START, DESTINATION)
    nextMove = getNextCell(MAZE_DICT, CURR_CELL)#this informatin needed for navtonextcell
    
    while not HAS_COLLIDED and not HAS_ARRIVED:
        await navigateToNextCell(robot, nextMove, orientation)
        HAS_ARRIVED = checkCellArrived(CURR_CELL, DESTINATION)
        if HAS_ARRIVED == True:
            await robot.set_wheel_speeds(0,0)
            await robot.set_lights_rgb(0 ,255 ,0)
        if HAS_COLLIDED:
            await robot.set_wheel_speeds(0,0)
            await robot.set_lights_rgb(255 ,0 ,0)
        pos = await robot.get_position()
        heading = pos.heading
        orientation = getRobotOrientation(heading)#this was also used in navToNextCell
        readings = (await robot.get_ir_proximity()).sensors
        IR0 = readings[0]
        IR3 = readings[3]
        IR6 = readings[6]
        #getting possible neighbors 
        neighborList = getPotentialNeighbors(CURR_CELL, orientation)
        walls = getWallConfiguration(IR0, IR3, IR6, WALL_THRESHOLD)
        navNeighbors = getNavigableNeighbors(walls, neighborList, PREV_CELL, N_X_CELLS, N_Y_CELLS)
        MAZE_DICT = updateMazeNeighbors(MAZE_DICT, CURR_CELL, navNeighbors)
        MAZE_DICT = updateMazeCost(MAZE_DICT, START, DESTINATION)
        nextMove = getNextCell(MAZE_DICT, CURR_CELL)#this informatin needed for navtonextcell
    if HAS_ARRIVED == True:
        await robot.set_wheel_speeds(0,0)
        await robot.set_lights_rgb(0 ,255 ,0)


robot.play()
