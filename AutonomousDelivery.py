from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note

import math as m

# robot is the instance of the robot that will allow us to call its methods and to define events with the @event decorator.
robot = Create3(Bluetooth("MARVIN"))  # Will connect to the first robot found.

HAS_COLLIDED = False
HAS_REALIGNED = True
HAS_FOUND_OBSTACLE = False
SENSOR2CHECK = 0
HAS_ARRIVED = False
DESTINATION = (0, 120)
ARRIVAL_THRESHOLD = 5
IR_ANGLES = [-65.3, -38.0, -20.0, -3.0, 14.25, 34.0, 65.3]
BUTTONPRESS = False
HAS_FOUND = False

# Implementation for fail-safe robots
# EITHER BUTTON
@event(robot.when_touched, [True, True])  # User buttons: [(.), (..)]
async def when_either_button_touched(robot):
    global BUTTONPRESS
    BUTTONPRESS = True
    await robot.set_lights_rgb(255,0,0)
    print("Hi")
    await robot.set_wheel_speeds(0,0)
    print("We Done")
    #pass

# EITHER BUMPER
@event(robot.when_bumped, [True, True])  # [left, right]
async def when_either_bumped(robot):
    global BUTTONPRESS
    BUTTONPRESS = True
    await robot.set_lights_rgb(255,0,0)
    print("Yo")
    await robot.set_wheel_speeds(0,0)
    print("We Done!")
    await robot.stop()
    #pass

# ==========================================================

# Helper Functions
def getMinProxApproachAngle(readings):
    global IR_ANGLES 
    maxNum = 0
    maxIndex = 0
    for i, reading in enumerate(readings):
        if reading > maxNum:
            maxNum = reading
            maxIndex = i
    proximity = round((4095/(maxNum + 1)), 3)
            
    aTup = (proximity, IR_ANGLES[maxIndex])

    return aTup
    #pass

def getCorrectionAngle(heading):
    """
    heading = heading % 360
    correctAngle = 90 - heading
    if correctAngle > 180:
        correctAngle += 360
    correctAngle *= -1
    """
    return int(heading - 90)

print(getCorrectionAngle(25))

def getAngleToDestination(currentPosition, destination):
    (xCur, yCur) = currentPosition
    (xDest, yDest) = destination

    xDiff = xDest - xCur
    yDiff = yDest - yCur

    destAngle = m.atan2(xDiff, yDiff)
    degreeAngle = m.degrees(destAngle)

    return int(degreeAngle)

def checkPositionArrived(currentPosition, destination, threshold):
    xCur, yCur = currentPosition
    xDest, yDest = destination

    xDiff = xDest - xCur
    yDiff = yDest - yCur

    distance = m.sqrt((xDiff)**2 + (yDiff)**2)

    return distance <= threshold

currentPosition = (97, 99)
destination = (100, 100)
threshold = 5.0
print(checkPositionArrived(currentPosition, destination, threshold))

# === REALIGNMENT BEHAVIOR
async def realignRobot(robot):
    global HAS_REALIGNED, DESTINATION
    #1 ensure robots orienttioaon is directed towards destination
    # calculate necessary adjustment
    """
    get heading of the robot
    get the position (x,y)
    pass them as parameters into functions
    """
    # execute turn to align robot
    position = await robot.get_position()
    x = position.x
    y = position.y

    heading = position.heading
    await robot.set_wheel_speeds(0,0)
    
    correction_angle = getCorrectionAngle(heading)
    
    angle_destination = getAngleToDestination((x,y), DESTINATION)
    
    await robot.turn_right(angle_destination + correction_angle)

    HAS_REALIGNED  = True

    """
    1: correction agle
    2: angle to destiniation

    1: first rotate robot so it faces north, then rotate towards
    correction angle + angle to destination
    """
    #pass

# === MOVE TO GOAL
async def moveTowardGoal(robot):
    global IR_ANGLES, SENSOR2CHECK, HAS_FOUND_OBSTACLE

    #position = await robot.get_position()

    print("Move Towards Goal")

    readings = (await robot.get_ir_proximity()).sensors
    
    proximity, angle = getMinProxApproachAngle(readings)
    
    if proximity < 20.0:
        await robot.set_wheel_speeds(0,0)
        
        if angle < 0:
    
            await robot.turn_right(90 + angle)
            SENSOR2CHECK = 0 
        else: 
            await robot.turn_left(90 - angle)
            SENSOR2CHECK = -1

        HAS_FOUND_OBSTACLE = True
    else:
        await robot.set_wheel_speeds(10, 10)
    #sensor readings 
    

    

   
        
    #similar to sweeper code, and robot pong 
    # first find obstcle, then rotate, switch to follow obstacle 
        
        
    #pass

# === FOLLOW OBSTACLE
async def followObstacle(robot):
    global HAS_REALIGNED, SENSOR2CHECK,HAS_FOUND_OBSTACLE
    #flag varaible true
    print("Follow Obstacle")
    await robot.set_wheel_speeds(10, 10)

    readings = (await robot.get_ir_proximity()).sensors

    proximity, angle = getMinProxApproachAngle(readings)
    
    if proximity < 20.0:
        if SENSOR2CHECK == -1 :
            await robot.turn_left(3)
        else:
            await robot.turn_right(3)
    elif proximity > 100.00:
        await robot.set_wheel_speeds(10, 10)
        await robot.wait(2.5)
        await robot.set_wheel_speeds(0,0)
        await realignRobot(robot)
        HAS_FOUND_OBSTACLE = False
        
 
        

    
     
    #change flag variablees afterwards
        #change to false 
    #pass

# ==========================================================

# Main function

@event(robot.when_play)
async def makeDelivery(robot):
    global HAS_ARRIVED, HAS_REALIGNED, HAS_FOUND, DESTINATION, ARRIVAL_THRESHOLD, BUTTONPRESS
    while not HAS_ARRIVED:
        if BUTTONPRESS:
            break
        position = await robot.get_position()
        currentposition = (position.x,position.y)
        if BUTTONPRESS:
            break
        if checkPositionArrived(currentposition, DESTINATION, ARRIVAL_THRESHOLD):
            HAS_ARRIVED = True
            if BUTTONPRESS:
                break
            await robot.set_wheel_speeds(0,0)
            await robot.set_lights_rgb(0,255,0)
            break
        
        if not HAS_REALIGNED:
            if BUTTONPRESS:
                break
            await realignRobot(robot)
            
        """
         await followObstacle()
         await followObstacle()
        
         """

        if not HAS_ARRIVED and not HAS_FOUND_OBSTACLE:
            if BUTTONPRESS:
                break
            await moveTowardGoal(robot)
            
        if HAS_FOUND_OBSTACLE:
            if BUTTONPRESS:
                break
            await followObstacle(robot)

        
        
            
            
      

        
    """
    if HAS_ARRIVED == True:
        await robot.set_wheel_speeds(0,0)
        await robot.set_lights_rgb(0,255,0)
    """
    #pass


# start the robot
robot.play()
