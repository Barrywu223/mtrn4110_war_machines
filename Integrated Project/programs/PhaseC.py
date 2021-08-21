import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import os
import signal
import sys
import subprocess
import time
import pyautogui

# CHANGE THESE VARIABLES IF USING ON ANOTHER SYSTEM
path = 'C:/Users/Stuart/AppData/Local/Programs/Webots/msys64/mingw64/bin'


if len(sys.argv) != 2:
    print('usage: python PhaseC.py worldName')
    exit(1)
worldName = sys.argv[1]


def cvShow(image, name):
    cv.imshow(name, image)
    cv.waitKey(0)
    cv.destroyAllWindows()


def pltShow(img, name):
    plt.title(name)
    plt.imshow(img)
    plt.show()


def resize(image, width=None, height=None, inter=cv.INTER_AREA):
    dim = None
    (h, w) = image.shape[:2]
    if width is None and height is None:
        return image
    if width is None:
        r = height/float(h)
        dim = (int(w*r), height)
    else:
        r = width/float(w)
        dim = (width, int(h*r))
    resized = cv.resize(image, dim, interpolation=inter)
    return resized

def printMaxtirx(matrix):
    for row in matrix:
        print(row)
    print(' ')

print("[war_machines_PhaseC] Generating map from images...")

MAZE_FILE_NAME = '../worlds/' + worldName + '_Maze.png'
ROBOT_FILE_NAME = '../worlds/' + worldName + '_Robot.png'
IMAGE_LADYBUG_FILE_NAME = '../worlds/Ladybug_small.png'
TXT_FILE_NAME = '../Map.txt'

MAZE_IMAGE_WIDTH = 1350
MAZE_IMAGE_HEIGHT = 750
ROBOT_IMAGE_WIDTH = 750
ROBOT_IMAGE_HEIGHT = 750

LIGHT_GREEN = [50, 240, 120]
LIGHT_PINK = [230, 130, 235]
LIGHT_BLUE = [130, 240, 240]
LIGHT_YELLOW = [255, 71, 25]
ORANGE = [255, 255, 75]
BLACK = [0, 0, 0]
WHITE = [255, 255, 255]
GREEN = [30, 220, 40]
BLUE = [0, 0, 255]
GOOD_YELLOW = [235, 225, 35]

NORTH = 0
SOUTH = 1
WEST = 2
EAST = 3

kernal9 = np.ones((9, 9), "uint8")
kernal7 = np.ones((7, 7), "uint8")
kernal5 = np.ones((5, 5), "uint8")
kernal3 = np.ones((3, 3), "uint8")
kernal1 = np.ones((1, 1), "uint8")

pink_lower = np.array([125, 200, 220])
pink_upper = np.array([175, 255, 255])

cyan_lower = np.array([80, 120, 220])
cyan_upper = np.array([100, 140, 255])

wall_lower = np.array([17, 40, 215])
wall_upper = np.array([22, 125, 255])

robot_lower = np.array([30, 0, 140])
robot_upper = np.array([120, 175, 200])

maze = cv.imread(MAZE_FILE_NAME)
robot = cv.imread(ROBOT_FILE_NAME)
ladybug = cv.imread(IMAGE_LADYBUG_FILE_NAME)
maze = resize(maze, width=MAZE_IMAGE_WIDTH, height=MAZE_IMAGE_HEIGHT)
maze_hsv = cv.cvtColor(maze, cv.COLOR_BGR2HSV)

maze_rgb = cv.cvtColor(maze, cv.COLOR_BGR2RGB)

maze2 = maze_rgb.copy()

# To find cyan(key)conerstones

cyan_mask = cv.inRange(maze_hsv, cyan_lower, cyan_upper)

cyan_mask = cv.dilate(cyan_mask, kernal3, iterations=1)

# pltShow(cyan_mask, 'cyan mask')

img,cyanContours, cyanHierarchy = cv.findContours(
    cyan_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

cyanContours = sorted(cyanContours, key=cv.contourArea, reverse=True)[:1]

if len(cyanContours) != 0:
    for contour in cyanContours:
        if cv.contourArea(contour) > 200:
            cv.ellipse(maze2, cv.fitEllipse(contour), LIGHT_YELLOW, 3)
            x, y, w, h = cv.boundingRect(contour)
            x_cord = int((x+x+w)/2)
            y_cord = int((y+y+h)/2)
            key_center = (x_cord, y_cord)

# To find pink cornerstones

pink_mask = cv.inRange(maze_hsv, pink_lower, pink_upper)

pink_mask = cv.dilate(pink_mask, kernal3, iterations=1)

# pltShow(pink_mask, 'pink mask')

img, pinkContours, pinkHierarchy = cv.findContours(
    pink_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

pinkContours = sorted(pinkContours, key=cv.contourArea, reverse=True)[:3]
if len(pinkContours) != 0:
    for contour in pinkContours:
        if cv.contourArea(contour) > 200:
            cv.ellipse(maze2, cv.fitEllipse(contour), ORANGE, 3)

maze3 = maze2.copy()

center_record = {}

for c in pinkContours:
    x, y, w, h = cv.boundingRect(c)
    x_cord = int((x+x+w)/2)
    y_cord = int((y+y+h)/2)
    center = (x_cord, y_cord)
    if (key_center[0] > 0 and key_center[0] < 337.5 and key_center[1] > 0 and key_center[1] < 375):

        if (x_cord > 0 and x_cord < 337.5 and y_cord > 375 and y_cord < 750):
            center_record[1] = center
        elif (x_cord > 1012.5 and x_cord < 1350 and y_cord > 0 and y_cord < 375):
            center_record[0] = center
        else:
            center_record[2] = center
    elif (key_center[0] > 1012.5 and key_center[0] < 1350 and key_center[1] > 375 and key_center[1] < 750):
        if (x_cord > 1012.5 and x_cord < 1350 and y_cord > 0 and y_cord < 375):
            center_record[1] = center
        elif (x_cord > 0 and x_cord < 337.5 and y_cord > 375 and y_cord < 750):
            center_record[0] = center
        else:
            center_record[2] = center
    else:
        print('Original image rotated too much！！！')

pts1 = np.float32([key_center, center_record[0], center_record[1], center_record[2]])
pts2 = np.float32([[0, 0], [MAZE_IMAGE_WIDTH, 0], [0, MAZE_IMAGE_HEIGHT], [MAZE_IMAGE_WIDTH, MAZE_IMAGE_HEIGHT]])
matrix = cv.getPerspectiveTransform(pts1, pts2)
maze3 = cv.warpPerspective(
    maze3, matrix, (MAZE_IMAGE_WIDTH, MAZE_IMAGE_HEIGHT))

robot = cv.copyMakeBorder(robot, 0, 0, 300, 300, cv.BORDER_CONSTANT)
# cvShow(robot, 'nonTransfered Robot')
robot = cv.warpPerspective(
    robot, matrix, (MAZE_IMAGE_WIDTH, MAZE_IMAGE_HEIGHT))

maze4 = maze3.copy()
maze_hsv = cv.cvtColor(maze4, cv.COLOR_RGB2HSV)
wall_mask = cv.inRange(maze_hsv, wall_lower, wall_upper)
# cvShow(wall_mask, 'Wall mask')
wall_mask = cv.morphologyEx(wall_mask, cv.MORPH_CLOSE, kernal7, iterations=1)
# cvShow(wall_mask, 'After close')
wall_mask = cv.erode(wall_mask, kernal3, iterations=2)
# cvShow(wall_mask, 'After erosion')
wall_mask = cv.dilate(wall_mask, kernal3, iterations=2)
# cvShow(wall_mask, 'After dilation')


wall_mask = 255-wall_mask
maze4 = cv.bitwise_and(maze4, maze4, mask=wall_mask)
Rmask = np.all(maze4 == BLACK,  axis=-1)
maze4[Rmask] = LIGHT_GREEN
wall_mask = 255-wall_mask

maze5 = maze4.copy()

dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250)

robot_bgr = robot.copy()
# robot_rgb_origin = cv.cvtColor(robot_bgr, cv.COLOR_BGR2RGB)
robot_rgb = cv.cvtColor(robot_bgr, cv.COLOR_BGR2RGB)

parameters = cv.aruco.DetectorParameters_create()

markerCorners, markerIds, rejectedCandidates = cv.aruco.detectMarkers(
    robot_rgb, dictionary, parameters=parameters)

robot_rgb = cv.aruco.drawDetectedMarkers(robot_rgb, markerCorners, markerIds)

acceptBias = 20.0
# print(markerCorners)
if (abs(markerCorners[0][0][0][1] - markerCorners[0][0][2][1]) < acceptBias and markerCorners[0][0][0][0] < markerCorners[0][0][2][0]):
    direction = WEST
#     print('west!')
elif (abs(markerCorners[0][0][0][1] - markerCorners[0][0][2][1]) < acceptBias and markerCorners[0][0][0][0] > markerCorners[0][0][2][0]):
    direction = EAST
#     print('east!')
elif (abs(markerCorners[0][0][0][0] - markerCorners[0][0][2][0]) < acceptBias and markerCorners[0][0][0][1] < markerCorners[0][0][2][1]):
    direction = NORTH
#     print('north!')
elif (abs(markerCorners[0][0][0][0] - markerCorners[0][0][2][0]) < acceptBias and markerCorners[0][0][0][1] > markerCorners[0][0][2][1]):
    direction = SOUTH
#     print('south!')
else:
    print('Oops! I cant recognize the direction')
    # print(markerCorners[0][0][0][0], markerCorners[0]
    #       [0][2][0], markerCorners[0][0][0][0], markerCorners[0][0][2][0])

maze_hsv = cv.cvtColor(maze5.copy(), cv.COLOR_RGB2HSV)
robot_mask = cv.inRange(maze_hsv, robot_lower, robot_upper)


robot_mask = cv.erode(robot_mask, kernal3, iterations=1)
# cvShow(robot_mask, 'robot mask')
robot_mask = cv.morphologyEx(robot_mask, cv.MORPH_CLOSE, kernal7, iterations=7)
# robot_mask = cv.erode(robot_mask, kernal1, iterations=5)
robot_mask = cv.dilate(robot_mask, kernal3, iterations=3)
# cvShow(robot_mask, 'robot mask')


_, robotContours, robotHierarchy = cv.findContours(
    robot_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
# robotContours = sorted(robotContours, key=cv.contourArea, reverse=True)[:1]
robotContours = max(robotContours, key=cv.contourArea)
x, y, w, h = cv.boundingRect(robotContours)
row, col = np.where(robot_mask > 0)
robot_center = (int(x+w/2), int(y+h/2))
# print(robot_center)
robot_radius = 30
maze5 = cv.circle(maze5, robot_center, robot_radius, GOOD_YELLOW, 3)



length = 15
if direction == NORTH:
    point1 = (robot_center[0], robot_center[1]-length)
    point2 = (robot_center[0]-length, robot_center[1]+length)
    point3 = (robot_center[0]+length, robot_center[1]+length)
elif direction == SOUTH:
    point1 = (robot_center[0], robot_center[1]+length)
    point2 = (robot_center[0]-length, robot_center[1]-length)
    point3 = (robot_center[0]+length, robot_center[1]-length)
elif direction == WEST:
    point1 = (robot_center[0]-length, robot_center[1])
    point2 = (robot_center[0]+length, robot_center[1]-length)
    point3 = (robot_center[0]+length, robot_center[1]+length)
elif direction == EAST:
    point1 = (robot_center[0]+length, robot_center[1])
    point2 = (robot_center[0]-length, robot_center[1]-length)
    point3 = (robot_center[0]-length, robot_center[1]+length)

maze5 = cv.line(maze5, point1, point2, GOOD_YELLOW, 3)
maze5 = cv.line(maze5, point1, point3, GOOD_YELLOW, 3)

# Detect the position for true target

maze6 = maze5.copy()

template = ladybug.copy()
w, h = template.shape[:-1]

# methods = ['cv.TM_CCOEFF', 'cv.TM_CCOEFF_NORMED', 'cv.TM_CCORR',
#            'cv.TM_CCORR_NORMED', 'cv.TM_SQDIFF', 'cv.TM_SQDIFF_NORMED']
img = maze6.copy()
method = cv.TM_CCOEFF

res = cv.matchTemplate(img, template, method)
min_val, max_val, min_loc, max_loc = cv.minMaxLoc(res)
if method in [cv.TM_SQDIFF, cv.TM_SQDIFF_NORMED]:
    top_left = min_loc
else:
    top_left = max_loc

bottom_right = (top_left[0] + w, top_left[1] + h)

ladybug_center = (int(top_left[0]+w/2), int(top_left[1]+h/2))

# Draw oulter circle
ladybug_radius = 60
cv.circle(maze6, ladybug_center, ladybug_radius, BLUE, 3)

# Drae cross sign
length = 30
point1 = (ladybug_center[0]-length, ladybug_center[1]-length)
point2 = (ladybug_center[0]+length, ladybug_center[1]-length)
point3 = (ladybug_center[0]+length, ladybug_center[1]+length)
point4 = (ladybug_center[0]-length, ladybug_center[1]+length)

cv.line(maze6, point1, point3, BLUE, 4)
cv.line(maze6, point2, point4, BLUE, 4)

# initialize the vertical wall matrix
rows, cols = (5, 10)
virtical_wall = np.zeros((5, 10))

# initialize the horizontal wall matrix
rows, cols = (6, 9)
horizontal_wall = np.zeros((6, 9))

wall_mask = cv.dilate(wall_mask,kernal3,1)
# cvShow(wall_mask,'Dilated wall mask')

bias = 10
for i, row in enumerate(virtical_wall):

    for j, val in enumerate(row):
        if (j == 0 or j == 9):
            row[j] = 1

        elif (wall_mask[150*i+75][150*j] == 255 or wall_mask[150*i+75+bias][150*j] or wall_mask[150*i+75-bias][150*j]):
            row[j] = 1

for i, row in enumerate(horizontal_wall):

    for j, val in enumerate(row):
        if (i == 0 or i == 5):
            row[j] = 1

        elif (wall_mask[150*i][150*j+75] or wall_mask[150*i][150*j+75+bias] or wall_mask[150*i][150*j+75-bias]):
            row[j] = 1

robot_coordinate = (
    round((robot_center[1]-75)/150), round((robot_center[0]-75)/150))
ladybug_coordinate = (
    round((ladybug_center[1]-75)/150), round((ladybug_center[0]-75)/150))

map = open(TXT_FILE_NAME, 'w', encoding="utf-8")

# print(horizontal_wall[0][:])
message = []
for i in range(11):
    newMessage = []
    # record horizontal wall:
    if (i % 2 == 0):

        for val in horizontal_wall[int(i / 2)]:

            newMessage.append(' ')
            if (val == 1):
                newMessage.append('---')

            else:
                newMessage.append('   ')

    # record vertical wall:
    else:
        colCounter = 0
        for val in virtical_wall[int((i-1) / 2)]:

            if (val == 1):
                newMessage.append('|')
            else:
                newMessage.append(' ')

            if(int((i-1) / 2) == ladybug_coordinate[0] and colCounter == ladybug_coordinate[1]):
                newMessage.append(' x ')
            elif(int((i-1) / 2) == robot_coordinate[0] and colCounter == robot_coordinate[1]):
                if (direction == NORTH):
                    newMessage.append(' ^ ')
                elif (direction == SOUTH):
                    newMessage.append(' v ')
                elif (direction == EAST):
                    newMessage.append(' > ')
                elif (direction == WEST):
                    newMessage.append(' < ')
            else:
                newMessage.append('   ')

            colCounter += 1
    message.append(newMessage)

print("[war_machines_PhaseC] Map generated: ")
for content in message:
    mapStr = ''
    for elements in content:
        mapStr += elements
    print(mapStr)
    mapStr += '\n'
    map.write(mapStr)
map.close()
print("[war_machines_PhaseC] Map saved to " + TXT_FILE_NAME)












print("[war_machines_PhaseC] Running Webots...")

# RUNS WEBOTS
# path = 'F:/Webots/msys64/mingw64/bin' # or whereever your webots installation is

worldPath = os.getcwd() + '/../worlds/' + worldName + '.wbt'
webots = subprocess.Popen(['webots', worldPath], shell=True, cwd=path, 
                          stdout=subprocess.PIPE, creationflags=subprocess.CREATE_NEW_PROCESS_GROUP)

print("[war_machines_PhaseC] Waiting for Webots to open...")
time.sleep(5)

# RECORDS WEBOTS
print("[war_machines_PhaseC] Recording Webots for path analysis...")
# Define filename/type/dimensions
filename = "output.avi"
record_seconds = 30
fps = 10

# Dont change anything below
prev = 0
frames = 0
video_type = cv.VideoWriter_fourcc(*'XVID')

out = cv.VideoWriter(filename,video_type,fps,(1920,1080))

# Loop to record video
time_elapsed = time.time() - prev
while frames < fps * record_seconds:
    # Record frame
    time_elapsed = time.time() - prev
    img = pyautogui.screenshot()
    if time_elapsed > 1.0/fps:
        frames += 1
        prev = time.time()
        frame = np.array(img)
        frame = cv.cvtColor(frame,cv.COLOR_BGR2RGB)
        frame = resize(frame, width=1920, height=1080)
        out.write(frame)
        
out.release()
print("[war_machines_PhaseC] Finished Recording!")
print("[war_machines_PhaseC] Closing Webots...")
subprocess.Popen(['taskkill', '/F', '/T', '/PID', str(webots.pid)], stdout=open(os.devnull, 'wb'))
print("[war_machines_PhaseC] Webots closed successfully!")

# print("[war_machines_PhaseC] Tracking robot in the recording...")

# cap = cv.VideoCapture("output.avi")

# while cap.isOpened():
#     ret, frame = cap.read()
    
#     if ret:
#         hsv = cv.cvtColor(frame,cv.COLOR_BGR2HSV)
#         cv.imshow('Frame', hsv)
#         # mask_robot = cv.inRange(hsv, robot_lower, robot_upper)

#         # mask_robot = cv.erode(mask_robot, kernal3, iterations=1)
#         # # cvShow(robot_mask, 'robot mask')
#         # mask_robot = cv.morphologyEx(mask_robot, cv.MORPH_CLOSE, kernal7, iterations=7)
#         # # robot_mask = cv.erode(robot_mask, kernal1, iterations=5)
#         # mask_robot = cv.dilate(mask_robot, kernal3, iterations=3)
#         # # cvShow(robot_mask, 'robot mask')

#         # _, robotContours, robotHierarchy = cv.findContours(
#         #     mask_robot, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
#         # # robotContours = sorted(robotContours, key=cv.contourArea, reverse=True)[:1]
#         # robotContours = max(robotContours, key=cv.contourArea)
#         # x, y, w, h = cv.boundingRect(robotContours)
#         # row, col = np.where(mask_robot > 0)
#         # robot_center = (int(x+w/2), int(y+h/2))
#         # # print(robot_center)
#         # robot_radius = 30
#         # eframe = cv.cvtColor(hsv, cv.COLOR_HSV2RGB)
#         # eframe = cv.circle(eframe, robot_center, robot_radius, GOOD_YELLOW, 3)

#     else:
#         break

# cap.release()
# cv.destroyAllWindows()

# print("[war_machines_PhaseC] Finished Robot Tracking!")
print("[war_machines_PhaseC] Program finished successfully!")