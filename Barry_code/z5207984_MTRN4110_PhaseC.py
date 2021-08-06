import cv2
import cv2.aruco
import numpy as np
import math
import imutils

img = cv2.imread('../Maze.png')
robot = cv2.imread('../Robot.png',0)

purple_list = []

# Convert to HSV
rgb_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
hsv_img = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2HSV)

# Cyan HSV values
lower_cyan = np.array([85,100,200])
upper_cyan = np.array([95,150,255])

# Purple HSV values
lower_purple = np.array([145,170,200])
upper_purple = np.array([155,255,255])

# Wall HSV values
lower_wall = np.array([16, 20, 220])
upper_wall = np.array([22, 120, 240])

# Robot HSV values
lower_robot = np.array([22,0,0])
upper_robot = np.array([106,224,199])

# Ladybug HSV values
lower_ladybug = np.array([170,120,125])
upper_ladybug = np.array([180,255,255])

# Mask for each colour
mask_cyan = cv2.inRange(hsv_img,lower_cyan,upper_cyan)
mask_purple = cv2.inRange(hsv_img,lower_purple,upper_purple)

# Get contours for cyan
cnts_cyan = cv2.findContours(mask_cyan,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
cnts_cyan = imutils.grab_contours(cnts_cyan)

# Get contours for purple
cnts_purple = cv2.findContours(mask_purple,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
cnts_purple = imutils.grab_contours(cnts_purple)


# Get Centre points for each colour
for c in cnts_cyan:
    area_cyan = cv2.contourArea(c)

    cv2.drawContours(img,[c],-1,(0,0,255),2)
    
    if area_cyan > 1:
        M = cv2.moments(c)

        cx_cyan = int(M["m10"]/ M["m00"])
        cy_cyan = int(M["m01"]/ M["m00"])

        
for c in cnts_purple:
    area_purple = cv2.contourArea(c)
    
    cv2.drawContours(img,[c],-1,(0,255,0),2)
    
    if area_purple > 1:
        M = cv2.moments(c)

        cx_purple = int(M["m10"]/ M["m00"])
        cy_purple = int(M["m01"]/ M["m00"])
        
        purple_list.append(cx_purple)
        purple_list.append(cy_purple)


# Perspective Transform
dist1 = math.hypot(purple_list[0] - cx_cyan, purple_list[1] - cy_cyan)
dist2 = math.hypot(purple_list[2] - cx_cyan, purple_list[3] - cy_cyan)
dist3 = math.hypot(purple_list[4] - cx_cyan, purple_list[5] - cy_cyan)
minimum = min(dist1,dist2,dist3)
sorted_purple = []

# Sort purple points to SW,SE,NE
if minimum == dist1:
    sorted_purple.append(purple_list[0])
    sorted_purple.append(purple_list[1])
    sorted_purple.append(purple_list[2])
    sorted_purple.append(purple_list[3])
    sorted_purple.append(purple_list[4])
    sorted_purple.append(purple_list[5])
elif minimum == dist2:
    sorted_purple.append(purple_list[2])
    sorted_purple.append(purple_list[3])
    if dist1 > dist3:
        sorted_purple.append(purple_list[0])
        sorted_purple.append(purple_list[1])
        sorted_purple.append(purple_list[4])
        sorted_purple.append(purple_list[5])
    else:
        sorted_purple.append(purple_list[4])
        sorted_purple.append(purple_list[5])
        sorted_purple.append(purple_list[0])
        sorted_purple.append(purple_list[1])
elif minimum == dist3:
    sorted_purple.append(purple_list[4])
    sorted_purple.append(purple_list[5])
    sorted_purple.append(purple_list[2])
    sorted_purple.append(purple_list[3])
    sorted_purple.append(purple_list[0])
    sorted_purple.append(purple_list[1])

pts1 = np.float32([[cx_cyan,cy_cyan],[sorted_purple[0],sorted_purple[1]],[sorted_purple[2],sorted_purple[3]],[sorted_purple[4],sorted_purple[5]]])
pts2 = np.float32([[0,0],[0,500],[900,500],[900,0]])
matrix = cv2.getPerspectiveTransform(pts1,pts2)
result = cv2.warpPerspective(img, matrix, (900,500))
flipped = None
sum_cyan = cx_cyan + cy_cyan
sum_purple1 = sorted_purple[0] + sorted_purple[1]
sum_purple2 = sorted_purple[2] + sorted_purple[3]
sum_purple3 = sorted_purple[4] + sorted_purple[5]
if max(sum_cyan, sum_purple1,sum_purple2,sum_purple3) == sum_cyan:
    flipped = True
# Create mask for top of walls
grey = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
hsv_result = cv2.cvtColor(result, cv2.COLOR_BGR2HSV)
mask_wall = cv2.inRange(hsv_result,lower_wall,upper_wall)

# Get height and width of mask
height, width = mask_wall.shape

# Change all white pixels in mask to blue in result
for h in range(height):
    for w in range(width):
        if mask_wall[h][w] == 255:
            result[h][w] = [255,0,0]
            

# Iterate over each tile to find robot
mask_robot = cv2.inRange(hsv_result,lower_robot,upper_robot)
cnts_robot = cv2.findContours(mask_robot,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
cnts_robot = imutils.grab_contours(cnts_robot)

for c in cnts_robot:
    area_robot = cv2.contourArea(c)
    
    if area_robot > 10:
        M = cv2.moments(c)

        cx_robot = int(M["m10"]/ M["m00"])
        cy_robot = int(M["m01"]/ M["m00"])
        
cv2.circle(result, (cx_robot,cy_robot),20,[0,0,255],2)

ladybug_crop = hsv_result[20:480,0:900].copy()
mask_ladybug = cv2.inRange(ladybug_crop,lower_ladybug ,upper_ladybug)
cnts_ladybug = cv2.findContours(mask_ladybug ,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
cnts_ladybug = imutils.grab_contours(cnts_ladybug)
area_list = []
ladybug_list = []
for c in cnts_ladybug:
    area_ladybug = cv2.contourArea(c)
    
    if area_ladybug > 10:
        area_list.append(area_ladybug)
        M = cv2.moments(c)

        cx_ladybug = int(M["m10"]/ M["m00"])
        cy_ladybug = int(M["m01"]/ M["m00"])
        ladybug_list.append(cx_ladybug)
        ladybug_list.append(cy_ladybug)
        
if area_list[0] < area_list[1]:
    cx_ladybug = ladybug_list[0]
    cy_ladybug = ladybug_list[1]

cv2.circle(result, (cx_ladybug,cy_ladybug),20,[0,255,0],2)
cv2.putText(result,'X', (cx_ladybug-9,cy_ladybug+10), cv2.FONT_HERSHEY_SIMPLEX, 1, [0,255,0],3)
dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_1000)
parameters =  cv2.aruco.DetectorParameters_create()
markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(robot, dictionary, parameters=parameters)
markerCoords = []
heading = ' '
smallestx = 0
smallesty = 0
for c in markerCorners:
    for i in c:
        for j in i:
            markerCoords.append(j)
x_coor = []
y_coor = []
front_coor = markerCoords[0]
for c in markerCoords:
    x_coor.append(c[0])
    y_coor.append(c[1])
smallestx = min(x_coor)
largestx = max(x_coor)
smallesty = min(y_coor)
largesty = max(y_coor)
if smallestx == front_coor[0]:
    if flipped == True:
        heading = '>'
    else:
        heading = '<'
elif largestx == front_coor[0]:
    if flipped == True:
        heading = '<'
    else:
        heading = '>'
elif smallesty == front_coor[1]:
    if flipped == True:
        heading = 'v'
    else:
        heading = '^'
elif largesty == front_coor[1]:
    if flipped == True:
        heading = '^'
    else:
        heading = 'v'

cv2.putText(result,heading, (cx_robot-9,cy_robot+10), cv2.FONT_HERSHEY_SIMPLEX, 1,[0,0,255],2)

# Initialise empty map with borders
map_text = np.empty([11,37],dtype=str)

for j in range(11):
    for i in range(37):
        if j == 0 or j == 10:
            if i not in {0,4,8,12,16,20,24,28,32,36}:
                map_text[j][i] = '-'
            else:
                map_text[j][i] = ' '
        elif i == 0 or i == 36:
            if j not in {0,2,4,6,8,10}:
                map_text[j][i] = '|'
        else:
            map_text[j][i] = " "

# Place robot and ladybug           
robot_coordX = int(cx_robot/100)
robot_coordY = int(cy_robot/100)
ladybug_coordX = int(cx_ladybug/100)
ladybug_coordY = int(cy_ladybug/100)

map_text[1+robot_coordY*2][2+robot_coordX*4] = heading
map_text[1+ladybug_coordY*2][2+ladybug_coordX*4] = 'x'


# Wall Placement
for row in range(1,10):
    for col in range(1,18):
        if mask_wall[50*row-1][50*col-1] == 255:
            if row not in {2,4,6,8}:
                map_text[row][col*2] = '|'
        if mask_wall[50*row-1][50*col-1] == 255:
            if col not in {2,4,6,8,10,12,14,16,18}:
                map_text[row][col*2] = '-'
                map_text[row][col*2+1] = '-'
                map_text[row][col*2+2] = '-'

f = open("../Map.txt","w+")
np.savetxt("../Map.txt",map_text,fmt='%s',delimiter='')

cv2.imshow('Task 1 - RGB',rgb_img)
cv2.imshow('Final transform',result)


cv2.waitKey(0)
cv2.destroyAllWindows()

