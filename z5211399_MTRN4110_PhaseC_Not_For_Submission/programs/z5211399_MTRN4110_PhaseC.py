from typing import final
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt


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


MAZE_FILE_NAME = '../Rang/Test3/Maze_3.png'
ROBOT_FILE_NAME = '../Rang/Test3/Robot_3.png'
IMAGE_LADYBUG_FILE_NAME = '../Ladybug_small.png'
TXT_FILE_NAME = '../MapBuilt.txt'
MAZE_IMAGE_WIDTH = 1350
MAZE_IMAGE_HEIGHT = 750
ROBOT_IMAGE_WIDTH = 750
ROBOT_IMAGE_HEIGHT = 750
LIGHT_GREEN = [120, 240, 50]
LIGHT_PINK = [235, 130, 230]
LIGHT_BLUE = [240, 240, 130]
LIGHT_YELLOW = [26, 71, 255]
ORANGE = [77, 255, 255]
BLACK = [0, 0, 0]
WHITE = [255, 255, 255]
GREEN = [40, 220, 30]
BLUE = [255, 0, 0]
NORTH = 0
SOUTH = 1
WEST = 2
EAST = 3
# pre-process
maze_original = cv.imread(MAZE_FILE_NAME)
maze = maze_original.copy()
# cvShow(maze, 'maze')
robot = cv.imread(ROBOT_FILE_NAME)
ladybug = cv.imread(IMAGE_LADYBUG_FILE_NAME)
maze = resize(maze, width=MAZE_IMAGE_WIDTH, height=MAZE_IMAGE_HEIGHT)
maze_hsv = cv.cvtColor(maze, cv.COLOR_BGR2HSV)

kernal9 = np.ones((9, 9), "uint8")
kernal7 = np.ones((7, 7), "uint8")
kernal5 = np.ones((5, 5), "uint8")
kernal3 = np.ones((3, 3), "uint8")
kernal1 = np.ones((1, 1), "uint8")


# 3.1. Read in an image and display it in RGB mode
maze_rgb = cv.cvtColor(maze, cv.COLOR_BGR2RGB)
# cvShow(maze_rgb, 'Task 1')
# pltShow(maze_rgb, 'RGB Maze')


# 3.2. Find the four ordered cornerstones of the maze
pink_lower = np.array([125, 200, 200])
pink_upper = np.array([175, 255, 255])

cyan_lower = np.array([80, 120, 200])
cyan_upper = np.array([100, 140, 255])

wall_lower = np.array([17, 40, 215])
wall_upper = np.array([22, 125, 255])

robot_lower = np.array([50, 23, 123])
robot_upper = np.array([104, 167, 210])

# To find cyan(key)conerstones
cyan_mask = cv.inRange(maze_hsv, cyan_lower, cyan_upper)
cyan_mask = cv.dilate(cyan_mask, kernal3, iterations=1)
# cvShow(cyan_mask, 'cyan mask')


cyanContours, cyanHierarchy = cv.findContours(
    cyan_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

cyanContours = sorted(cyanContours, key=cv.contourArea, reverse=True)[:1]

if len(cyanContours) != 0:
    for contour in cyanContours:
        if cv.contourArea(contour) > 200:
            cv.ellipse(maze, cv.fitEllipse(contour), LIGHT_YELLOW, 3)
            x, y, w, h = cv.boundingRect(contour)
            x_cord = int((x+x+w)/2)
            y_cord = int((y+y+h)/2)
            key_center = (x_cord, y_cord)


# To find pink cornerstones

pink_mask = cv.inRange(maze_hsv, pink_lower, pink_upper)
pink_mask = cv.dilate(pink_mask, kernal3, iterations=1)
# cvShow(pink_mask, 'Pink mask')


pinkContours, pinkHierarchy = cv.findContours(
    pink_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

pinkContours = sorted(pinkContours, key=cv.contourArea, reverse=True)[:3]
if len(pinkContours) != 0:
    for contour in pinkContours:
        if cv.contourArea(contour) > 200:
            cv.ellipse(maze, cv.fitEllipse(contour), ORANGE, 3)

cvShow(maze, 'Task 2')


# 3.3. Perspective transform the maze from the original image to a rectangle image
rotation_threshold = 400
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
        print('Original image rotated too mcuh！！！')
print(center_record)
pts1 = np.float32([key_center, center_record[0],
                  center_record[1], center_record[2]])
pts2 = np.float32([[0, 0], [MAZE_IMAGE_WIDTH, 0], [0, MAZE_IMAGE_HEIGHT], [
                  MAZE_IMAGE_WIDTH, MAZE_IMAGE_HEIGHT]])
matrix = cv.getPerspectiveTransform(pts1, pts2)
maze = cv.warpPerspective(
    maze, matrix, (MAZE_IMAGE_WIDTH, MAZE_IMAGE_HEIGHT))

robot = cv.copyMakeBorder(robot, 0, 0, 300, 300, cv.BORDER_CONSTANT)
# cvShow(robot, 'nonTransfered Robot')
robot = cv.warpPerspective(
    robot, matrix, (MAZE_IMAGE_WIDTH, MAZE_IMAGE_HEIGHT))

# cvShow(robot, 'Transfered Robot')

cvShow(maze, 'Task 3')

# pltShow(maze, 'Task 3')

# 3.4 Detect all the internal walls

maze_hsv = cv.cvtColor(maze, cv.COLOR_BGR2HSV)
wall_mask = cv.inRange(maze_hsv, wall_lower, wall_upper)
# cvShow(wall_mask, 'Wall mask')
wall_mask = cv.morphologyEx(wall_mask, cv.MORPH_CLOSE, kernal7, iterations=1)
# cvShow(wall_mask, 'After close')
wall_mask = cv.erode(wall_mask, kernal3, iterations=2)
# cvShow(wall_mask, 'After erosion')
wall_mask = cv.dilate(wall_mask, kernal3, iterations=2)
# cvShow(wall_mask, 'After dilation')


wall_mask = 255-wall_mask
maze = cv.bitwise_and(maze, maze, mask=wall_mask)
Rmask = np.all(maze == BLACK,  axis=-1)
maze[Rmask] = LIGHT_GREEN
wall_mask = 255-wall_mask


# cvShow(maze, 'Task 4')

# 3.5 Detection the location and heading of the robot

dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250)

# markerImage = np.zeros((200, 200), dtype=np.uint8)
# markerImage = cv.aruco.drawMarker(dictionary, 33, 200, markerImage, 1)

robot_bgr = robot.copy()
robot_rgb_origin = cv.cvtColor(robot_bgr, cv.COLOR_BGR2RGB)
robot_rgb = cv.cvtColor(robot_bgr, cv.COLOR_BGR2RGB)

parameters = cv.aruco.DetectorParameters_create()

markerCorners, markerIds, rejectedCandidates = cv.aruco.detectMarkers(
    robot_rgb, dictionary, parameters=parameters)

robot_rgb = cv.aruco.drawDetectedMarkers(robot_rgb, markerCorners, markerIds)
acceptBias = 20.0
# print(markerCorners)
if (abs(markerCorners[0][0][0][1] - markerCorners[0][0][2][1]) < acceptBias and markerCorners[0][0][0][0] < markerCorners[0][0][2][0]):
    direction = WEST
    # print('west!')
elif (abs(markerCorners[0][0][0][1] - markerCorners[0][0][2][1]) < acceptBias and markerCorners[0][0][0][0] > markerCorners[0][0][2][0]):
    direction = EAST
    # print('east!')
elif (abs(markerCorners[0][0][0][0] - markerCorners[0][0][2][0]) < acceptBias and markerCorners[0][0][0][1] < markerCorners[0][0][2][1]):
    direction = NORTH
    # print('north!')
elif (abs(markerCorners[0][0][0][0] - markerCorners[0][0][2][0]) < acceptBias and markerCorners[0][0][0][1] > markerCorners[0][0][2][1]):
    direction = SOUTH
    # print('south!')
else:
    print('Ops! I cant recognize the direction')
    # print(markerCorners[0][0][0][0], markerCorners[0]
    #       [0][2][0], markerCorners[0][0][0][0], markerCorners[0][0][2][0])

# detect the position by aruco
# maze_bgr = maze.copy()
# maze_rgb_origin = cv.cvtColor(maze_bgr, cv.COLOR_BGR2RGB)
# maze_rgb = cv.cvtColor(maze_bgr, cv.COLOR_BGR2RGB)
# markerCorners, markerIds, rejectedCandidates = cv.aruco.detectMarkers(
#     maze_rgb, dictionary, parameters=parameters)
# print(markerIds)
# maze_rgb = cv.aruco.drawDetectedMarkers(maze_rgb, markerCorners, markerIds)
# if direction == WEST or direction == EAST:
#     robot_center = (int((markerCorners[0][0][0][0] + markerCorners[0][0][2][0])/2), int(
#         (markerCorners[0][0][1][1] + markerCorners[0][0][3][1])/2))
# elif direction == NORTH or direction == SOUTH:
#     robot_center = (int((markerCorners[0][0][1][0] + markerCorners[0][0][3][0])/2), int(
#         (markerCorners[0][0][0][1] + markerCorners[0][0][2][1])/2))

# print('robot center: ', robot_center)


# detect by circle
# robot_gray = cv.cvtColor(robot.copy(), cv.COLOR_BGR2GRAY)
# robot_gray_blurred = cv.blur(robot_gray, (3, 3))
# detected_circles = cv.HoughCircles(robot_gray_blurred,
#                                    cv.HOUGH_GRADIENT, 1, 1000, param1=100,
#                                    param2=50, minRadius=0, maxRadius=100)
# print(detected_circles)

# if detected_circles is not None:
#     # Convert the circle parameters a, b and r to integers.
#     detected_circles = np.uint16(np.around(detected_circles))
#     for pt in detected_circles[0, :]:
#         a, b, r = pt[0], pt[1], pt[2]

#         # Draw the circumference of the circle.
#         # cv.circle(robot, (a, b), r, (0, 255, 0), 2)

#         # # Draw a small circle (of radius 1) to show the center.
#         # cv.circle(robot, (a, b), 1, (0, 0, 255), 3)
#         print(a, b)
# cv.imshow("Detected Circle", robot)
# cv.waitKey(0)
# size = 250
# print(a-size, a+size)
# robot = robot[b-size:b+size, a-size:a+size]
# cvShow(robot, 'new')


# print('robot center: ', robot_center)
# print(markerCorners)
# fig, (ax1, ax2) = plt.subplots(figsize=(18, 10), ncols=2)
# ax1.imshow(robot_rgb_origin)
# ax2.imshow(robot_rgb)
# plt.show()

# detect by template

# template = robot.copy()
# cvShow(template, 's')
# w, h = template.shape[:-1]

# methods = ['cv.TM_CCOEFF', 'cv.TM_CCOEFF_NORMED', 'cv.TM_CCORR',
#            'cv.TM_CCORR_NORMED', 'cv.TM_SQDIFF', 'cv.TM_SQDIFF_NORMED']
# img = maze.copy()
# method = cv.TM_CCORR

# res = cv.matchTemplate(img, template, method)
# min_val, max_val, min_loc, max_loc = cv.minMaxLoc(res)
# if method in [cv.TM_SQDIFF, cv.TM_SQDIFF_NORMED]:
#     top_left = min_loc
# else:
#     top_left = max_loc

# bottom_right = (top_left[0] + w, top_left[1] + h)
# cv.rectangle(maze, top_left, bottom_right, 255, 2)
# cv.rectangle(maze, top_left, bottom_right, 255, 2)
# robot_center = (int(top_left[0]+w/2), int(top_left[1]+h/2))
# print(top_left[0], top_left[1])
# print(top_left, bottom_right)
# maze = cv.circle(maze, robot_center, robot_radius, (35, 225, 235), 3)

# Draw oulter circle
# robot_radius = 30
# cv.circle(maze, robot_center, robot_radius, BLUE, 3)


maze_hsv = cv.cvtColor(maze.copy(), cv.COLOR_BGR2HSV)
robot_mask = cv.inRange(maze_hsv, robot_lower, robot_upper)


robot_mask = cv.erode(robot_mask, kernal3, iterations=1)
# cvShow(robot_mask, 'robot mask')
robot_mask = cv.morphologyEx(robot_mask, cv.MORPH_CLOSE, kernal7, iterations=7)
# robot_mask = cv.erode(robot_mask, kernal1, iterations=5)
robot_mask = cv.dilate(robot_mask, kernal3, iterations=3)
cvShow(robot_mask, 'robot mask')


robotContours, robotHierarchy = cv.findContours(
    robot_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
# robotContours = sorted(robotContours, key=cv.contourArea, reverse=True)[:1]
robotContours = max(robotContours, key=cv.contourArea)
x, y, w, h = cv.boundingRect(robotContours)
row, col = np.where(robot_mask > 0)
robot_center = (int(x+w/2), int(y+h/2))
print(robot_center)
robot_radius = 30
maze = cv.circle(maze, robot_center, robot_radius, (35, 225, 235), 3)
cvShow(maze, 'dsa')


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

maze = cv.line(maze, point1, point2, (35, 225, 235), 3)
maze = cv.line(maze, point1, point3, (35, 225, 235), 3)

# Detect the position for true target
template = ladybug.copy()
w, h = template.shape[:-1]

methods = ['cv.TM_CCOEFF', 'cv.TM_CCOEFF_NORMED', 'cv.TM_CCORR',
           'cv.TM_CCORR_NORMED', 'cv.TM_SQDIFF', 'cv.TM_SQDIFF_NORMED']
img = maze.copy()
method = cv.TM_CCOEFF

res = cv.matchTemplate(img, template, method)
min_val, max_val, min_loc, max_loc = cv.minMaxLoc(res)
if method in [cv.TM_SQDIFF, cv.TM_SQDIFF_NORMED]:
    top_left = min_loc
else:
    top_left = max_loc

bottom_right = (top_left[0] + w, top_left[1] + h)
# cv.rectangle(img, top_left, bottom_right, 255, 2)
# cv.rectangle(img, top_left, bottom_right, 255, 2)
ladybug_center = (int(top_left[0]+w/2), int(top_left[1]+h/2))
# print(top_left, bottom_right)
# maze = cv.circle(maze, robot_center, robot_radius, (35, 225, 235), 3)

# Draw oulter circle
ladybug_radius = 60
cv.circle(maze, ladybug_center, ladybug_radius, BLUE, 3)

# Drae cross sign
length = 30
point1 = (ladybug_center[0]-length, ladybug_center[1]-length)
point2 = (ladybug_center[0]+length, ladybug_center[1]-length)
point3 = (ladybug_center[0]+length, ladybug_center[1]+length)
point4 = (ladybug_center[0]-length, ladybug_center[1]+length)

cv.line(maze, point1, point3, BLUE, 4)
cv.line(maze, point2, point4, BLUE, 4)


# plt.subplot(121), plt.imshow(res, cmap='gray')
# plt.title('Matching Result'), plt.xticks([]), plt.yticks([])
# plt.subplot(122), plt.imshow(img, cmap='gray')
# plt.title('Detected Point'), plt.xticks([]), plt.yticks([])
# plt.suptitle(method)
# plt.show()
cvShow(maze, 'Task 5')

# 3.6 Generate the map
# print(maze.shape)
# print(np.where(wall_mask == 0))
# pltShow(maze, 'maze')
# initialize the vertial wall matrix
rows, cols = (5, 10)
virtical_wall = np.zeros((5, 10))
# printMaxtirx(vertialWall)

# initialize the horizontal wall matrix
rows, cols = (6, 9)
horizontal_wall = np.zeros((6, 9))
# printMaxtirx(horizontalWall)

bias = 10
for i, row in enumerate(virtical_wall):

    for j, val in enumerate(row):
        if (j == 0 or j == 9):

            row[j] = 1

        elif (wall_mask[150*i+75][150*j] == 255 or wall_mask[150*i+75+bias][150*j] or wall_mask[150*i+75-bias][150*j]):

            # print('wall mask: ', 150*i,
            #       150*(j-1)+75, ' at: ', i, ' ', j)
            row[j] = 1

for i, row in enumerate(horizontal_wall):

    for j, val in enumerate(row):
        if (i == 0 or i == 5):

            row[j] = 1

        elif (wall_mask[150*i][150*j+75] or wall_mask[150*i][150*j+75+bias] or wall_mask[150*i][150*j+75-bias]):

            # print('wall mask: ', 150*i,
            #       150*(j-1)+75, ' at: ', i, ' ', j)
            row[j] = 1

# pltShow(wall_mask, 's')
# printMaxtirx(virtical_wall)
# printMaxtirx(horizontal_wall)
# print(robot_center)
robot_coordinate = (
    round((robot_center[1]-75)/150), round((robot_center[0]-75)/150))
# print(robot_coordinate)
# print(ladybug_center)
ladybug_coordinate = (
    round((ladybug_center[1]-75)/150), round((ladybug_center[0]-75)/150))
# print(ladybug_coordinate)

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

for content in message:
    str = ''
    for elements in content:
        str += elements
    str += '\n'
    map.write(str)
    print(str)

map.close()


maze_original_rgb = cv.cvtColor(maze_original.copy(), cv.COLOR_BGR2RGB)
maze_rgb = cv.cvtColor(maze.copy(), cv.COLOR_BGR2RGB)

fig, (ax1, ax2) = plt.subplots(figsize=(18, 10), ncols=2)
ax1.imshow(maze_original_rgb)
ax2.imshow(maze_rgb)
ax1.set_title('Original image')
ax2.set_title('Final image')
# plt.show()

# final_img = cv.hconcat(
#     (resize(maze_original, 675, 375), resize(maze, 675, 375)))
# cvShow(final_img, 'Final image')
