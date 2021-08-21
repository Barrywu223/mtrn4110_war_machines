# mtrn4110_war_machines

Welcome to the War Machines Project!

We have 3 main sections to view:

## Keyboard controller 

Simply open the controller world, and use the WASD keys to control the robot.

## Explorer

Open any of the provided world files, and the robot will explore to generate a map, labelling the middle of the maze as the target location. The robot saves this as a map so that Phase A and B can make an efficient path to the centre.

## Integrated Project

Phases A, B and C combined into one. Phase C acts as the controller, so running Phase C will open Webots and run Phase A and B too. While running, it will record the Webots screen automatically, shut down Webots when the recording is finished, and then replay the path with OpenCV used to track the robots path. Please run Phase C from the program directory, using the provided requirements file, and specifying which example world to run (possible are options are world1, world2, world3, world4 or world5).