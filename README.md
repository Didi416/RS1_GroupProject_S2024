# RS1_GroupProject_S2024
Robotics Studio 1 Project - Robotic Guide Dog (Spot)

Collaborators:
- Dyandra Prins
- Claire Matthews
- Jake Nockles 
- Angelyn Co

The use of robots as a guide dog is a revolutionary concept to provide visually impaired clients with an accessible and reliable solution when guide dogs are not an option. With the ability to map and localise the environment using various sensors such as cameras, infra-red and laser, a high complexity robot like Boston Dynamics’ Spot can be programmed to improve independence and mobility through everyday life. 
The development of Spot the Robot Dog integrates simultaneous localisation and mapping algorithms (SLAM) to navigate and react with the environment around it and its client. Spot the Robot Dog will provide the visually impaired community with an alternative solution to the traditional guide dog, therefore increasing accessibility and reliability. This project will provide an opportunity for users to access individual support when they couldn’t before. This includes any circumstances where allergies and physical boundaries have restricted one’s ability to own a guide dog. 

To run rs1_group_package ensure that you have run a colcon build in the ros2_ws directory and also run a source bash on each terminal:
```
ros2 launch rs1_group_project rs1_group_project.launch.py
```
And then in two other terminals (with a source bash) run the userInput and objectDetection nodes (executables):
```
ros2 run rs1_group_project userInput
```
```
ros2 run rs1_group_project detect
```
Turtlebot can now be viewed in both Gazebo and RVIZ already localised in position. Use the userInput node to manually navigate, using w, a, and d, around the environment and/or use the number 1-10 to navigate to a predefined location on the map. The path the robot will take can be visualised using the Global Planner topic in RVIZ. 
When in front of an object, enter 'b' into the userInput node to check that there is a bin in front of the user. If there is, the objectDetection node terminal will display the coordinates of the bin. 

________________________________________________________________

To run the cylinder detection node (for SLO 3.5), simply enter:
```
ros2 launch cylinder_package cylinder_detect.launch.py
```
The robot will spawn and localise on the map and automatically start moving towards a predetermined location. It will run into the cylinder, detect it, drive around it in a circle and then continue on to the goal point. 
