**Line Following Robot with Obstacle Avoidance**

**Overview**  
This project implements a line-following robot using the E-puck model in Webots. The primary objectives are to enable the robot to follow a defined path using a PID controller and avoid obstacles using proximity sensors. The robot detects the line through infrared sensors and responds to any obstacles it encounters.

**Features**  
Line Following: The robot uses a PID controller to keep on track by adjusting motor speeds based on line position.
Obstacle Detection: Proximity sensors enable the robot to detect obstacles and stop or change course accordingly.
Customizable Settings: Adjustable PID constants allow for fine-tuning the robot’s line-following accuracy.

**Components and Requirements**  
Motors: Two DC motors control the left and right wheels.
Infrared Sensors: Five infrared sensors detect the line position.
Proximity Sensors: Two proximity sensors detect obstacles in the robot's path.
Controller: Webots' Python API is used to control the robot’s sensors and actuators.

**Learning Experience**  
Developing this project was a hands-on introduction to robotics. Although the robot can follow a line and detect obstacles, improvements are needed in PID tuning for smoother performance.

**License**  
This project is open-source and available under the MIT License.
