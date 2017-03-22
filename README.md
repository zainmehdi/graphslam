# GraphSLAM

## Rationale

This is the last practical session of the course. We will be demonstrating graphSLAM on a real robot. 

You will be mostly playing with the robot, tuning some parameters, and investigating some improvements. For this, we will do the following:

1. Start the robot and perform an initial demo run. Explain the main commands and visual rendering that we will use in the experiments.

2. Explore the limits of the current implementation:
  - See how fast we can go, or how fast we can turn, before the algorithm shows signs of poor robustness.
  - See how close to the obstacles, or how far, we can go
  - See how often we create keyframes
  - See how often we close loops, and at which distances from other keyframes
  - See how long we can make a loop and close it successfully. Here, we will try a long loop along a circular corridor, if available.
  
3. Tune the main algorithm parameters. See their effect on the relevant evaluation subjects of point 2. above.

4. See if we can improve some parts of the code to obtain better results in point 2. above.

## Installation

    git clone git@github.com:davidswords/GraphSLAM.git
    
## Execution

In one terminal:

    cd GraphSLAM
    catkin_make
    roslaunch common graphSLAM.launch
    
In a second terminal
    
    rosrun teleop_twist_keyboard teleop_twist_keyboard.py 
    
and click 'Z' 9 times to ensure that the angular speed is somwhere close to 0.4 rad/s, or smaller.

Drive your robot using the teleop keys, and see the trajectory and map being created. If you experience bad results, lower the angular velocity and start over.


