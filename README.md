# GraphSLAM

## Rationale

This is the last practical session of the course. We will be demonstrating graphSLAM on a real robot. 

You will be mostly playing with the robot, tuning some parameters, and investigating some improvements. The main goal is to explore the limits of the current implementation, for example:
  - See how fast we can go, or how fast we can turn, before the algorithm shows signs of poor robustness.
  - See how close to the obstacles, or how far, we can go
  - See how often we create keyframes
  - See how often we close loops, and at which distances from other keyframes
  - See how big we can make a loop and close it successfully. Here, we will try a long loop along a circular corridor, if available.
  
For this, we will do the following:

1. Start the robot and perform an initial run, collecting a couple of rosbags: one easy, one more challenging. We will explain the main commands and visual rendering that we will use in the experiments.

2. Use the rosbags in you own laptops to try to build nice maps. For this, you need to tune the main algorithm parameters. See their effect on the relevant evaluation subjects above.

  These parameters are:
  
    - In scanner.cpp:
      - const double fitness_keyframe_threshold
      - const double fitness_loop_threshold
      - const double distance_threshold
      - const double rotation_threshold
      - const unsigned int loop_closure_skip
      - const double sigma_xy, sigma_th
      
    - in graph.cpp:
      - int keyframes_to_skip_in_loop_closing

3. Once you are satisfied with your parameter set, you will be able to try them on the real robot.

4. If time allows, we'll see if we can improve some parts of the code to obtain better results.

## Installation

    $ git clone git@github.com:davidswords/GraphSLAM.git

Edit ~/.bashrc: add line `source ~/[...]/GraphSLAM/devel/setup.sh`
    

## Execution

### With the simulator

In one terminal:

    $ cd GraphSLAM
    $ catkin_make
    
    $ roslaunch common graphSLAM.launch
    
In a second terminal
    
    $ rosrun teleop_twist_keyboard teleop_twist_keyboard.py 
    
and click `Z` and/or `C` 9 times to ensure that the angular speed is somewhere close to 0.4 rad/s, or smaller.

Drive your robot using the `teleop` keys, and see the trajectory and map being created. If you experience bad results, lower the angular velocity and start over (you need to start over both the graphSLAM and the teleop processes).

### With a real pre-acquired rosbag

In terminal 1:

    $ cd GraphSLAM
    $ catkin_make

In terminal 2:
 
    $ roscore
    
In terminal 3:

    $ rosbag play subset.bag

In terminal 1:

    $ roslaunch common robot-graphSLAM.launch

