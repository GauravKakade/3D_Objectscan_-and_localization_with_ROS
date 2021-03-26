# 3D Objectscan and localization with ROS

## Copy files to respective devices

Copy the object_detection folder, final_position_generation.py file to any folder in Ubuntu 18.04 [Remote PC]. 
Copy the zelle_searcher package in the catkin workspace and do catkin make [Remote PC].
Copy the SendframesfromJetson.py to your Jetson Nano.

## Libraries installations

In the Remote PC you will need the following packages for running the object detection code: cv2, numpy, pyrealsense2, ensemble_boxes (pip install ensemble-boxes or https://github.com/ZFTurbo/Weighted-Boxes-Fusion), rospy, vidgear, and imagiz.

In Jetson Nano you need vidgear, pyrealsense2, numpy, cv2, imagiz, rospy


## Steps for Implementation

Step 1) Open a Terminator. Create four cells. 
In first cell run roscore. 
In second SSH Jetson Nano. 
In third cell SSH raspberry pi from the TurtleBot and run the TurtleBot_bringup package
In fourth cell run the Navigation stack with the Map.

Step 3) Open a new terminal and run the detect_sides.py from the object_detection folder

Step 4) Run the SendframesfromJetson.py code in the SSH cell obtained from the Jetson in Terminator in step 1.

By now the frame transfer should have started and also the object_detection shall now work.

Step 5) Run the zelle_searcher ROS package. 

The TurtleBot shall now start navigating in the lab. 

Step 6) Open a terminal and run the final_position_generation.py code.

This code will generate a static broadcaster and a end goal as soon as there is a detection. 



