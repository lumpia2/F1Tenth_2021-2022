# F1Tenth_2021-2022
Official Repository for Pitt RAS F1Tenth 2021-2022 Team 

To get "up to speed", take a look at our [Getting Started](https://docs.google.com/document/d/1MPcZuDyK1n8SSpPkCYEtRkjRHpZaNV0-Qj5Cd3Ra4kE/edit?usp=sharing) Guide

Once you've done that, make sure that you understand the topics covered each week so you can hop on a current project.

Introduction to ROS - Part 1

Intro [slides].(https://docs.google.com/presentation/d/10-BS7uOYaSVuBZqPPlhUMrAvJ8rqE7bGuX3cjUwzc6Q/edit?usp=sharing)

We then moved to ROS tutorials 1-5. This involved

1. Configuring a ROS environment
2. Creating ROS packages 
3. Building a ROS package
4. Using ROS Nodes
5. Using ROS Topics

Online tutortials can be found [here.](http://wiki.ros.org/ROS/Tutorials)

The weekly challenge is to push an image of your turtle path to the F1Tenth GitHub.

Post the image to the appropriate folder path i.e. **F1Tenth_2021-2022/media/images/turtle-sim-path**. Make sure to pull the images other have posted before you push yours. 

## 10/10 - 10/16: Introduction to ROS - Part 2

We dove a bit deeper into ROS publishers and subscribers. We looked at some example publishers and subscribers, and examined their code to see exactly how they fit into the overall ROS system. We also experimented with changing data types and contents of messages sent. 

ROS Tutorials used: 

12. Writing a Simple Publisher and Subscriber 
13. Examining the Publisher and Subscriber 

Online tutortials can be found [here.](http://wiki.ros.org/ROS/Tutorials)

The weekly challenge is similar to last week's - commit & push an image of your modified publisher (talker.py) message to the F1Tenth GitHub, as well as your subscriber (listener.py) outputting the same message.

Post the image to the appropriate folder path i.e. **F1Tenth_2021-2022/media/images/publisher-subscriber-message/**. Make sure to pull the images others have posted before you push yours. 


## 10/24 - 10/30: RGBD Camera 

We finally have our hands on some hardware - the [Intel Realsense d435](https://www.intelrealsense.com/depth-camera-d435/). We split up into 3 groups which will independently attempt to interface the camera with ROS. Ideally, we will be able to use this interface later to do some advanced image processing on the hardware of the car. 

The three groups are shown by members on each card located in the [Trello.](https://trello.com/b/ofsd7s9F/weekly-sprint) If you missed the meeting or don't have a group, feel free to hop on whichever team you'd like. The camera will be located in the RAS room, just down the hall from 1211 BNDM, our usual meeting place. The room is normally locked, but someone should be in there for most of the day. 

If you develop code that you think might be useful in the fututre, either to use or to reference, upload it to the appropriate **RGBD-Camera** directory on the GitHub. 

## 10/31 - 11/6: Intro to SLAM

We introduced SLAM - simultaneous localization and mapping. We can utilize an Extended Kalman Filter (EKF) to both localize (estimate our position) and create a map of our environment. 

Like last week, we broke into three groups located in the [SLAM Teams Trello Card](https://trello.com/b/ofsd7s9F/weekly-sprint) If you missed the meeting or don't have a group, feel free to hop on whichever team you'd like. The shop should be open, so stop by during the week to mess around and see what you can produce.

Anything that might be helpful to others should be shared via GitHub or other medium. As always, have fun with it!

## 11/7 - 11/13: Perception Presentation 

We had an excellent presentation from Adam Johnson, a senior CoE student who led the LiDAR development on the Indy Autonomous Challenge car. Slides can be accessed [here.](https://docs.google.com/presentation/d/1OpZpCFbR4MlRBM9_tBVKmbXkI15ABNUqcTTLkb2kKVQ/edit?usp=sharing)

Continue on with developing ROS connections between the camera and RTAB-Map. We've had some great success so far with getting it set up on the RAS computer in the workshop.

## 11/14-11/28

We've gotten some hardware in, and can finally begin to connect some concepts. To-do for this week
- Design & 3D-print a mechanical enclosure to hold an Intel Realsense d435 and Intel T265 parallel, co-planar, and pecisely distanced from each other at all times. This will allow us to fulfill the depth & odometry requirements of RTAB-Map.
- Create ROS code to store camera data in a ROS .bag file. This will be necessary to store data in a mobile source before creating a map from it on a desktop.
- Create ROS code to retrive camera data from a ROS.bag file and use it in RTAB map. 
- Begin building the physical car - we now have a chassis. Check out this [F1tenth build link](https://f1tenth.org/build.html) to get started. 

Push any relevant info to the GitHub. We'll ideally have a map of the 11th floor by 12/5/2021. 

