# README #

This package allows to extract from an occupancy grid map frontier points, grouping them into regions. 
Frontier points and centroids of the regions are published into ROS topics.

### Requirements ###

* OpenCV
* [ROS](http://www.ros.org/)


### Installation ###

* Clone the repository in your catkin workspace
* Compile with ``catkin_make``

### How to test it? ###
Run the frontierExample node providing as input an occupancy grid map.

In a terminal type:

``$ rosrun frontier_based_exploration frontierExample occupancyMap.jpg ``

It will create 2 output images, one containing the frontier points, one containing the frontier regions and their centroids.
Frontier points and centroids are also published into ROS topics.

NOTE: It's important to set the thresholdSize and the threhsholdNeighbors for the construcor.
