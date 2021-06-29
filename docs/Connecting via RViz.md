# Connecting to Parakeet ROS Node using rviz
#### 1. Run the parakeet_ros node following the instructions in [Building and Running](Building%20and%20Running.md)
#### 2. Running rviz
- Open a seperate terminal

```
rviz
```

#### 3. Connecting to parakeet_ros node
- Inside rviz, under Displays -> Global Options, set Fixed Frame to the value of the parameter: laserScanFrameID, defaults to: laser
- Near the bottom left of rviz, click Add
- Select LaserScan and click OK
- Open the LaserScan tree under the Displays section
- Specify the topic the ROS Node is publishing on (defaults to "/parakeet_ros_talker/scan")
- Note: This topic can be specified through the parameter server
- Note: If messages are being recieved, but no data is visible, consider upping the LaserScan size, and zooming out