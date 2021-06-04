# Parakeet ROS Node Building and Running using Terminal
#### 1. Install the parakeet-sdk following the [Building and Installing](https://github.com/MechaSpin/parakeet-sdk/blob/main/docs/Building%20and%Installing.md) instructions
#### 2. Clone parakeet-ros into your catkin workspace folder (~/catkin_ws), the cloned directory will be refered to as {PARAKEET_ROS_ROOT}

```
git clone https://github.com/MechaSpin/parakeet-ros
```

#### 3. Install [ROS](http://wiki.ros.org/ROS/Installation)
#### 4. Building via Terminal
- Navigate to your catkin workspace folder 

```
cd ~/catkin_ws
```

- Build the catkin workspace

```
catkin_make
source devel/setup.bash
```


#### 5a. Running via roslaunch
- A example roslaunch file has been created and can be found in {PARAKEET_ROS_ROOT}/Launch/Example.launch

```
roslaunch {PARAKEET_ROS_ROOT}/Launch/Example.launch
```

- Note: this launch file will launch the ROS node with the default runtime parameters, use the Example.launch to create a launch file more fitting to your needs. Information on each parameter can be found in [Runtime Parameters](Runtime%20Parameters.md)

#### 5a. Running via rosrun
- Initialize the runtime parameters using rosparam (look at [Runtime Parameters](Runtime%20Parameters.md))
- Running the ROS Node via rosrun requires two terminals (Terminal A and Terminal B)
- On Terminal A:

```
roscore
```

- On Terminal B:

```
rosrun parakeet_ros parakeet_ros_talker
```