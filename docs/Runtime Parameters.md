# Parakeet ROS Node Runtime parameters

#### 1a. Modifying parameters with Launch file
- Look into the Example.launch file
- There should be several lines specifying runtime parameters which look similar to:

```
<param name="port" value="/dev/ttyUSB0" />
```

- Modifying requires changing the value from "/dev/ttyUSB0" to the new value

#### 1b. Setting parameters using rosparam
rosparam allows for parameters to be set before calling rosrun
- For each parameter needing change, we will execute the following (subbing values in for PARAMETER and PARAMETER_VALUE)

```
rosparam set parakeet_ros_talker/PARAMETER PARAMETER_VALUE
```

- ie:

```
rosparam set parakeet_ros_talker/port "/dev/ttyUSB0"
```

After setting the parameters, you can execute the ROS node via:


	rosrun parakeet_ros parakeet_ros_talker



#### 1c. Setting parameters during rosrun
rosrun can also take parameters and will pass them all to rosparam
- An execution of rosrun with setting parameters will look like:

```
rosrun parakeet_ros parakeet_ros_talker _PARAMETER1:=PARAMETER1_VALUE _PARAMETER2:=PARAMETER2_VALUE
```

- A full rosrun execution modifying each parameter will look like:

```
rosrun parakeet_ros parakeet_ros_talker _port:="/dev/ttyUSB0" _baudrate:=0 _intensityData:=true _scanningFrequency_Hz:=10 _dataSmoothing:=false _dragPointRemoval:=false
```


#  All parameters
#### port
The location of the serial port

ie: "/dev/ttyUSB0"

#### baudrate
The baudrate the sensor is currently set to

ie: 500000

A baudrate value of 0 will result in the Driver attempting to find the correct baud rate.

#### intensityData 
Whether the sensor should return intensity data with each point

accepted inputs: { true, false }

#### scanningFrequency_Hz 
The speed at which the sensor should spin at

accepted inputs: { 7, 10, 15 }

#### dataSmoothing 
Whether the sensor should apply the data smoothing algorithm before returing points

accepted inputs: { true, false }

#### dragPointRemoval 
Whether the sensor should apply the drag point removal algorithm before returing points

accepted inputs: { true, false }

#### laserScanTopic - OPTIONAL
The topic the laser scan should be published under

ie: "myCustomTopic"

default value: "scan"