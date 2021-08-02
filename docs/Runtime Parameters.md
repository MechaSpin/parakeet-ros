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
rosparam set PARAMETER PARAMETER_VALUE
```

- ie:

```
rosparam set port "/dev/ttyUSB0"
```

After setting the parameters, you can execute the ROS node via:
```
rosrun parakeet_ros parakeet_ros_talker
```

#  All parameters
#### sensorType
Which Parakeet sensor is being used

accepted inputs: { "Pro", "ProE" }

#### port
The location of the serial port, for use with only the Parakeet Pro

ie: "/dev/ttyUSB0"

#### baudrate
The baudrate the sensor is currently set to, for use with only the Parakeet Pro

ie: 500000

A baudrate value of 0 will result in the Driver attempting to find the correct baud rate.

#### ipAddress
The IP Address of the sensor, for use with only the Parakeet ProE

ie: "192.168.158.98"

#### dstPort
The port of the sensor which messages will be sent to, for use with only the Parakeet ProE

default value: 6543

#### srcPort
The port of the sensor which messages will be received from, for use with only the Parakeet ProE

default value: 6668

#### intensityData 
Whether the sensor should return intensity data with each point

accepted inputs: { true, false }

#### scanningFrequency_Hz 
The speed at which the sensor should spin at

Parakeet Pro accepted inputs: { 7, 10, 15 }
Parakeet ProE accepted inputs: { 10, 15 }

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

#### laserScanFrameID - OPTIONAL
The frame ID the laser scan should be published under

ie: "myCustomFrameID"

default value: "laser"