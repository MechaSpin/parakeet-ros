/*
	Copyright 2021 OpenJAUS, LLC (dba MechaSpin). Subject to the MIT license.
*/

#include <parakeet/Driver.h>
#include "ros/ros.h"

#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"

#define NODE_NAME "parakeet_ros_talker"
#define degreeToRadianCoeff M_PI / 180

#define GET_PARAM(param, optional)                                                                  \
if(!nodeHandle.getParam(#param, param) && !optional)           \
{                                                                                                   \
    std::cout << "Unable to read param /" << #param << std::endl;                                   \
    return -2;                                                                                      \
}

#define SEND_DEBUG_MESSAGE(message)                                                                 \
{                                                                                                   \
    std_msgs::String debugMsg;                                                                      \
    debugMsg.data = message;                                                                        \
                                                                                                    \
    debugMessagePublisher.publish(debugMsg);                                                        \
}

ros::Publisher rosNodePublisher;
ros::Publisher debugMessagePublisher;
uint32_t sequenceID = 0;

void onPointsReceived(const mechaspin::parakeet::ScanDataPolar& scanData)
{
    if(scanData.getPoints().size() < 1)
    {
        return;
    }

    SEND_DEBUG_MESSAGE("Publishing LaserScan with " + std::to_string(scanData.getPoints().size()) + " points");

    sensor_msgs::LaserScan msg;
    msg.header.seq = sequenceID++;

    auto timeSinceEpoch = scanData.getTimestamp().time_since_epoch();
    int sec = std::chrono::duration_cast<std::chrono::seconds>(timeSinceEpoch).count();
    int nanoSec = std::chrono::duration_cast<std::chrono::nanoseconds>(timeSinceEpoch).count();
    msg.header.stamp = ros::Time(sec, nanoSec);

    msg.header.frame_id = "laser";

    float minAngle = 2*M_PI;
    float maxAngle = -2*M_PI;
    uint16_t minRange = 65535;
    uint16_t maxRange = 0;

    for(auto point : scanData.getPoints())
    {
        float angle = point.getAngleInDegrees() * degreeToRadianCoeff;
        if(angle > maxAngle)
        {
            maxAngle = angle;
        }

        if(angle < minAngle)
        {
            minAngle = angle;
        }

        float range = point.getRange();
        if(range > maxRange)
        {
            maxRange = range;
        }

        if(range < minRange)
        {
            minRange = range;
        }

        msg.ranges.push_back(range);
        msg.intensities.push_back(point.getIntensity());
    }

    msg.angle_min = minAngle;
    msg.angle_max = maxAngle;
    msg.range_min = minRange;
    msg.range_max = maxRange;

    msg.angle_increment = (maxAngle - minAngle) / scanData.getPoints().size();

    msg.scan_time = 0;

    rosNodePublisher.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);

    ros::NodeHandle nodeHandle("~");

    std::string laserScanTopic;
    GET_PARAM(laserScanTopic, true);

    rosNodePublisher = nodeHandle.advertise<sensor_msgs::LaserScan>(laserScanTopic == ""?"scan":laserScanTopic, 1000);
    debugMessagePublisher = nodeHandle.advertise<std_msgs::String>("debug_msgs", 1000);

    //Get params from paremeter server
    // ex: rosrun parakeet_ros parakeet_ros_talker _port:="/dev/ttyUSB0" _baudrate:=500000 _intensityData:=true _scanningFrequency_Hz:=10 _dataSmoothing:=false _dragPointRemoval:=false
    std::string port;
    int baudrate;
    bool intensityData;
    int scanningFrequency_Hz;
    bool dataSmoothing;
    bool dragPointRemoval;

    GET_PARAM(port, false);
    GET_PARAM(baudrate, false);
    GET_PARAM(intensityData, false);
    GET_PARAM(scanningFrequency_Hz, false);
    GET_PARAM(dataSmoothing, false);
    GET_PARAM(dragPointRemoval, false);

    std::vector<mechaspin::parakeet::Driver::ScanningFrequency> allFrequencies =
    {
        mechaspin::parakeet::Driver::ScanningFrequency::Frequency_7Hz,
        mechaspin::parakeet::Driver::ScanningFrequency::Frequency_10Hz,
        mechaspin::parakeet::Driver::ScanningFrequency::Frequency_15Hz
    };

    bool validFrequency = false;
    for(auto frequency : allFrequencies)
    {
        if (static_cast<int>(frequency) == scanningFrequency_Hz)
        {
            validFrequency = true;
            break;
        }
    }

    if(!validFrequency)
    {
        throw std::runtime_error("Invalid Scanning Frequency specified.");
    }

    mechaspin::parakeet::Driver::ScanningFrequency enumScanningFrequency_Hz = static_cast<mechaspin::parakeet::Driver::ScanningFrequency>(scanningFrequency_Hz);

    ros::Rate loop_rate(10);

    mechaspin::parakeet::Driver driver;

    if(!driver.connect(port, mechaspin::parakeet::BaudRate(baudrate), intensityData, enumScanningFrequency_Hz, dataSmoothing, dragPointRemoval))
    {
        SEND_DEBUG_MESSAGE("Failure to connect to: " + port + ".");

        return -1;
    }

    driver.registerScanCallback(onPointsReceived);

    driver.start();

    ros::spin();

    SEND_DEBUG_MESSAGE("Shutting down.");

    driver.stop();
}