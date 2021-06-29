/*
	Copyright 2021 OpenJAUS, LLC (dba MechaSpin). Subject to the MIT license.
*/

#include <algorithm>
#include <parakeet/Driver.h>
#include <parakeet/util.h>
#include "ros/ros.h"

#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"

#define NODE_NAME "parakeet_ros_talker"

#define GET_PARAM(param, optional)                                                                  \
if(!nodeHandle.getParam(#param, param) && !optional)                                                \
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

const uint32_t PARAKEET_SENSOR_MIN_RANGE_MM = 0;
const uint32_t PARAKEET_SENSOR_MAX_RANGE_MM = 50000;
const double NANOSECONDS_IN_SECOND = 1000000000;

ros::Publisher rosNodePublisher;
ros::Publisher debugMessagePublisher;
uint32_t sequenceID = 0;
std::string laserScanFrameID;
std::chrono::system_clock::duration timeReceivedLastPoints;

void onPointsReceived(const mechaspin::parakeet::ScanDataPolar& scanData)
{
    int numberOfPointsReceived = scanData.getPoints().size();

    if(numberOfPointsReceived < 1)
    {
        return;
    }

    SEND_DEBUG_MESSAGE("Publishing LaserScan with " + std::to_string(numberOfPointsReceived) + " points");

    sensor_msgs::LaserScan laserScanMessage;
    laserScanMessage.header.frame_id = laserScanFrameID;
    laserScanMessage.header.seq = sequenceID++;

    auto currentTimeSinceEpoch = scanData.getTimestamp().time_since_epoch();

    laserScanMessage.scan_time = (double)std::chrono::duration_cast<std::chrono::nanoseconds>(currentTimeSinceEpoch - timeReceivedLastPoints).count() / NANOSECONDS_IN_SECOND;
    laserScanMessage.time_increment = laserScanMessage.scan_time / numberOfPointsReceived;

    int currentTimeSinceEpochSeconds = std::chrono::duration_cast<std::chrono::seconds>(currentTimeSinceEpoch).count();
    int currentTimeSinceEpochNanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(currentTimeSinceEpoch).count() - (currentTimeSinceEpochSeconds * NANOSECONDS_IN_SECOND);
    laserScanMessage.header.stamp = ros::Time(currentTimeSinceEpochSeconds, currentTimeSinceEpochNanoseconds);

    laserScanMessage.header.frame_id = "laser";

    float minAngle_rad = mechaspin::parakeet::util::degreesToRadians(scanData.getPoints()[0].getAngle_deg());
    float maxAngle_rad = mechaspin::parakeet::util::degreesToRadians(scanData.getPoints()[numberOfPointsReceived - 1].getAngle_deg());

    laserScanMessage.angle_min = minAngle_rad;
    laserScanMessage.angle_max = maxAngle_rad;
    laserScanMessage.range_min = PARAKEET_SENSOR_MIN_RANGE_MM;
    laserScanMessage.range_max = PARAKEET_SENSOR_MAX_RANGE_MM;

    for(auto point : scanData.getPoints())
    {
        laserScanMessage.ranges.push_back(point.getRange_mm() / 1000);
        laserScanMessage.intensities.push_back(point.getIntensity());
    }

    laserScanMessage.angle_increment = -(maxAngle_rad - minAngle_rad) / numberOfPointsReceived;

    rosNodePublisher.publish(laserScanMessage);

    timeReceivedLastPoints = currentTimeSinceEpoch;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    timeReceivedLastPoints = std::chrono::system_clock::now().time_since_epoch();

    ros::NodeHandle nodeHandle("~");

    std::string laserScanTopic;
    GET_PARAM(laserScanTopic, true);
    GET_PARAM(laserScanFrameID, true);
    laserScanFrameID = laserScanFrameID == "" ? "laser" : laserScanFrameID;

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