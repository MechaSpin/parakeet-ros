/*
	Copyright 2021 OpenJAUS, LLC (dba MechaSpin). Subject to the MIT license.
*/

#ifndef PARAKEET_ROS_ROSNODE_H
#define PARAKEET_ROS_ROSNODE_H

#include <parakeet/Pro/Driver.h>
#include <parakeet/ProE/Driver.h>
#include <parakeet/util.h>

#include <ros/ros.h>

#include <chrono>

namespace mechaspin
{
namespace parakeet
{
class ROSNode
{
public:
    ROSNode();

    void run();
private:
    void sendROSDebugMessage(const std::string& debugMessage);

    Pro::Driver::SensorConfiguration readSerialSensorConfigurationFromParameterServer();
    ProE::Driver::SensorConfiguration readEthernetSensorConfigurationFromParameterServer();
    bool isValidScanningFrequency_HzValue(int scanningFrequency_Hz);

    void onPointsReceived(const mechaspin::parakeet::ScanDataPolar& scanData);

    ros::Publisher rosNodePublisher;
    ros::Publisher debugMessagePublisher;
    uint32_t sequenceID;
    std::string laserScanFrameID;
    std::chrono::system_clock::time_point timeReceivedLastPoints;
    ros::NodeHandle nodeHandle;

    int extraLatencyDelay_ns = 0;
    int scanningFrequency_Hz;
};
}
}

#endif