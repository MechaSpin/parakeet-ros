/*
	Copyright 2021 OpenJAUS, LLC (dba MechaSpin). Subject to the MIT license.
*/

#include "ROSNode.h"

#include <algorithm>

#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"

#define Get_Parameter_From_Parameter_Server(parameterName, outputValue, isOptional)                                 \
if(!nodeHandle.getParam(parameterName, outputValue) && !isOptional)                                                 \
{                                                                                                                   \
    throw std::runtime_error(std::string("Unable to read parameter: ") + parameterName + " from the parameter server.");  \
}

namespace mechaspin
{
namespace parakeet
{
    const uint32_t PARAKEET_SENSOR_MIN_RANGE_M = 0;
    const uint32_t PARAKEET_SENSOR_MAX_RANGE_M = 50;
    const double NANOSECONDS_IN_SECOND = 1000000000;

    ROSNode::ROSNode() : nodeHandle("")
    {
        debugMessagePublisher = nodeHandle.advertise<std_msgs::String>("parakeet_ros_debug_messages", 1000);

        std::string laserScanTopic;
        Get_Parameter_From_Parameter_Server("laserScanTopic", laserScanTopic, true);
        Get_Parameter_From_Parameter_Server("laserScanFrameID", laserScanFrameID, true);
        laserScanFrameID = laserScanFrameID == "" ? "laser" : laserScanFrameID;

        rosNodePublisher = nodeHandle.advertise<sensor_msgs::LaserScan>(laserScanTopic == ""?"scan":laserScanTopic, 1000);

        timeReceivedLastPoints = std::chrono::system_clock::now().time_since_epoch();
        sequenceID = 0;

        sendROSDebugMessage("ROSNode initialized");
    }

    void ROSNode::run()
    {
        Driver* driver;

        std::string sensorType;
        Get_Parameter_From_Parameter_Server("sensorType", sensorType, false);

        if(sensorType == "Pro")
        {
            Pro::Driver::SensorConfiguration sensorConfiguration = readSerialSensorConfigurationFromParameterServer();

            Pro::Driver* proDriver = new Pro::Driver();
            proDriver->connect(sensorConfiguration);

            driver = proDriver;
        }
        else if(sensorType == "ProE")
        {
            ProE::Driver::SensorConfiguration sensorConfiguration = readEthernetSensorConfigurationFromParameterServer();

            ProE::Driver* proEDriver = new ProE::Driver();
            proEDriver->connect(sensorConfiguration);

            driver = proEDriver;
        }
        else
        {
            throw std::runtime_error("Invalid value for \"sensorType\"");
        }

        driver->registerScanCallback(std::bind(&ROSNode::onPointsReceived, this, std::placeholders::_1));

        sendROSDebugMessage("Starting driver.");
        driver->start();

        ros::Rate loop_rate(10);
        ros::spin();

        sendROSDebugMessage("Shutting driver down.");

        driver->stop();

        delete driver;
    }

    void ROSNode::sendROSDebugMessage(const std::string& debugMessage)
    {
        std_msgs::String debugMsg;
        debugMsg.data = debugMessage;

        debugMessagePublisher.publish(debugMsg);
    }

    void ROSNode::onPointsReceived(const ScanDataPolar& scanData)
    {
        int numberOfPointsReceived = scanData.getPoints().size();

        if(numberOfPointsReceived < 1)
        {
            return;
        }

        sendROSDebugMessage("Publishing LaserScan with " + std::to_string(numberOfPointsReceived) + " points");

        sensor_msgs::LaserScan laserScanMessage;
        laserScanMessage.header.frame_id = laserScanFrameID;
        laserScanMessage.header.seq = sequenceID++;

        auto currentTimeSinceEpoch = scanData.getTimestamp().time_since_epoch();

        laserScanMessage.scan_time = (double)std::chrono::duration_cast<std::chrono::nanoseconds>(currentTimeSinceEpoch - timeReceivedLastPoints).count() / NANOSECONDS_IN_SECOND;
        laserScanMessage.time_increment = laserScanMessage.scan_time / numberOfPointsReceived;

        int currentTimeSinceEpochSeconds = std::chrono::duration_cast<std::chrono::seconds>(currentTimeSinceEpoch).count();
        int currentTimeSinceEpochNanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(currentTimeSinceEpoch).count() - (currentTimeSinceEpochSeconds * NANOSECONDS_IN_SECOND);
        laserScanMessage.header.stamp = ros::Time(currentTimeSinceEpochSeconds, currentTimeSinceEpochNanoseconds);

        laserScanMessage.header.frame_id = laserScanFrameID;

        float minAngle_rad = util::degreesToRadians(scanData.getPoints()[0].getAngle_deg());
        float maxAngle_rad = util::degreesToRadians(scanData.getPoints()[numberOfPointsReceived - 1].getAngle_deg());

        laserScanMessage.angle_min = minAngle_rad;
        laserScanMessage.angle_max = maxAngle_rad;
        laserScanMessage.range_min = PARAKEET_SENSOR_MIN_RANGE_M;
        laserScanMessage.range_max = PARAKEET_SENSOR_MAX_RANGE_M;

        for(auto point : scanData.getPoints())
        {
            laserScanMessage.ranges.push_back(point.getRange_mm() / 1000);
            laserScanMessage.intensities.push_back(point.getIntensity());
        }

        laserScanMessage.angle_increment = -(maxAngle_rad - minAngle_rad) / numberOfPointsReceived;

        rosNodePublisher.publish(laserScanMessage);

        timeReceivedLastPoints = currentTimeSinceEpoch;
    }

    bool ROSNode::isValidScanningFrequency_HzValue(int scanningFrequency_Hz)
    {
        std::vector<Driver::ScanningFrequency> allFrequencies =
        {
            Driver::ScanningFrequency::Frequency_7Hz,
            Driver::ScanningFrequency::Frequency_10Hz,
            Driver::ScanningFrequency::Frequency_15Hz
        };

        for(auto frequency : allFrequencies)
        {
            if (static_cast<int>(frequency) == scanningFrequency_Hz)
            {
                return true;
            }
        }
        return false;
    }

    Pro::Driver::SensorConfiguration ROSNode::readSerialSensorConfigurationFromParameterServer()
    {
        //Get params from paremeter server
        // ex: rosrun parakeet_ros parakeet_ros_talker _sensor:="Pro" _port:="/dev/ttyUSB0" _baudrate:=500000 _intensityData:=true _scanningFrequency_Hz:=10 _dataSmoothing:=false _dragPointRemoval:=false

        Pro::Driver::SensorConfiguration sensorConfiguration;

        Get_Parameter_From_Parameter_Server("port", sensorConfiguration.comPort, false);

        int baudRate;
        Get_Parameter_From_Parameter_Server("baudrate", baudRate, false);
        sensorConfiguration.baudRate = BaudRate(baudRate);

        Get_Parameter_From_Parameter_Server("intensityData", sensorConfiguration.intensity, false);
        
        int scanningFrequency_Hz;
        Get_Parameter_From_Parameter_Server("scanningFrequency_Hz", scanningFrequency_Hz, false);

        if(!isValidScanningFrequency_HzValue(scanningFrequency_Hz))
        {
            throw std::runtime_error("Invalid Scanning Frequency specified.");
        }

        sensorConfiguration.scanningFrequency_Hz = static_cast<Driver::ScanningFrequency>(scanningFrequency_Hz);

        Get_Parameter_From_Parameter_Server("dataSmoothing", sensorConfiguration.dataSmoothing, false);
        Get_Parameter_From_Parameter_Server("dragPointRemoval", sensorConfiguration.dragPointRemoval, false);

        return sensorConfiguration;
    }

    ProE::Driver::SensorConfiguration ROSNode::readEthernetSensorConfigurationFromParameterServer()
    {
        //Get params from paremeter server
        // ex: rosrun parakeet_ros parakeet_ros_talker _sensor:="ProE" _ipAddress:="192.168.158.98" _dstPort:=6543 _srcPort:=6668 _intensityData:=true _scanningFrequency_Hz:=10 _dataSmoothing:=false _dragPointRemoval:=false _resampleFilter:=true
        ProE::Driver::SensorConfiguration sensorConfiguration;

        Get_Parameter_From_Parameter_Server("ipAddress", sensorConfiguration.ipAddress, false);
        Get_Parameter_From_Parameter_Server("dstPort", sensorConfiguration.dstPort, false);
        Get_Parameter_From_Parameter_Server("srcPort", sensorConfiguration.srcPort, false);

        Get_Parameter_From_Parameter_Server("intensityData", sensorConfiguration.intensity, false);
        
        int scanningFrequency_Hz;
        Get_Parameter_From_Parameter_Server("scanningFrequency_Hz", scanningFrequency_Hz, false);

        if(!isValidScanningFrequency_HzValue(scanningFrequency_Hz))
        {
            throw std::runtime_error("Invalid Scanning Frequency specified.");
        }

        sensorConfiguration.scanningFrequency_Hz = static_cast<Driver::ScanningFrequency>(scanningFrequency_Hz);

        Get_Parameter_From_Parameter_Server("dataSmoothing", sensorConfiguration.dataSmoothing, false);
        Get_Parameter_From_Parameter_Server("dragPointRemoval", sensorConfiguration.dragPointRemoval, false);
        Get_Parameter_From_Parameter_Server("resampleFilter", sensorConfiguration.resampleFilter, false);
        return sensorConfiguration;
    }
}
}