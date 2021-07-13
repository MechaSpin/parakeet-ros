/*
	Copyright 2021 OpenJAUS, LLC (dba MechaSpin). Subject to the MIT license.
*/

#include "ROSNode.h"

#include "ros/ros.h"

#include <string>

const std::string PARAKEET_NODE_NAME = "parakeet_ros_talker";

int main(int argc, char **argv)
{
    ros::init(argc, argv, PARAKEET_NODE_NAME);
    mechaspin::parakeet::ROSNode rosNode;

    rosNode.run();
}