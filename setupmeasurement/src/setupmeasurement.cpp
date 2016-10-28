/*
 * Olalekan Ogunmolu
 * [ogunmolu@amazon.com] | <<ecs.utdallas.edu/~olalekan.ogunmolu>>
 * 
 * Licensed under The MIT License.
 * Redistributions of files must retain the above copyright notice.
 *
 * January 18, 2016
 */
#include <GoSdk/GoSdk.h>
 
#include <ros/ros.h>

#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <signal.h>

#define SENSOR_IP			    "192.168.1.10"		

std::string nodename = "setupmeasure";

void mySigintHandler(int sig)
{
  ROS_WARN("Shutdown Signal Received. Shutting down ROS!");

  ros::shutdown();
}

void start()
{
	ROS_INFO("=================================================================");
	ROS_INFO("               GoSDK ROS Bridge: Set up Measurement              ");
	ROS_INFO("               Bridge to the GoSDK version 4.3.3                 ");
	ROS_INFO("        Code by Olalekan Ogunmolu <<ogunmolu@amazon.com>>        ");
	ROS_INFO("              	      Amazion Robotics License                              ");
	ROS_INFO("=================================================================");
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, nodename, ros::init_options::NoSigintHandler);
	ros::NodeHandle nh;

	signal(SIGINT, mySigintHandler);

	start();

	kAssembly api = kNULL;
	kStatus status;	
	GoSystem system = kNULL;
	GoSensor sensor = kNULL;
	//GoProfilePositionX positionX = kNULL;
	kIpAddress ipAddress;
	GoSetup setup = kNULL;
	GoTools tools = kNULL;
	GoProfilePosition profilePositionTool = kNULL;
	GoProfilePositionZ profileMeasurementTopZ =kNULL;
	GoProfileFeature profileFeatureTop = kNULL;
	GoProfileRegion regionTop = kNULL;
	GoOutput outputModule = kNULL;
	GoEthernet ethernetOutput = kNULL;

	// construct Gocator API Library
	if ((status = GoSdk_Construct(&api)) != kOK)
	{
		ROS_INFO("Error: GoSdk_Construct:%d", status);
	}

	// construct GoSystem object
	if ((status = GoSystem_Construct(&system, kNULL)) != kOK)
	{
		ROS_INFO("Error: GoSystem_Construct:%d", status);		
	}

	// Parse IP address into address data structure
	kIpAddress_Parse(&ipAddress, SENSOR_IP);

	// obtain GoSensor object by sensor IP address
	if ((status = GoSystem_FindSensorByIpAddress(system, &ipAddress, &sensor)) != kOK)
	{
		ROS_INFO("Error: GoSystem_FindSensorByIpAddress:%d", status);
	}

	// create connection to GoSensor object
	if ((status = GoSensor_Connect(sensor)) != kOK)
	{
		ROS_INFO("Error: GoSensor_Connect:%d", status);
	}

	// enable sensor data channel
	if ((status = GoSystem_EnableData(system, kTRUE)) != kOK)
	{
		ROS_INFO("Error: GoSensor_EnableData:%d", status);
	}
	
	// retrieve setup handle
	if ((setup = GoSensor_Setup(sensor)) == kNULL)
	{
		ROS_INFO("Error: GoSensor_Setup: Invalid Handle");
	}	

	// retreive tools handle
	if ((tools = GoSensor_Tools(sensor)) == kNULL)
	{
		ROS_INFO("Error: GoSensor_Tools: Invalid Handle");
	}

	// add ProfilePosition tool, retreive tool handle
	if ((status = GoTools_AddTool(tools, GO_TOOL_PROFILE_POSITION, &profilePositionTool)) != kOK)
	{
		ROS_INFO("Error: GoTools_AddTool:%d", status);
	}

	if ((status = GoTool_SetName(profilePositionTool, "Profile position Test")) != kOK)
	{
		ROS_INFO("Error: GoTools_AddTool: Invalid Handle");
	}

	// add Z measurement for ProfilePosition tool
	if ((profileMeasurementTopZ = GoProfilePosition_ZMeasurement(profilePositionTool)) == kNULL)
	{
		ROS_INFO("Error: GoProfilePosition_ZMeasurement: Invalid Handle");
	}

	// enable zprofileMeasurementTop
	if ((status = GoMeasurement_Enable(profileMeasurementTopZ, kTRUE)) != kOK)
	{
		ROS_INFO("Error: GoMeasurement_Enable:%d", status);
	}

	// set the measurement ID for zprofileMeasurementTop
	if ((status = GoMeasurement_SetId(profileMeasurementTopZ, 0)) != kOK)
	{
		ROS_INFO("Error: GoMeasurement_SetId:%d", status);
	}

	// set the name for zprofileMeasuermentTop
	if ((status = GoMeasurement_SetName(profileMeasurementTopZ, "Profile Measurement Z")) != kOK)
	{
		ROS_INFO("Error: GoMeasurement_SetName:%d", status);
	}

	// set ProfilePosition feature to top
	if ((profileFeatureTop = GoProfilePosition_Feature(profilePositionTool)) == kNULL)
	{
		ROS_INFO("Error: GoProfilePosition_Feature: Invalid Handle");
	}

	if ((status = GoProfileFeature_SetType(profileFeatureTop, GO_PROFILE_FEATURE_TYPE_MAX_Z)) != kOK)
	{
		ROS_INFO("Error: GoProfileFeature_SetType:%d", status);
	}

	// set the ROI to fill the entire active area
	if ((regionTop = GoProfileFeature_Region(profileFeatureTop)) == kNULL)
	{
		ROS_INFO("Error: GoProfileFeature_Region: Invalid Handle");
	}
	if ((status = GoProfileRegion_SetX(regionTop, GoSetup_TransformedDataRegionX(setup, GO_ROLE_MAIN))) != kOK)
	{
		ROS_INFO("Error: GoProfileRegion_SetX:%d", status);
	}
	if ((status = GoProfileRegion_SetZ(regionTop, GoSetup_TransformedDataRegionZ(setup, GO_ROLE_MAIN))) != kOK)
	{
		ROS_INFO("Error: GoProfileRegion_SetZ:%d", status);
	}
	if ((status = GoProfileRegion_SetHeight(regionTop, GoSetup_TransformedDataRegionHeight(setup, GO_ROLE_MAIN))) != kOK)
	{
		ROS_INFO("Error: GoProfileRegion_SetHeight:%d", status);
	}
	if ((status = GoProfileRegion_SetWidth(regionTop, GoSetup_TransformedDataRegionWidth(setup, GO_ROLE_MAIN))) != kOK)
	{
		ROS_INFO("Error: GoProfileRegion_SetWidth:%d", status);
	}

	// enable Ethernet output for measurement tool	

	if ((outputModule = GoSensor_Output(sensor)) == kNULL)
	{
		ROS_INFO("Error: GoSensor_Output: Invalid Handle");
	}
	if ((ethernetOutput = GoOutput_Ethernet(outputModule)) == kNULL)
	{
		ROS_INFO("Error: GoOutput_Ethernet: Invalid Handle");
	}
	if ((status = GoEthernet_ClearAllSources(ethernetOutput)) != kOK)
	{
		ROS_INFO("Error: GoEthernet_ClearAllSources:%d", status);
	}
	if ((status = GoEthernet_AddSource(ethernetOutput, GO_OUTPUT_SOURCE_MEASUREMENT, 0)) != kOK)
	{
		ROS_INFO("Error: GoEthernet_AddSource:%d", status);
	}

	// refer to ReceiveMeasurement.c for receiving of the measurement data.

	// destroy handles
	GoDestroy(system);
	GoDestroy(api);

	ROS_INFO("Press any key to continue...");
	getchar();
	
}