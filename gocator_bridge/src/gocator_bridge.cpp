/******************************************************************
 * Olalekan Ogunmolu
 * [ogunmolu@amazon.com] | <<lakehanne.github.io>>
 * 
 * Licensed under The Amazon Robotics LLC License.
 * Redistributions of files must retain the above copyright notice.
 *
 * January 14, 2016
 *******************************************************************
 * This is a minimal boiler plate for "gocator_profile" and "gocator_surface"
 *  packages. It provides the necessary includes, API Defs and shared object 
 * libraries needed to compile the utility codes. For details on how to link 
 * to the libraries or use the includes, look at the CMakeLists.txt files
 *  for each sub-package within the gocator meta-package system.
 */

#include "ros/ros.h"
#include <ros/console.h>

#include <GoSdk/GoSdk.h>
#include <stdio.h>
 #include <stdlib.h>
#include <signal.h>
#include <memory.h>

#define SENSOR_IP			"192.168.1.10"

std::string nodename = "lmircvr";

void mySigintHandler(int sig)
{
  ROS_WARN("Shutdown Signal Received. Shutting down ROS!");

  ros::shutdown();
}

void start()
{
	ROS_INFO("=================================================================");
	ROS_INFO("                      GoSDK ROS Bridge                           ");
	ROS_INFO("               Bridge to the GoSDK version 4.3.3                 ");
	ROS_INFO("        Code by Olalekan Ogunmolu <<ogunmolu@amazon.com>>        ");
	ROS_INFO("              	      Amazon Robotics License                  ");
	ROS_INFO("=================================================================");
}

typedef struct
{
	k32u count;
}DataContext;

// data callback function
kStatus kCall onData(void* ctx, void* sys, void* dataset)
{
	unsigned int i, j;
	DataContext *context = (DataContext*)ctx;

	ROS_INFO("onData Callback:");
	ROS_INFO("Data message received:"); 
	ROS_INFO_STREAM("Dataset message count: " << GoDataSet_Count(dataset));
	
	for (i = 0; i < GoDataSet_Count(dataset); ++i)
	{
		GoDataMsg dataObj = GoDataSet_At(dataset, i);

		// retrieve GoStamp message
		switch(GoDataMsg_Type(dataObj))
		{
		case GO_DATA_MESSAGE_TYPE_STAMP:	
			{
				GoStampMsg stampMsg = dataObj;
				for (j = 0; j < GoStampMsg_Count(stampMsg); ++j)
				{
					GoStamp *stamp = GoStampMsg_At(stampMsg, j);
					ROS_INFO("  Timestamp: %llu", stamp->timestamp);
					ROS_INFO("  Encoder: %lld", stamp->encoder); 
					ROS_INFO("  Frame index: %llu", stamp->frameIndex);					
					context->count++;
				}
			}
			break;
			// Refer to example ReceiveRange, ReceiveProfile, ReceiveMeasurement and ReceiveWholePart on how to receive data				
		}				
	}
	GoDestroy(dataset);
	return kOK;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, nodename, ros::init_options::NoSigintHandler);
	ros::NodeHandle nh;

	signal(SIGINT, mySigintHandler);

	start();

	kAssembly api = kNULL;
	GoSystem system = kNULL;
	GoSensor sensor = kNULL;
	kStatus status;
	kIpAddress ipAddress;

	DataContext contextPointer;
	
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
		ROS_INFO("Error: GoSystem_FindSensor:%d", status);		
	}	

	// create GoSystem_Connectction to GoSystem object
	if ((status = GoSystem_Connect(system)) != kOK)
	{
		ROS_INFO("Error: GoSystem_Connect:%d", status);		
	}

	// enable sensor data channel
	if ((status = GoSystem_EnableData(system, kTRUE)) != kOK)
	{
		ROS_INFO("Error: GoSensor_EnableData:%d", status);		
	}
	
	// set data handler to receive data asynchronously
	if ((status = GoSystem_SetDataHandler(system, onData, &contextPointer)) != kOK)
	{
		ROS_INFO("Error: GoSystem_SetDataHandler:%d", status);
	}	
	
	// start Gocator sensor
	if ((status = GoSystem_Start(system)) != kOK)
	{
		ROS_INFO("Error: GoSystem_Start:%d", status);		
	}

	ROS_INFO("Press any key to stop sensor...");
	//getchar();

	int c = getchar();
	putchar(c);
	if(c == 'q')		
	{
		ROS_INFO("Shutdown Signal Received. Shutting down ROS!");
		ros::shutdown();
	}

	// stop Gocator sensor
	if ((status = GoSystem_Stop(system)) != kOK)
	{
		ROS_INFO("Error: GoSystem_Stop:%d", status);		
	}

	// destroy handles
	GoDestroy(system);
	GoDestroy(api);

	ROS_INFO("Press any key to continue...");
	getchar();
	return 0;
}