/*
 * Olalekan Ogunmolu
 * [ogunmolu@amazon.com] | <<ecs.utdallas.edu/~olalekan.ogunmolu>>
 * 
 * Licensed under The MIT License.
 * Redistributions of files must retain the above copyright notice.
 *
 * January 14, 2016
 */

#include <ros/ros.h>

#include <GoSdk/GoSdk.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>

 #include <signal.h>

#define RECEIVE_TIMEOUT			(20000000) 
#define INVALID_RANGE_16BIT		((signed short)0x8000)			// gocator transmits range data as 16-bit signed integers. 0x8000 signifies invalid range data.	
#define DOUBLE_MAX				((k64f)1.7976931348623157e+308)	// 64-bit double - largest positive value.	
#define INVALID_RANGE_DOUBLE	((k64f)-DOUBLE_MAX)				// floating point value to represent invalid range data.	
#define SENSOR_IP			    "192.168.1.10"						

#define NM_TO_MM(VALUE) (((k64f)(VALUE))/1000000.0)
#define UM_TO_MM(VALUE) (((k64f)(VALUE))/1000.0)

 std::string nodename = "measure";

 void mySigintHandler(int sig);

 void mySigintHandler(int sig)
 {
   ROS_WARN("Shutdown Signal Received. Shutting down ROS!");

   ros::shutdown();
 }

 void start()
 {
 	ROS_INFO("=================================================================");
 	ROS_INFO("                      GoSDK ROS Measurement Profiler             ");
 	ROS_INFO("               Retrieves the measurements from the profile       ");
 	ROS_INFO("              Code by Olalekan Ogunmolu <<ogunmolu@amazon.com    ");
 	ROS_INFO("              	      MIT License                              ");
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
	unsigned int i, j, k;
	GoSystem system = kNULL;
	GoSensor sensor = kNULL;
	GoDataSet dataset = kNULL;
	GoStamp *stamp = kNULL;
	//GoProfilePositionX positionX = kNULL;
	GoDataMsg dataObj;
	kIpAddress ipAddress;
	GoMeasurementData *measurementData = kNULL;

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
	
	// refer to SetupMeasurement for set up of the measurement tool

	// start Gocator sensor
	if ((status = GoSystem_Start(system)) != kOK)
	{
		ROS_INFO("Error: GoSystem_Start:%d", status);		
	}

	if (GoSystem_ReceiveData(system, &dataset, RECEIVE_TIMEOUT) == kOK)
	{		 	
		ROS_INFO("Data message received:"); 
		ROS_INFO_STREAM("Dataset count: " << GoDataSet_Count(dataset));
		// each result can have multiple data items
		// loop through all items in result message
		for (i = 0; i < GoDataSet_Count(dataset); ++i)
		{			
			dataObj = GoDataSet_At(dataset, i);
			//Retrieve GoStamp message
			switch(GoDataMsg_Type(dataObj))
			{
			case GO_DATA_MESSAGE_TYPE_STAMP:
				{
					GoStampMsg stampMsg = dataObj;

					ROS_INFO_STREAM("Stamp Message batch count: " << GoStampMsg_Count(stampMsg));  
					for (j = 0; j < GoStampMsg_Count(stampMsg); ++j)
					{
						stamp = GoStampMsg_At(stampMsg, j);
						ROS_INFO("Timestamp: %llu", stamp->timestamp);
						ROS_INFO("Encoder: %lld", stamp->encoder); 
						ROS_INFO("Frame index: %llu", stamp->frameIndex);						
					}
				}
				break;

			case GO_DATA_MESSAGE_TYPE_MEASUREMENT:			
				{
					GoMeasurementMsg measurementMsg = dataObj;

					ROS_INFO_STREAM("Measurement Message batch count:" << GoMeasurementMsg_Count(measurementMsg)); 

					for (k = 0; k < GoMeasurementMsg_Count(measurementMsg); ++k)
					{
						measurementData = GoMeasurementMsg_At(measurementMsg, k);
						ROS_INFO("Measurement ID: %u", GoMeasurementMsg_Id(measurementMsg));
						ROS_INFO("Measurement Value: %.1f", measurementData->value);
						ROS_INFO("Measurement Decision: %d", measurementData->decision);
					}	
				}	
				break;
				
			}
		}
		GoDestroy(dataset);
	}
	else
	{
		ROS_INFO ("Error: No data received during the waiting period");
	}

	// stop Gocator sensor
	if ((status = GoSystem_Stop(system)) != kOK)
	{
		ROS_INFO("Error: GoSystem_Stop:%d", status);		
	}

	// destroy handles
	GoDestroy(system);
	GoDestroy(api);

	ROS_INFO("Press [ENTER] key to continue...");
	getchar();	

	return 0;
}
