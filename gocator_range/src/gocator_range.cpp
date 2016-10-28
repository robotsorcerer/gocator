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
#include <ros/console.h>
#include <GoSdk/GoSdk.h>
#include <stdio.h>
#include <stdlib.h>
 #include <signal.h>
#include <memory.h>
 #include <string>

#define RECEIVE_TIMEOUT			(20000000) 						//wait for 20 secs before timeout
#define INVALID_RANGE_16BIT		((signed short)0x8000)			// gocator transmits range data as 16-bit signed integers. 0x8000 signifies invalid range data.	
#define DOUBLE_MAX				((k64f)1.7976931348623157e+308)	// 64-bit double - largest positive value.	
#define INVALID_RANGE_DOUBLE	((k64f)-DOUBLE_MAX)				// floating point value to represent invalid range data.	
#define SENSOR_IP			    "192.168.1.10"						

#define NM_TO_MM(VALUE) (((k64f)(VALUE))/1000000.0)
#define UM_TO_MM(VALUE) (((k64f)(VALUE))/1000.0)

std::string nodename = "ranger";
std::string godatamsg;

void mySigintHandler(int sig)
{
  ROS_WARN("Shutdown Signal Received. Shutting down the ROS Node!");

  ros::shutdown();
}

;;

void start()
{
	ROS_INFO("==========================================================");
	ROS_INFO("                 GoSDK ROS_Range Profiler!               ");
	ROS_INFO("                                                          ");
	ROS_INFO(" Connect to Gocator system and receive range data in Range\
	           Mode and translate to engineering units (mm).  \
	           Ethernet output for range data must be enabled. Ethernet/IP \
	           is protocol used for PLC, Gocator 1x00 single point \
	           sensors only                                             ");
	ROS_INFO("                                                          ");
	ROS_INFO("Code by Olalekan Ogunmolu <<ogunmolu@amazon.com>>  | ecs.utdallas.edu/~olalekan.ogunmolu ");
	ROS_INFO("                 MIT License                              ");
	ROS_INFO("==========================================================");
}

void help()
{
	ROS_INFO_STREAM( "\n\n" 
	                 <<  std::setw('0') << std::setw(35)  <<"Error ******************************************************************\n"
	                 <<  std::setw('0') << std::setw(35)  << "Usage: rosrun gocator_bridge gocator_ranger <GoDataMsgType>            \n"
	                 <<  std::setw('0') << std::setw(35)  << "where GoDataMsgType(dataObj) could be any of <GO_DATA_MESSAGE_TYPE_STAMP>\n"
	                 <<  std::setw('0') << std::setw(35)  << "or <GO_DATA_MESSAGE_TYPE_RANGE>\n or <GO_DATA_MESSAGE_TYPE_RANGE_INTENSITY>");
}


typedef struct
{
	double x;	// x-coordinate in engineering units (mm) - position along laser line
	double z;	// z-coordinate in engineering units (mm) - height (at the given x position)
	unsigned char intensity;
}ProfilePoint;

int main(int argc, char **argv)
{
	ros::init(argc, argv, nodename, ros::init_options::NoSigintHandler);
	ros::NodeHandle nh;

	signal(SIGINT, mySigintHandler);

	start();

	if (argc != 2)
	  {
	    help();			//show user what's up :)
	  }

	godatamsg = argv[1];

	kAssembly api = kNULL;
	kStatus status;
	unsigned int i, j, k;
	GoSystem system = kNULL;
	GoSensor sensor = kNULL;
	GoDataSet dataset = kNULL;
	ProfilePoint* profileBuffer = NULL;	
	GoStamp *stamp =kNULL;
	GoDataMsg dataObj;
	kIpAddress ipAddress;

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
			ROS_INFO_STREAM("dataObj: " << dataObj);
			//Retrieve GoStamp message
			switch(GoDataMsg_Type(dataObj))
			{
			case GO_DATA_MESSAGE_TYPE_RANGE:			
				{					
					GoRangeMsg rangeMsg = dataObj;

					ROS_INFO_STREAM("Range Message batch count:" <<  GoRangeMsg_Count(rangeMsg)); 

					for (k = 0; k < GoRangeMsg_Count(rangeMsg); ++k)
					{						
						short* data = GoRangeMsg_At(rangeMsg, k); 						
						double ZResolution = NM_TO_MM(GoRangeMsg_ZResolution(rangeMsg));
						double ZOffset = UM_TO_MM(GoRangeMsg_ZOffset(rangeMsg));
						double pointZ; 
						
						if (data[k] != INVALID_RANGE_16BIT )
						{
							pointZ =  ZOffset + ZResolution * *data;
						}
						else
						{
							pointZ = INVALID_RANGE_DOUBLE;
						}
						ROS_INFO_STREAM("ZResolution: " << ZResolution);
						ROS_INFO_STREAM("Zoffset: "     << ZOffset);
						ROS_INFO_STREAM("ZPoint:  "     << pointZ);
					}
				}
				break;		

			case GO_DATA_MESSAGE_TYPE_STAMP:
				{
					GoStampMsg stampMsg = dataObj;

					ROS_INFO_STREAM("Stamp Message batch count: " << GoStampMsg_Count(stampMsg));  
					for (j = 0; j < GoStampMsg_Count(stampMsg); ++j)
					{
						stamp = GoStampMsg_At(stampMsg, j);
						ROS_INFO("  Timestamp: %llu", stamp->timestamp);
						ROS_INFO("  Encoder: %lld", stamp->encoder); 
						ROS_INFO("  Frame index: %llu", stamp->frameIndex);												
					}
				}
				break;

			case GO_DATA_MESSAGE_TYPE_RANGE_INTENSITY:			
				{
					//unsigned int validPointCount = 0;
					GoRangeIntensityMsg intensityMsg = dataObj;
					ROS_INFO_STREAM("Intensity Message batch count: " << GoRangeIntensityMsg_Count(intensityMsg)); 

					for (k = 0; k < GoRangeIntensityMsg_Count(intensityMsg); ++k)
					{
						unsigned char* data = GoRangeIntensityMsg_At(intensityMsg, k);						
						unsigned char intensity = *data;
						ROS_INFO_STREAM("Range intensity data: " << data);
						ROS_INFO_STREAM("Actual Intensity: " << intensity);
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
	free(profileBuffer);

	ROS_INFO("Press any key to continue...");
	getchar();

	return 0;
	
}