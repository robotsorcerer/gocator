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
#include <ros/xmlrpc_manager.h>
#include <geometry_msgs/Point.h>

#include <GoSdk/GoSdk.h>
#include <gocator_bridge/maxon.h> 
 const std::string viewer_id = "Video_Clouds"; //must be here so visualizer can see it
#include <gocator_bridge/Visualizer.h>

#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include <memory.h>
#include <iostream>
#include <vector>
#include <string.h>
#include <sstream>
#include <fstream>
#include <math.h>

#include <opencv2/opencv.hpp>

/*Global variables*/
const std::string nodename = "video";
unsigned int i, j;

bool rgb(false);
// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

boost::mutex mutex;

//function prototypes
void mySigintHandler(int sig);

void mySigintHandler(int sig)
{
  ROS_WARN("Shutdown Signal Received. Shutting down ROS!");
  g_request_shutdown = 1;

 //we stop because we do not want to wait for currently running service to stop before we shut down the node.
  ros::shutdown(); 
}

// Replacement "shutdown" XMLRPC callback
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  int num_params = 0;
  if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
    num_params = params.size();
  if (num_params > 1)
  {
    string reason = params[1];
    ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
    g_request_shutdown = 1; // Set flag
  }

  result = ros::xmlrpc::responseInt(1, "", 0);
}

void wait(int seconds)
{
  boost::this_thread::sleep (boost::posix_time::seconds(seconds));
}

void lock(){mutex.lock();}
void unlock(){mutex.unlock();}

void start_mover(motor_motion* move)
{
	boost::lock_guard<boost::mutex> guard(mutex);
	if((lResult = move->start_motion(&ulErrorCode))==MMC_SUCCESS)
	{
		ROS_INFO_STREAM("starting motion " <<", " << lResult << ", " << ulErrorCode);
	}
}

void end_mover(motor_motion* move)
{
	if((lResult = move->end_motion(&ulErrorCode))==MMC_SUCCESS)
	{
		ROS_INFO_STREAM("starting motion " <<", " << lResult << ", " << ulErrorCode);
	}
}

pcl::PointCloud<pcl::PointXYZ>::Ptr retrieve_video(int argc, char** argv, \
													GoSystem system,  GoDataSet dataset, Visualizer* viz)
{	
	boost::lock_guard<boost::mutex> guard (mutex);
	uint8_t r(255), g(15), b(15);
	pcl::PointCloud<pcl::PointXYZ>::Ptr video_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>& video_cloud = *video_cloud_ptr;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr video_rgb_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
	k8u* videoBuffer = NULL;
	k8u* data = kNULL;

	if (GoSystem_ReceiveData(system, &dataset, RECEIVE_TIMEOUT) == kOK && ros::ok())
	{
		k32u videoBufferHeight = 0;
		k32u videoBufferWidth = 0;

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
					ROS_INFO_STREAM("Stamp Message batch count: "<< GoStampMsg_Count(stampMsg));
					for (j = 0; j < GoStampMsg_Count(stampMsg); ++j)
					{
						stamp = GoStampMsg_At(stampMsg, j);
						ROS_INFO("  Timestamp: %llu", stamp->timestamp);
						ROS_INFO("  Encoder position at leading edge: %lld", stamp->encoder);
						ROS_INFO("  Frame index: %llu", stamp->frameIndex);
					}
				}
				break;

				case GO_DATA_MESSAGE_TYPE_VIDEO:
				{
					GoVideoMsg videoMsg = dataObj;		
					unsigned int rowIdx, colIdx;
					unsigned int cameraIdx, filtArray, exposure;
					pcl::PointXYZ videoPixels;
					pcl::PointXYZRGB videoRGBPixels;

					videoBufferHeight = GoVideoMsg_Height(videoMsg);
					videoBufferWidth  = GoVideoMsg_Width(videoMsg);
					ROS_INFO("Video buffer height: %u", videoBufferHeight);
					ROS_INFO("Video buffer width: %u", videoBufferWidth);

					cameraIdx = GoVideoMsg_CameraIndex(videoMsg);
					filtArray = GoVideoMsg_Cfa(videoMsg);	
					exposure = GoVideoMsg_Exposure(videoMsg);
					ROS_INFO("Video originating from camera: %u, with exposure %u, 'uS", cameraIdx, exposure);
					ROS_INFO("Image color filter array: %u", filtArray);

					//kSize videoPixelSize = GoVideoMsg_PixelSize(videoMsg);	

					for(rowIdx=0; rowIdx < videoBufferHeight; ++rowIdx)
					{					
						data = (k8u*) GoVideoMsg_RowAt(videoMsg, rowIdx);
						//memcpy(&videoBuffer[rowIdx*videoBufferWidth], data, (videoBufferWidth * videoPixelSize));
						
						for(colIdx=0; colIdx < videoBufferWidth; ++colIdx)
						{
							videoPixels.x  = colIdx;
							videoPixels.y  = rowIdx;
							videoPixels.z = data[colIdx]; //videoBuffer[rowIdx*videoBufferWidth+colIdx];
							//ROS_INFO_STREAM("videoPixels.z: " << videoPixels.z);
							video_cloud_ptr->points.push_back(videoPixels);

							videoRGBPixels.x = videoPixels.x;
							videoRGBPixels.y = videoPixels.y;
							videoRGBPixels.z = videoPixels.z;
							uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
							        static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));

							videoRGBPixels.rgb = *reinterpret_cast<int*>(&rgb);
							video_rgb_ptr->points.push_back(videoRGBPixels);
						}
						video_cloud_ptr->width = video_rgb_ptr->width = videoBufferWidth;
						video_cloud_ptr->height = video_rgb_ptr->height = videoBufferHeight;
						video_cloud_ptr->is_dense = video_rgb_ptr->is_dense = true;  //does it contain NaN points
						//video_cloud_ptr->points.resize(video_cloud_ptr->height * video_cloud_ptr->width);
					}
				}
				break;
			default:
				break;
			}	//close switch
		}
		GoDestroy(dataset);
		//free memory arrays
		if (videoBuffer) 
		{
			free(videoBuffer);
		}
	}		//close if

	return video_cloud_ptr;
	//return video_rgb_ptr;
}

int main(int argc, char **argv)
{
	bool running(false);
	bool updateCloud(false);
	bool save(false);

	GoSystem system = kNULL;
	GoDataSet dataset = kNULL;
	GoSensor sensor = kNULL;
	GoSetup setup = kNULL;

	GoMode 	video_mode = GO_MODE_VIDEO;

	//Override SIGINT handler
	ros::init(argc, argv, nodename, ros::init_options::NoSigintHandler);	
	signal(SIGINT, mySigintHandler);

	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("video_pixels", 100);
	
	// Override XMLRPC shutdown
	ros::XMLRPCManager::instance()->unbind("shutdown");
	ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);

	Visualizer viz;
	motor_motion move;

	viz.help();

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
		ROS_INFO("Error: GoSensor_Start:%d", status);		
	}

	// retrieve setup handle
	if ((setup = GoSensor_Setup(sensor)) == kNULL)
	{
		ROS_INFO("Error: GoSensor_Setup: Invalid Handle");
	}	

	if((status = GoSetup_SetScanMode(setup, video_mode)) != kOK)
	{
		ROS_INFO("Error: GoSetup_SetScanMode, %d", status);
	}


	if (GoSetup_UniformSpacingEnabled(setup))
	{
		//We always go to role main since there is no buddy sensor.
		profilePointCount = GoSetup_XSpacingCount(setup, GO_ROLE_MAIN);
		ROS_INFO_STREAM("X spacing count is " << profilePointCount);
	}
	else
	{
		// Get the camera ROI width 
		profilePointCount = GoSetup_FrontCameraWidth(setup, GO_ROLE_MAIN);
		ROS_INFO_STREAM("Region of interest width (pixels): " << profilePointCount);
	}


	ROS_INFO("Waiting for video data ... \n\n");


/*	move.SetDefaultParameters();

	if((lResult = move.OpenDevice(&ulErrorCode))!=MMC_SUCCESS && !ros::ok())
	{
		ROS_ERROR_STREAM("OpenDevice" <<", " << lResult <<", " << ulErrorCode);
		return lResult;
	}*/

	if (ros::ok()) {running = true;}

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;	
	pcl::PointCloud<pcl::PointXYZ>::Ptr video_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr videorgb_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	viewer = viz.createViewer();	
	viewer->setSize(1280, 1300);
	pcl::PCDWriter writer;

	for(; running && !viewer->wasStopped() ;)
	{
		//Follow C++ API convention and cast the return of malloc to ProfilePoint to avoid glibc errors
/*		if ((videoBuffer = (SurfacePoint*)malloc(profilePointCount * sizeof(SurfacePoint))) == kNULL)
		{
			ROS_ERROR("Error: Cannot allocate Surface Data, %d points", profilePointCount);		
		}*/

/*		if((lResult = move.prepare_motion(&ulErrorCode))!=MMC_SUCCESS)
		{
			ROS_ERROR_STREAM("could not prepare motion " <<", " << lResult << ", " << ulErrorCode);
			// return lResult;
		}	*/

/*		if((lResult = move.go_home(&ulErrorCode))!=MMC_SUCCESS && !ros::ok())
		{
			ROS_ERROR_STREAM("Going home" <<", " << lResult <<", " << ulErrorCode);
			return lResult;
		}*/

		for(int i = 0; i < 5; ++i)
		{
			video_cloud_ptr = retrieve_video(argc, argv, system, dataset, &viz);
			ROS_INFO_STREAM("video_cloud_ptr " << *video_cloud_ptr);

			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> video_color_handler (video_cloud_ptr, 255, 0, 255);
			viewer->addPointCloud<pcl::PointXYZ>(video_cloud_ptr, video_color_handler, viewer_id);
			viewer->spinOnce(10);
			boost::this_thread::sleep(boost::posix_time::microseconds(100));

			if(save)
			{
				viz.saveCloud(video_cloud_ptr);
			}

/*			start_mover(&move);

			ROS_INFO_STREAM("motion iteration: " << i);

			if((lResult = move.prepare_motion(&ulErrorCode))!=MMC_SUCCESS)
			{
				ROS_ERROR_STREAM("could not prepare motion " <<", " << lResult << ", " << ulErrorCode);
			}*/

			sensor_msgs::PointCloud2 msg;
			pcl::toROSMsg(*video_cloud_ptr, msg);
			msg.header.stamp = ros::Time::now();
			msg.header.frame_id = "video_frame";
			pub.publish(msg);

			ros::Rate looper(10);

			if(updateCloud)
			{				
				viewer->removePointCloud(viewer_id);
				viewer->removeCoordinateSystem(1.0);
				viewer->updatePointCloud(video_cloud_ptr, viewer_id);
			}
			updateCloud = true;
		}			

		//writer.writeBinary("cloud.pcd", *video_cloud_ptr);
	} //close while	

	// stop Gocator sensor
	if ((status = GoSystem_Stop(system)) == kOK)
	{
		ROS_INFO(" Stopping all connected sensors: %d\n", status);
	}

	if((lResult = move.close_device(&ulErrorCode))!=MMC_SUCCESS)
	{
		ROS_INFO_STREAM("CloseDevice " << lResult << ", " <<  ulErrorCode);
	}

	std::cout << "\n" << std::endl;
	ROS_INFO("Press Ctrl + C to continue...\n");

	// destroy handles
	GoDestroy(system);
	//GoDestroy(api);
	free(videoBuffer);
	std::cout << "\n" << std::endl;
	ROS_INFO("Press the enter key to continue...\n");

	pause();

	ros::spin();

	return 0;
}