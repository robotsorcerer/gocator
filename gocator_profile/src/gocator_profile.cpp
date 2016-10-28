/*****************************************************************************************************************************
 * Author: Olalekan Ogunmolu
 * [ogunmolu@amazon.com] | <<lakehanne.github.io>>
 * 
 * Licensed under The Amazon Robotics License.
 * Redistributions of files must retain the above copyright notice.
 *
 * January 14, 2016
 ******************************************************************************************************************************
 *
 * PIPELINE 2: May 2016
 * Program is now integrated to work with maxon encoder. There is a a launch script within the gocator_profile and gocator_surface
 * package directories. Doing "roslaunch gocator_porfile profile.launch" or "roslaunch gocator_surface surface.launch would start
 * the gocator sensor on the rail. The launch file launches two executables:
 *
 *	1. The Epos controller executable which moves back and forth in velocity mode at a constant speed of 1500rpm. I have set it up
 * to do a 1500rpm motion consecutively for four iterations in the forward and backward directions respectively. I added a 3 seconds
 * delay in-between each motion to avoid race conditions. It is important to change the triggering mode to "Encoder" on the web UI.
 * Technically, you shouldn't be required to do this but I have found that the API's provided doesn't automatically change the 
 * triggering mode
 *
 * 2. The "gocator_profile" and "gocator_surface" executables listen asynchronously for encoder triggers. Whenever the motor moves, 
 * the encoder triggers and a message is sent to the sensor to acquire a frame. After each frame acquisition, the laser points are 
 * pushed into a point cloud container and an update is made on the point cloud visualizer window.
 * 
 * 3. @TODO
 * Gather profile triggers into a buffer after every end-to-end motion of the sensor and display the 3D points on the visualizer
 * 
 */
#include <ros/ros.h>

#include <GoSdk/GoSdk.h>
std::string viewer_id = "profile_view";
#include <gocator_bridge/Visualizer.h>
#include <gocator_bridge/maxon.h>

#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <string.h>
#include <iostream>
#include <math.h>
#include <vector>
#include <list>
#include <thread>

#include <unistd.h>
#include <chrono>
#include <ctime>

#include <pcl/io/pcd_grabber.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>


typedef struct
{
	ProfilePoint profileBuffer[1280];
	k32u count;
}DataContext;

bool updateCloud = false;
boost::mutex mutex;
const std::string nodename = "gocator_profile";
pcl::PointCloud<pcl::PointXYZ>::Ptr profile_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr rviz_profile_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> profile_color_handler (profile_cloud_ptr, 255, 100, 50);
pcl::PointCloud<pcl::PointXYZ>& profile_cloud = *profile_cloud_ptr;
sensor_msgs::PointCloud2 cld_msg;

pcl::PointXYZRGB rviz_colored_points;
pcl::PointXYZ profile_points, rviz_profile_points;

const std::string cloudName = "ProfileCloud";
Visualizer viz;
uint frame_idx;
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = viz.createViewer();
std::list<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudMap;

// data callback function
// std::pair<kStatus, pcl::PointCloud<pcl::PointXYZ>::Ptr>  kCall onData(void* ctx, void* sys, void* dataset)
kStatus  kCall onData(void* ctx, void* sys, void* dataset)
{	
	float x, z;
	unsigned int i, j, k, arrayIndex;
	DataContext *context = static_cast<DataContext *> (ctx);
	
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
					frame_idx = stamp->frameIndex;
					// printf("Timestamp: %llu; Encoder: %lld;  Frame idx: %llu. ", stamp->timestamp, stamp->encoder , frame_idx);
					std::cout << "\n"					 << std::endl;
					context->count++;
				}
			}
			break;	
		
		case GO_DATA_MESSAGE_TYPE_RESAMPLED_PROFILE:
			{
				GoResampledProfileMsg profileMsg = dataObj;
				// ROS_INFO_STREAM("Resampled Profile Message batch count: " << GoResampledProfileMsg_Count(profileMsg));

				unsigned int validPointCount = 0;
				
				for (k = 0; k < GoResampledProfileMsg_Count(profileMsg); ++k)
				
				{
					short* data = GoResampledProfileMsg_At(profileMsg, k);
					double XResolution = NM_TO_MM(GoResampledProfileMsg_XResolution(profileMsg));
					double ZResolution = NM_TO_MM(GoResampledProfileMsg_ZResolution(profileMsg));
					double XOffset = UM_TO_MM(GoResampledProfileMsg_XOffset(profileMsg));
					double ZOffset = UM_TO_MM(GoResampledProfileMsg_ZOffset(profileMsg));
				 	//translate 16-bit range data to engineering units and copy profiles to memory array

					for (arrayIndex = 0; arrayIndex < GoResampledProfileMsg_Width(profileMsg); ++arrayIndex)
					{ 	
						if (data[arrayIndex] != INVALID_RANGE_16BIT )
						{	
							// ROS_INFO("VALID!");
							context->profileBuffer[arrayIndex].x = XOffset + XResolution * arrayIndex;
							context->profileBuffer[arrayIndex].z = ZOffset + ZResolution * data[arrayIndex];

							//x & z are the address of the retrieved points
							x = static_cast<float>((context->profileBuffer[arrayIndex].x)/1);
							z = static_cast<float>((context->profileBuffer[arrayIndex].z)/1);

							// ROS_INFO("(x: %f,  z: %f)", x, z);

							profile_points.x = x;
							profile_points.y = 0.0;
							profile_points.z = z;
							profile_cloud_ptr-> points.push_back(profile_points);

						 	rviz_profile_points.x = x/1000;
						 	rviz_profile_points.y = 0.0;
						 	rviz_profile_points.z = z/1000;
						 	rviz_profile_cloud_ptr-> points.push_back(rviz_profile_points);

						 	validPointCount++;
						}
					 	else
					 	{	
					 		// ROS_INFO("INVALID!");
					 		context->profileBuffer[arrayIndex].x = XOffset + XResolution * arrayIndex;
					 		context->profileBuffer[arrayIndex].z = INVALID_RANGE_DOUBLE;
					 		x = static_cast<float>(context->profileBuffer[arrayIndex].x);
					 		z = static_cast<float>(context->profileBuffer[arrayIndex].z);
					 		// ROS_INFO("(x: %f,  z: %f)", x, z);
					 		profile_points.x = x;
					 		profile_points.y = 0.0;
					 		profile_points.z = z;
					 		profile_cloud_ptr-> points.push_back(profile_points);

					 	 	rviz_profile_points.x = x/1000;
					 	 	rviz_profile_points.y = 0.0;
					 	 	rviz_profile_points.z = z/1000;
					 	 	rviz_profile_cloud_ptr-> points.push_back(rviz_profile_points);
					 		validPointCount++;
						}

					}
					// ROS_INFO("Profile Valid Point %d out of max %d", validPointCount, profilePointCount);
					//unorganized point cloud with 307200 points
					profile_cloud_ptr->width 	= rviz_profile_cloud_ptr->width = (int)profile_cloud.points.size();
					profile_cloud_ptr->height 	= rviz_profile_cloud_ptr->height = 1;

					profile_cloud_ptr->is_dense = rviz_profile_cloud_ptr->is_dense = true;

					profile_cloud_ptr->points.resize(profile_cloud_ptr->height * profile_cloud_ptr->width);
					rviz_profile_cloud_ptr->points.resize(rviz_profile_cloud_ptr->height * rviz_profile_cloud_ptr->width);

					pcl::toROSMsg(*rviz_profile_cloud_ptr, cld_msg);
					cld_msg.header.stamp = ros::Time::now();
					cld_msg.header.frame_id = "profile_tf_frame";
				}
			}		//closes case statement
			break;
		
		case GO_DATA_MESSAGE_TYPE_PROFILE: 
		{
			GoProfileMsg profileMsg = dataObj;
			ROS_INFO_STREAM("Profile Message batch count: " <<  GoProfileMsg_Count(profileMsg));
			unsigned int validPointCount = 0;

			for (k = 0; k < GoProfileMsg_Count(profileMsg); ++k)
			{	
				kPoint16s* data = GoProfileMsg_At(profileMsg, k);
				double XResolution = NM_TO_MM(GoProfileMsg_XResolution(profileMsg));
				double ZResolution = NM_TO_MM(GoProfileMsg_ZResolution(profileMsg));
				double XOffset = UM_TO_MM(GoProfileMsg_XOffset(profileMsg));
				double ZOffset = UM_TO_MM(GoProfileMsg_ZOffset(profileMsg));
				//translate 16-bit range data to engineering units and copy profiles to memory array

				for (arrayIndex = 0; arrayIndex < GoProfileMsg_Width(profileMsg); ++arrayIndex)
				{	
					if (data[arrayIndex].x != INVALID_RANGE_16BIT)
					{	
						// ROS_INFO("Valid Data Received! \n");
						context->profileBuffer[arrayIndex].x = XOffset + XResolution * data[arrayIndex].x;
						context->profileBuffer[arrayIndex].z = ZOffset + ZResolution * data[arrayIndex].y;					
						x = static_cast<float>(context->profileBuffer[arrayIndex].x);
						z = static_cast<float>(context->profileBuffer[arrayIndex].z);

						profile_points.x = x;
						profile_points.y = 0.0;
						profile_points.z = z;
						profile_cloud_ptr-> points.push_back(profile_points);

						rviz_profile_points.x = x/1000;
						rviz_profile_points.y = 0.0;
						rviz_profile_points.z = z/1000;
						rviz_profile_cloud_ptr-> points.push_back(rviz_profile_points);
						 //save points to csv file

						validPointCount++;
						// ROS_INFO("(x: %f,  z: %f)", x, z);
					}
					else
					{ 	
						// ROS_INFO("Invalid Data Received! \n");
						context->profileBuffer[arrayIndex].x = INVALID_RANGE_DOUBLE;
						context->profileBuffer[arrayIndex].z = INVALID_RANGE_DOUBLE;
						x = static_cast<float>(context->profileBuffer[arrayIndex].x);
						z = static_cast<float>(context->profileBuffer[arrayIndex].z);
						// ROS_INFO("INVALID: (x: %f,  z: %f)", x, z);
						profile_points.x = x;
						profile_points.y = 0.0;
						profile_points.z = z;
						profile_cloud_ptr-> points.push_back(profile_points);

						rviz_profile_points.x = x/1000;
						rviz_profile_points.y = 0.0;
						rviz_profile_points.z = z/1000;
						rviz_profile_cloud_ptr-> points.push_back(rviz_profile_points);
						validPointCount++;
					}
				}
				// ROS_INFO("  Profile Valid Point %d out of max %d", validPointCount, profilePointCount);
				profile_cloud_ptr->width 	= rviz_profile_cloud_ptr->width = (int)profile_cloud.points.size();
				profile_cloud_ptr->height 	= rviz_profile_cloud_ptr->height = 1;

				profile_cloud_ptr->is_dense = rviz_profile_cloud_ptr->is_dense = true;

				profile_cloud_ptr->points.resize(profile_cloud_ptr->height * profile_cloud_ptr->width);
				rviz_profile_cloud_ptr->points.resize(rviz_profile_cloud_ptr->height * rviz_profile_cloud_ptr->width);
				
				sensor_msgs::PointCloud2 cld_msg;
				pcl::toROSMsg(*rviz_profile_cloud_ptr, cld_msg);
				cld_msg.header.stamp = ros::Time::now();
				cld_msg.header.frame_id = "profile_tf_frame";
			}
		}
		break;		
		}	
	}
	GoDestroy(dataset);
	return kOK;
}

void trans(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, float y)
{ 
	float theta = 0; 
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
	transform.translation() << 0.0, y, 0.0;   //translate along x
	pcl::transformPointCloud(*cloud_in, *cloud_out, transform);  
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, nodename);
	ros::NodeHandle pr;
	ros::Publisher pub = pr.advertise<sensor_msgs::PointCloud2>("profile_points", 1);
	std::string mode;
	pr.getParam("mode", mode);

	ROS_INFO("Started node %s", ros::this_node::getName().c_str());

	kAssembly api = kNULL;
	GoSystem system = kNULL;
	GoSensor sensor = kNULL;
	GoSetup setup = kNULL;
	GoTrigger trigger_mode = kNULL;

	kStatus status;
	kIpAddress ipAddress;

	DataContext contextPointer;
	bool running = false;	

	// construct Gocator API Library
	if ((status = GoSdk_Construct(&api)) != kOK)
	{
		ROS_ERROR("Error: GoSdk_Construct:%d ", status);
		// return 0;
	}
	
	// construct GoSystem object
	if ((status = GoSystem_Construct(&system, kNULL)) != kOK)
	{
		ROS_ERROR("Error: GoSystem_Construct:%d ", status);
	}

	// Parse IP address into address data structure
	kIpAddress_Parse(&ipAddress, SENSOR_IP);

	// obtain GoSensor object by sensor IP address
	if ((status = GoSystem_FindSensorByIpAddress(system, &ipAddress, &sensor)) != kOK)
	{
		ROS_ERROR("Error: GoSystem_FindSensor:%d ", status);
		return 0;
	}	

	// create connection to GoSystem object
	if ((status = GoSystem_Connect(system)) != kOK)
	{
		ROS_ERROR("Error: GoSystem_Connect:%d ", status);
		return 0;
	}

	// enable sensor data channel
	if ((status = GoSystem_EnableData(system, kTRUE)) != kOK)
	{
		ROS_ERROR("Error: GoSensor_EnableData:%d ", status);
	}
	
	// set data handler to receive data asynchronously
	if ((status = GoSystem_SetDataHandler(system, onData, &contextPointer)) != kOK)
	{
		ROS_ERROR("Error: GoSystem_SetDataHandler:%d ", status);
	}	
	
	if(!mode.compare("async"))
	{		
		ROS_INFO_STREAM("Operating in mode async mode");
	}

	if ((setup = GoSensor_Setup(sensor)) == kNULL)
	{
		ROS_ERROR("Error: GoSensor_Setup: Invalid Handle ");
		if ((setup = GoSensor_Setup(sensor)) == kNULL)
		{
			ROS_ERROR("Error: GoSensor_Setup: Invalid Handle ");
		}
	}

	if ((trigger_mode = GoSetup_TriggerSource(setup)) != GO_TRIGGER_ENCODER)
	{
		if((status = GoSetup_SetTriggerSource(setup, GO_TRIGGER_ENCODER)) != kOK)
		{
			if ((setup = GoSensor_Setup(sensor)) == kNULL)
			{
				ROS_ERROR("Error: GoSetup_SetTriggerSource: %d ", status);
			}
		}
	}
	else 
	{	
		((trigger_mode = GoSetup_TriggerSource(setup))!= GO_TRIGGER_TIME);
		if((status = GoSetup_SetTriggerSource(setup, GO_TRIGGER_TIME)) != kOK)
		{
			if ((setup = GoSensor_Setup(sensor)) == kNULL)
			{
				ROS_ERROR("Error: GoSetup_SetTriggerSource: %d ", status);
			}
		}
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
	
	running = true;

	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud2 (new pcl::PointCloud<pcl::PointXYZ>);

	float y = 0.0;

	for(; running && ros::ok() ;)
	{	
		if ((status = GoSystem_Start(system)) != kOK)
		{
			ROS_ERROR("Error: GoSystem_Start:%d ", status);
		}

		pub.publish(cld_msg);

 		if(remainder(frame_idx, 20)!=0)
		{
			if(GoSystem_SetDataHandler(system, onData, &contextPointer))
			{						
				std::string cloudName = viewer_id + std::to_string(y);
				trans(profile_cloud_ptr, temp_cloud2, y);
				viewer->addPointCloud(temp_cloud2, profile_color_handler, cloudName);
  				viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,\
  													 3, cloudName);

				y+=5.5;
				temp_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
				temp_cloud2.reset(new pcl::PointCloud<pcl::PointXYZ>);	
			}
			if(!viewer->wasStopped()){
				viewer->spinOnce();}
		}
		else
		{
			y = 0.0;			
		}
		std::chrono::milliseconds duration(10);
		std::this_thread::sleep_for(duration);
	}

	// stop Gocator sensor
	if ((status = GoSystem_Stop(system)) != kOK)
	{
		ROS_ERROR("Error: GoSystem_Stop:%d\n", status);
		running = false;
	}
	ROS_INFO("system stop %d", status);

	// destroy handles
	GoDestroy(system);
	GoDestroy(api);

	if(!running or !ros::ok())
	{		
		ROS_INFO("Sensor Interrupt. Stopping Sensor");
		viz.quit();
		return 0;
	}
}
