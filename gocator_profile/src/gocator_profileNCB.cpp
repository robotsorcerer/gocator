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
#include <ros/xmlrpc_manager.h>
#include <geometry_msgs/Point.h>
#include <GoSdk/GoSdk.h>
#include <gocator_bridge/maxon.h>
const std::string viewer_id = "Profile_PointClouds"; //must be here so visualizer can see it
#include <gocator_bridge/Visualizer.h>

#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <signal.h>
#include <iostream>
#include <vector>
#include <string.h>
#include <sstream>
#include <fstream>
#include <math.h>

// #include <boost/chrono.hpp>
const std::string nodename = "profileNCB";


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

pcl::PointCloud<pcl::PointXYZ>::Ptr retrieve_profile(int argc, char** argv, ros::Publisher pub, \
													GoSystem system,  GoDataSet dataset, k32u profilePointCount, Visualizer* viz)
{	
	boost::lock_guard<boost::mutex> guard (mutex);
	std::vector<float> xvec, zvec;
	unsigned int i, j, k, arrayIndex;
	pcl::PointCloud<pcl::PointXYZ>::Ptr profile_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr rviz_profile_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PointCloud<pcl::PointXYZ>& profile_cloud = *profile_cloud_ptr;
	
	if (GoSystem_ReceiveData(system, &dataset, RECEIVE_TIMEOUT) == kOK && ros::ok())
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
					ROS_INFO_STREAM("Stamp Message batch count: "<< GoStampMsg_Count(stampMsg));
					for (j = 0; j < GoStampMsg_Count(stampMsg); ++j)
					{
						stamp = GoStampMsg_At(stampMsg, j);
						ROS_INFO("  Timestamp: %llu", stamp->timestamp);
						ROS_INFO("  Encoder: %lld", stamp->encoder);
						ROS_INFO("  Frame index: %llu", stamp->frameIndex);
					}
				}
				break;

				case GO_DATA_MESSAGE_TYPE_RESAMPLED_PROFILE:
				{
					GoResampledProfileMsg profileMsg = dataObj;
					ROS_INFO_STREAM("Resampled Profile Message batch count: " << GoResampledProfileMsg_Count(profileMsg));

					float x, z;
					unsigned int validPointCount = 0;

					for (k = 0; k < GoResampledProfileMsg_Count(profileMsg); ++k)
					{
						short* data = GoResampledProfileMsg_At(profileMsg, k);
						double XResolution = NM_TO_MM(GoResampledProfileMsg_XResolution(profileMsg));
						double ZResolution = NM_TO_MM(GoResampledProfileMsg_ZResolution(profileMsg));
						double XOffset = UM_TO_MM(GoResampledProfileMsg_XOffset(profileMsg));
						double ZOffset = UM_TO_MM(GoResampledProfileMsg_ZOffset(profileMsg));
				 	//translate 16-bit range data to engineering units and copy profiles to memory array
						ROS_INFO("Looping thru count of points in each re-sampled profile array");

						for (arrayIndex = 0; arrayIndex < GoResampledProfileMsg_Width(profileMsg); ++arrayIndex)
						{
							if (data[arrayIndex] != INVALID_RANGE_16BIT )
							{
								profileBuffer[arrayIndex].x = XOffset + XResolution * arrayIndex;
								profileBuffer[arrayIndex].z = ZOffset + ZResolution * data[arrayIndex];

								//x & z are the address of the retrieved points
								x = static_cast<float>((profileBuffer[arrayIndex].x)/1);
								z = static_cast<float>((profileBuffer[arrayIndex].z)/1);


/*								ROS_INFO_STREAM("x resampled profile : " << x );
								ROS_INFO_STREAM("z resampled profile : " << z);*/

								xvec.push_back(x);	zvec.push_back(z);

								pcl::PointXYZ profile_points, rviz_profile_points;
								pcl::PointXYZRGB rviz_colored_points;

								profile_points.x = x;
								profile_points.y = 0.0;
								profile_points.z = z;
								profile_cloud_ptr-> points.push_back(profile_points);

						 		rviz_profile_points.x = x/1000;
						 		rviz_profile_points.y = 0.0;
						 		rviz_profile_points.z = z/1000;
						 		rviz_profile_cloud_ptr-> points.push_back(rviz_profile_points);

						 		//save points to csv file
						 		if(pcl::console::find_argument(argc, argv, "-s") >= 0)
						 		{
						 			viz->savepoints2d(profileBuffer, arrayIndex);
						 		}
						 		validPointCount++;
						 	}
						 	else
						 	{
						 		profileBuffer[arrayIndex].x = XOffset + XResolution * arrayIndex;
						 		profileBuffer[arrayIndex].z = INVALID_RANGE_DOUBLE;
						 		x = static_cast<float>(profileBuffer[arrayIndex].x);
						 		z = static_cast<float>(profileBuffer[arrayIndex].z);
						 		//ROS_INFO_STREAM(Invalid profile (x,z) : (" << x << ",\t" << z << ")");
							}
						}

						 ROS_INFO("Profile Valid Point %d out of max %d", validPointCount, profilePointCount);
						 //unorganized point cloud with 307200 points
						 profile_cloud_ptr->width 	= rviz_profile_cloud_ptr->width = (int)profile_cloud.points.size();
						 profile_cloud_ptr->height 	= rviz_profile_cloud_ptr->height = 1;

						 profile_cloud_ptr->is_dense = rviz_profile_cloud_ptr->is_dense = true;

						 profile_cloud_ptr->points.resize(profile_cloud_ptr->height * profile_cloud_ptr->width);
						 rviz_profile_cloud_ptr->points.resize(rviz_profile_cloud_ptr->height * rviz_profile_cloud_ptr->width);

						 sensor_msgs::PointCloud2 cld_msg;
						 pcl::toROSMsg(*rviz_profile_cloud_ptr, cld_msg);
						 cld_msg.header.stamp = ros::Time::now();
						 cld_msg.header.frame_id = "profile_tf_frame";
						 pub.publish(cld_msg);
					}
				}		//closes case statement
				break;

				case GO_DATA_MESSAGE_TYPE_PROFILE: // Note this is NON resampled profile
				{
					GoProfileMsg profileMsg = dataObj;
					ROS_INFO_STREAM("Profile Message batch count: " <<  GoProfileMsg_Count(profileMsg));
					float x, z;

					for (k = 0; k < GoProfileMsg_Count(profileMsg); ++k)
					{
						kPoint16s* data = GoProfileMsg_At(profileMsg, k);
						unsigned int validPointCount = 0;
						double XResolution = NM_TO_MM(GoProfileMsg_XResolution(profileMsg));
						double ZResolution = NM_TO_MM(GoProfileMsg_ZResolution(profileMsg));
						double XOffset = UM_TO_MM(GoProfileMsg_XOffset(profileMsg));
						double ZOffset = UM_TO_MM(GoProfileMsg_ZOffset(profileMsg));
						//translate 16-bit range data to engineering units and copy profiles to memory array
						for (arrayIndex = 0; arrayIndex < GoProfileMsg_Width(profileMsg); ++arrayIndex)
						{	
							// ROS_INFO_STREAM("data[arrayIndex].x: " << data[arrayIndex].x);
							if (data[arrayIndex].x != INVALID_RANGE_16BIT)
							{
								profileBuffer[arrayIndex].x = XOffset + XResolution * data[arrayIndex].x;
								profileBuffer[arrayIndex].z = ZOffset + ZResolution * data[arrayIndex].y;
								
								x = static_cast<float>(profileBuffer[arrayIndex].x);
								z = static_cast<float>(profileBuffer[arrayIndex].z);

								xvec.push_back(x);	zvec.push_back(z);
								pcl::PointXYZ profile_points, rviz_profile_points;
								pcl::PointXYZRGB rviz_colored_points;
								profile_points.x = x;
								profile_points.y = 0.0;
								profile_points.z = z;
								profile_cloud_ptr-> points.push_back(profile_points);

								rviz_profile_points.x = x/1000;
								rviz_profile_points.y = 0.0;
								rviz_profile_points.z = z/1000;
								rviz_profile_cloud_ptr-> points.push_back(rviz_profile_points);

								 //save points to csv file
								 if(pcl::console::find_argument(argc, argv, "-s") >= 0)
								 {
								 	viz->savepoints2d(profileBuffer, arrayIndex);
								 }
								validPointCount++;
								// ROS_INFO_STREAM("x non-resampled profile : " << profileBuffer[arrayIndex].x );
								// // ROS_INFO_STREAM("z non-resampled profile : " << profileBuffer[arrayIndex].z);
								// ROS_INFO_STREAM("profileBuffer: " << profileBuffer[arrayIndex].intensity<< "\n");
							}
							else
							{
								profileBuffer[arrayIndex].x = INVALID_RANGE_DOUBLE;
								profileBuffer[arrayIndex].z = INVALID_RANGE_DOUBLE;
								x = static_cast<float>(profileBuffer[arrayIndex].x);
								z = static_cast<float>(profileBuffer[arrayIndex].z);
							}
						}
						ROS_INFO("  Profile Valid Point %d out of max %d", validPointCount, profilePointCount);
						profile_cloud_ptr->width 	= rviz_profile_cloud_ptr->width = (int)profile_cloud.points.size();
						profile_cloud_ptr->height 	= rviz_profile_cloud_ptr->height = 1;

						profile_cloud_ptr->is_dense = rviz_profile_cloud_ptr->is_dense = true;

						profile_cloud_ptr->points.resize(profile_cloud_ptr->height * profile_cloud_ptr->width);
						rviz_profile_cloud_ptr->points.resize(rviz_profile_cloud_ptr->height * rviz_profile_cloud_ptr->width);

						sensor_msgs::PointCloud2 cld_msg;
						pcl::toROSMsg(*rviz_profile_cloud_ptr, cld_msg);
						cld_msg.header.stamp = ros::Time::now();
						cld_msg.header.frame_id = "profile_tf_frame";
						pub.publish(cld_msg);
					}
				}
				break;

				case GO_DATA_MESSAGE_TYPE_PROFILE_INTENSITY:
				{
					//unsigned int validPointCount = 0;
					GoProfileIntensityMsg intensityMsg = dataObj;
					ROS_INFO_STREAM("Intensity Message batch count: " << GoProfileIntensityMsg_Count(intensityMsg));
					for (k = 0; k < GoProfileIntensityMsg_Count(intensityMsg); ++k)
					{
						unsigned char* data = GoProfileIntensityMsg_At(intensityMsg, k);
						for (arrayIndex = 0; arrayIndex < GoProfileIntensityMsg_Width(intensityMsg); ++arrayIndex)
						{
							profileBuffer[arrayIndex].intensity = data[arrayIndex];
							ROS_INFO_STREAM("Intensity of Laser Points: " << profileBuffer[arrayIndex].intensity << "\n");
						}
					}
				}
				break;
			}	//close switch
		}
		GoDestroy(dataset);
	}		//close if
	//return profile_cloud_ptr;
	return rviz_profile_cloud_ptr;
}

int main(int argc, char **argv)
{
	bool running(false);
	bool updateCloud(false);

	GoSystem system = kNULL;
	GoDataSet dataset = kNULL;
	GoSensor sensor = kNULL;
	GoSetup setup = kNULL;

	GoMode 	profile_mode = GO_MODE_PROFILE;
/*	k32s GoSurfaceGenerationStartTrigger = GO_SURFACE_GENERATION_START_TRIGGER_SEQUENTIAL;
	GoEncoderTriggerMode TriggerMode = GO_ENCODER_TRIGGER_MODE_BIDIRECTIONAL;*/

	//Override SIGINT handler
	ros::init(argc, argv, nodename, ros::init_options::NoSigintHandler);
	signal(SIGINT, mySigintHandler);

	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("profile_points", 100);
	
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


/*	move.SetDefaultParameters();

	if((lResult = move.OpenDevice(&ulErrorCode))!=MMC_SUCCESS && !ros::ok())
	{
		ROS_ERROR_STREAM("OpenDevice" <<", " << lResult <<", " << ulErrorCode);
		return lResult;
	}*/

	if (ros::ok()) {running = true;}

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;	
	pcl::PointCloud<pcl::PointXYZ>::Ptr profile_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	viewer = viz.createViewer();

	for(; running && !viewer->wasStopped() ;)
	{
		if (GoSetup_UniformSpacingEnabled(setup))
		{
			// Uniform spacing is enabled. The number is based on the X Spacing setting
			profilePointCount = GoSetup_XSpacingCount(setup, GO_ROLE_MAIN);
			ROS_INFO_STREAM("Uniform spacing profile point count is " << profilePointCount);
		}
		else
		{
			// non-uniform spacing is enabled. The max number is based on the number of columns used in the camera. 
			profilePointCount = GoSetup_FrontCameraWidth(setup, GO_ROLE_MAIN);
			ROS_INFO_STREAM("Non-uniform spacing profile point count is " << profilePointCount);
		}

		//Follow C++ API convention and cast the return of malloc to ProfilePoint to avoid glibc errors
		if ((profileBuffer = (ProfilePoint*)malloc(profilePointCount * sizeof(ProfilePoint))) == kNULL)
		{
			ROS_ERROR("Error: Cannot allocate profileData, %d points", profilePointCount);		
		}

		if((status = GoSetup_SetScanMode(setup, profile_mode)) == kOK)
		{
			ROS_INFO("GoSetup_SetScanMode, %d", status);
		}

/*		if((lResult = move.go_home(&ulErrorCode))!=MMC_SUCCESS && !ros::ok())
		{
			ROS_ERROR_STREAM("Going home" <<", " << lResult <<", " << ulErrorCode);
			return lResult;
		}*/

		for(int i = 0; i < 5; ++i)
		{
			profile_cloud_ptr = retrieve_profile(argc, argv, pub, system, dataset, profilePointCount, &viz);
			ROS_INFO_STREAM("point_cloud_ptr " << *profile_cloud_ptr);

			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> profile_color_handler (profile_cloud_ptr, 255, 0, 0);
			viewer->addPointCloud<pcl::PointXYZ>(profile_cloud_ptr, profile_color_handler, viewer_id);
			viewer->spinOnce(10);
			boost::this_thread::sleep(boost::posix_time::microseconds(100));

/*			start_mover(&move);

			ROS_INFO_STREAM("motion iteration: " << i);

			if((lResult = move.prepare_motion(&ulErrorCode))!=MMC_SUCCESS)
			{
				ROS_ERROR_STREAM("could not prepare motion " <<", " << lResult << ", " << ulErrorCode);
			}*/

			if(updateCloud)
			{				
				// viewer->removePointCloud(viewer_id);
				viewer->removeCoordinateSystem(1.0);
				viewer->updatePointCloud(profile_cloud_ptr, viewer_id);
			}
			updateCloud = true;

		}			
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
	GoDestroy(api);
	free(profileBuffer);
	std::cout << "\n" << std::endl;
	viewer->close();

	ros::spin();

	return 0;
}