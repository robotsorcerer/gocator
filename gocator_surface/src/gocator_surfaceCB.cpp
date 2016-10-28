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

#include <GoSdk/GoSdk.h>
#include <gocator_bridge/maxon.h>
#include <gocator_surface/surface.h>

const std::string viewer_id = "Surface_PointClouds"; //must be here so visualizer can see it
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


/*Global variables*/
const std::string nodename = "surface";
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

pcl::PointCloud<pcl::PointXYZ>::Ptr retrieve_surface(int argc, char** argv, ros::Publisher pub, GoSystem system, GoDataSet dataset, Visualizer* viz)
{
	boost::lock_guard<boost::mutex> guard (mutex);
	SurfacePoint **surfaceBuffer = NULL;
	k32u surfaceBufferHeight = 0;
	pcl::PointXYZ surface_point;

	pcl::PointCloud<pcl::PointXYZ>::Ptr surface_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>& surface_cloud = *surface_cloud_ptr;

	if((GoSystem_ReceiveData(system, &dataset, RECEIVE_TIMEOUT) == kOK)) 
	{
		ROS_INFO_STREAM("Dataset count:" << GoDataSet_Count(dataset));
		// each result can have multiple data items
		// loop through all items in result message
		for (i = 0; i < GoDataSet_Count(dataset); ++i) 
		{
			GoDataMsg dataObj = GoDataSet_At(dataset, i);

		 switch (GoDataMsg_Type(dataObj)) 
		  {
			case GO_DATA_MESSAGE_TYPE_STAMP: 
			{
				GoStampMsg stampMsg = dataObj;
				ROS_INFO_STREAM("  Stamp Message batch count: " << GoStampMsg_Count(stampMsg));
				for (j = 0; j < GoStampMsg_Count(stampMsg); j++) 
				{
					GoStamp *stamp = GoStampMsg_At(stampMsg, j);
					ROS_INFO("  Timestamp: %llu", stamp->timestamp);
					ROS_INFO("  Encoder position at leading edge: %lld", stamp->encoder);
					ROS_INFO("  Frame index: %llu", stamp->frameIndex);
				}
			}
			break;

			case GO_DATA_MESSAGE_TYPE_SURFACE_INTENSITY: 
			{
				GoSurfaceIntensityMsg surfaceIntMsg = dataObj;
				unsigned int rowIdx, colIdx;
				double XResolution = NM_TO_MM(GoSurfaceIntensityMsg_XResolution(surfaceIntMsg));
				double YResolution = NM_TO_MM(GoSurfaceIntensityMsg_YResolution(surfaceIntMsg));
				double XOffset = UM_TO_MM(GoSurfaceIntensityMsg_XOffset(surfaceIntMsg));
				double YOffset = UM_TO_MM(GoSurfaceIntensityMsg_YOffset(surfaceIntMsg));

				ROS_INFO_STREAM("  Surface intensity width: "<< GoSurfaceIntensityMsg_Width(surfaceIntMsg));
				ROS_INFO_STREAM("  Surface intensity height: "<< GoSurfaceIntensityMsg_Length(surfaceIntMsg));

				//allocate memory if needed
				if (surfaceBuffer == NULL) 
				{
					surfaceBuffer = (SurfacePoint**) malloc(GoSurfaceIntensityMsg_Length(surfaceIntMsg) \
						* sizeof(SurfacePoint *));

					for (j = 0; j < GoSurfaceMsg_Length(surfaceIntMsg); j++) 
					{
						surfaceBuffer[j] = (SurfacePoint*) malloc(GoSurfaceIntensityMsg_Width(surfaceIntMsg) \
									* sizeof(SurfacePoint));
					}
					surfaceBufferHeight = GoSurfaceIntensityMsg_Length(surfaceIntMsg);
				}

				for (rowIdx = 0; rowIdx < GoSurfaceIntensityMsg_Length(surfaceIntMsg); rowIdx++) 
				{
					k8u *data = GoSurfaceIntensityMsg_RowAt(surfaceIntMsg, rowIdx);

					// gocator transmits intensity data as an 8-bit grayscale image of identical width and height as the corresponding height map
					for (colIdx = 0; colIdx < GoSurfaceIntensityMsg_Width(surfaceIntMsg);colIdx++) 
					{
						surfaceBuffer[rowIdx][colIdx].x = XOffset + XResolution * colIdx;
						surfaceBuffer[rowIdx][colIdx].y = YOffset + YResolution * rowIdx;
						surfaceBuffer[rowIdx][colIdx].intensity = data[colIdx];
						ROS_INFO_STREAM("Surface Buffer Intensity: " << surfaceBuffer[rowIdx][colIdx].intensity);
					}
				}
			}
			break;

			case GO_DATA_MESSAGE_TYPE_SURFACE: 
			{
				GoSurfaceMsg surfaceMsg = dataObj;
				unsigned int rowIdx, colIdx;
				double XResolution = NM_TO_MM(GoSurfaceMsg_XResolution(surfaceMsg));
				double YResolution = NM_TO_MM(GoSurfaceMsg_YResolution(surfaceMsg));
				double ZResolution = NM_TO_MM(GoSurfaceMsg_ZResolution(surfaceMsg));
				double XOffset = UM_TO_MM(GoSurfaceMsg_XOffset(surfaceMsg));
				double YOffset = UM_TO_MM(GoSurfaceMsg_YOffset(surfaceMsg));
				double ZOffset = UM_TO_MM(GoSurfaceMsg_ZOffset(surfaceMsg));

				ROS_INFO_STREAM("  Surface data width: "<< GoSurfaceMsg_Width(surfaceMsg));
				// ROS_INFO_STREAM("  Surface data length: " << GoSurfaceMsg_Length(surfaceMsg));

				//allocate memory if needed
				if (surfaceBuffer == NULL) 
				{	
					surfaceBuffer = (SurfacePoint**) malloc(GoSurfaceMsg_Length(surfaceMsg) \
										* sizeof(SurfacePoint *));

					for (j = 0; j < GoSurfaceMsg_Length(surfaceMsg); j++) 
					{
						surfaceBuffer[j] = (SurfacePoint*) malloc(GoSurfaceMsg_Width(surfaceMsg) \
											* sizeof(SurfacePoint));
					}

						surfaceBufferHeight = GoSurfaceMsg_Length(surfaceMsg);
						ROS_INFO_STREAM("Surface Buffer Height: " << surfaceBufferHeight);
				}

				for (rowIdx = 0; rowIdx < GoSurfaceMsg_Length(surfaceMsg); rowIdx++) 
				{
					k16s *data = GoSurfaceMsg_RowAt(surfaceMsg, rowIdx);

					for (colIdx = 0; colIdx < GoSurfaceMsg_Width(surfaceMsg); colIdx++) 
					{
						surfaceBuffer[rowIdx][colIdx].x = XOffset + XResolution * colIdx;
						surfaceBuffer[rowIdx][colIdx].y = YOffset + YResolution * rowIdx;

						//print a bunch'o'stuff
						if (data[colIdx] != INVALID_RANGE_16BIT) 
						{
							surfaceBuffer[rowIdx][colIdx].z = ZOffset + (ZResolution * data[colIdx]);
						} 
						else 
						{
							surfaceBuffer[rowIdx][colIdx].z = INVALID_RANGE_DOUBLE;
						}					
						surfaceBuffer[rowIdx][colIdx].z = ZOffset + ZResolution * data[colIdx];						

						ROS_INFO_STREAM("Surface Map Coordinates (x, y, z):	" << std::fixed << std::setfill ('0') << std::setprecision (3) << "(" << surfaceBuffer[rowIdx][colIdx].x << ", " <<
							            surfaceBuffer[rowIdx][colIdx].y << ", " << surfaceBuffer[rowIdx][colIdx].z << ").");

						surface_point.x = static_cast<float>(surfaceBuffer[rowIdx][colIdx].x);
						surface_point.y = static_cast<float>(surfaceBuffer[rowIdx][colIdx].y);
						surface_point.z = static_cast<float>(surfaceBuffer[rowIdx][colIdx].z);
						surface_cloud_ptr->points.push_back(surface_point);
					}
				}
				surface_cloud_ptr->width = (int)surface_cloud.points.size();
				surface_cloud_ptr->height = surfaceBufferHeight;
				surface_cloud_ptr->is_dense = true; //does data contain NaN values
				surface_cloud_ptr->points.resize(surface_cloud_ptr->height * surface_cloud_ptr->width);

				sensor_msgs::PointCloud2 msg;
				pcl::toROSMsg(*surface_cloud_ptr, msg);
				msg.header.stamp = ros::Time::now();
				msg.header.frame_id = "surface_tf_frame";
				pub.publish(msg);
			} //close case
			break;
		default:
			break;
		 }//close switch
		}//close for
	}
		//free memory arrays
		if (surfaceBuffer) 
		{
			unsigned int i;
			for (i = 0; i < surfaceBufferHeight; i++) 
			{
				free(surfaceBuffer[i]);
			}
		}
	return surface_cloud_ptr;
} //close retrieve_surface

pcl::PointCloud<pcl::PointXYZRGB>::Ptr retrieve_csurface(int argc, char** argv, ros::Publisher pub, GoSystem system, GoDataSet dataset, Visualizer* viz)
{
	boost::lock_guard<boost::mutex> guard (mutex);
	SurfacePoint **surfaceBuffer = NULL;
	k32u surfaceBufferHeight = 0;
	pcl::PointXYZRGB surface_point;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr surface_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>& surface_cloud = *surface_cloud_ptr;

	if((GoSystem_ReceiveData(system, &dataset, RECEIVE_TIMEOUT) == kOK)) 
	{
		ROS_INFO_STREAM("Dataset count:" << GoDataSet_Count(dataset));
		// each result can have multiple data items
		// loop through all items in result message
		for (i = 0; i < GoDataSet_Count(dataset); ++i) 
		{
			GoDataMsg dataObj = GoDataSet_At(dataset, i);

		 switch (GoDataMsg_Type(dataObj)) 
		  {
			case GO_DATA_MESSAGE_TYPE_STAMP: 
			{
				GoStampMsg stampMsg = dataObj;
				ROS_INFO_STREAM("  Stamp Message batch count: " << GoStampMsg_Count(stampMsg));
				for (j = 0; j < GoStampMsg_Count(stampMsg); j++) 
				{
					GoStamp *stamp = GoStampMsg_At(stampMsg, j);
					ROS_INFO("  Timestamp: %llu", stamp->timestamp);
					ROS_INFO("  Encoder position at leading edge: %lld", stamp->encoder);
					ROS_INFO("  Frame index: %llu", stamp->frameIndex);
				}
			}
			break;

			case GO_DATA_MESSAGE_TYPE_SURFACE_INTENSITY: 
			{
				GoSurfaceIntensityMsg surfaceIntMsg = dataObj;
				unsigned int rowIdx, colIdx;
				double XResolution = NM_TO_MM(GoSurfaceIntensityMsg_XResolution(surfaceIntMsg));
				double YResolution = NM_TO_MM(GoSurfaceIntensityMsg_YResolution(surfaceIntMsg));
				double XOffset = UM_TO_MM(GoSurfaceIntensityMsg_XOffset(surfaceIntMsg));
				double YOffset = UM_TO_MM(GoSurfaceIntensityMsg_YOffset(surfaceIntMsg));

				ROS_INFO_STREAM("  Surface intensity width: "<< GoSurfaceIntensityMsg_Width(surfaceIntMsg));
				ROS_INFO_STREAM("  Surface intensity height: "<< GoSurfaceIntensityMsg_Length(surfaceIntMsg));

				//allocate memory if needed
				if (surfaceBuffer == NULL) 
				{
					surfaceBuffer = (SurfacePoint**) malloc(GoSurfaceIntensityMsg_Length(surfaceIntMsg) \
						* sizeof(SurfacePoint *));

					for (j = 0; j < GoSurfaceMsg_Length(surfaceIntMsg); j++) 
					{
						surfaceBuffer[j] = (SurfacePoint*) malloc(GoSurfaceIntensityMsg_Width(surfaceIntMsg) \
									* sizeof(SurfacePoint));
					}
					surfaceBufferHeight = GoSurfaceIntensityMsg_Length(surfaceIntMsg);
				}

				for (rowIdx = 0; rowIdx < GoSurfaceIntensityMsg_Length(surfaceIntMsg); rowIdx++) 
				{
					k8u *data = GoSurfaceIntensityMsg_RowAt(surfaceIntMsg, rowIdx);

					// gocator transmits intensity data as an 8-bit grayscale image of identical width and height as the corresponding height map
					for (colIdx = 0; colIdx < GoSurfaceIntensityMsg_Width(surfaceIntMsg);colIdx++) 
					{
						surfaceBuffer[rowIdx][colIdx].x = XOffset + XResolution * colIdx;
						surfaceBuffer[rowIdx][colIdx].y = YOffset + YResolution * rowIdx;
						surfaceBuffer[rowIdx][colIdx].intensity = data[colIdx];
						ROS_INFO_STREAM("Surface Buffer Intensity: " << surfaceBuffer[rowIdx][colIdx].intensity);
					}
				}
			}
			break;

			case GO_DATA_MESSAGE_TYPE_SURFACE: 
			{
				GoSurfaceMsg surfaceMsg = dataObj;
				unsigned int rowIdx, colIdx;
				double XResolution = NM_TO_MM(GoSurfaceMsg_XResolution(surfaceMsg));
				double YResolution = NM_TO_MM(GoSurfaceMsg_YResolution(surfaceMsg));
				double ZResolution = NM_TO_MM(GoSurfaceMsg_ZResolution(surfaceMsg));
				double XOffset = UM_TO_MM(GoSurfaceMsg_XOffset(surfaceMsg));
				double YOffset = UM_TO_MM(GoSurfaceMsg_YOffset(surfaceMsg));
				double ZOffset = UM_TO_MM(GoSurfaceMsg_ZOffset(surfaceMsg));

				ROS_INFO_STREAM("  Surface data width: "<< GoSurfaceMsg_Width(surfaceMsg));
				ROS_INFO_STREAM("  Surface data length: " << GoSurfaceMsg_Length(surfaceMsg));

				//allocate memory if needed
				if (surfaceBuffer == NULL) 
				{	
					surfaceBuffer = (SurfacePoint**) malloc(GoSurfaceMsg_Length(surfaceMsg) \
										* sizeof(SurfacePoint *));

					for (j = 0; j < GoSurfaceMsg_Length(surfaceMsg); j++) 
					{
						surfaceBuffer[j] = (SurfacePoint*) malloc(GoSurfaceMsg_Width(surfaceMsg) \
											* sizeof(SurfacePoint));
					}

						surfaceBufferHeight = GoSurfaceMsg_Length(surfaceMsg);
						ROS_INFO_STREAM("Surface Buffer Height: " << surfaceBufferHeight);
				}

				uint8_t r(255), g(15), b(15);
				for (rowIdx = 0; rowIdx < GoSurfaceMsg_Length(surfaceMsg); rowIdx++) 
				{
					k16s *data = GoSurfaceMsg_RowAt(surfaceMsg, rowIdx);

					for (colIdx = 0; colIdx < GoSurfaceMsg_Width(surfaceMsg); colIdx++) 
					{
						surfaceBuffer[rowIdx][colIdx].x = XOffset + XResolution * colIdx;
						surfaceBuffer[rowIdx][colIdx].y = YOffset + YResolution * rowIdx;

						//print a bunch'o'stuff
						if (data[colIdx] != INVALID_RANGE_16BIT) 
						{
							surfaceBuffer[rowIdx][colIdx].z = ZOffset + (ZResolution * data[colIdx]);
						} 
						else 
						{
							surfaceBuffer[rowIdx][colIdx].z = INVALID_RANGE_DOUBLE;
						}					
						
						// ROS_INFO_STREAM("Surface Map Coordinates (x, y, z):	" << std::fixed << std::setfill ('0') << std::setprecision (3) << "(" << surfaceBuffer[rowIdx][colIdx].x << ", " <<
						// 	            surfaceBuffer[rowIdx][colIdx].y << ", " << surfaceBuffer[rowIdx][colIdx].z << ").");

						surface_point.x = static_cast<float>(surfaceBuffer[rowIdx][colIdx].x);
						surface_point.y = static_cast<float>(surfaceBuffer[rowIdx][colIdx].y);
						surface_point.z = static_cast<float>(surfaceBuffer[rowIdx][colIdx].z);

						if (surface_point.z < 0.0)
						{
						  r -= 12;
						  g += 12;
						}
						else if (surface_point.z > 1000)
						{
						  g -= 12;
						  b += 12;
						}
						uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
						        static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));

						surface_point.rgb = *reinterpret_cast<float*>(&rgb);						
						surface_cloud_ptr->points.push_back(surface_point);
					}
				}
				surface_cloud_ptr->width = (int)surface_cloud.points.size();
				surface_cloud_ptr->height = surfaceBufferHeight;
				surface_cloud_ptr->is_dense = true; //does data contain NaN values
				surface_cloud_ptr->points.resize(surface_cloud_ptr->height * surface_cloud_ptr->width);

				sensor_msgs::PointCloud2 msg;
				pcl::toROSMsg(*surface_cloud_ptr, msg);
				msg.header.stamp = ros::Time::now();
				msg.header.frame_id = "surface_tf_frame";
				pub.publish(msg);
			} //close case
			break;
		default:
			break;
		 }//close switch
		}//close for
	}
		//free memory arrays
		if (surfaceBuffer) 
		{
			unsigned int i;
			for (i = 0; i < surfaceBufferHeight; i++) 
			{
				free(surfaceBuffer[i]);
			}
		}
	return surface_cloud_ptr;
} //close retrieve_surface

int main(int argc, char **argv)
{
	bool running(false);
	bool updateCloud(false);

	GoSystem system = kNULL;
	GoDataSet dataset = kNULL;
	//GoControl control;
	GoSensor sensor = kNULL;
	GoSetup setup = kNULL;
	//k64s encoder;
		
	GoMode surface_mode = GO_MODE_SURFACE;
	// k32s GoSurfaceGenerationStartTrigger;
	// GoSurfaceGenerationStartTrigger = GO_SURFACE_GENERATION_START_TRIGGER_SEQUENTIAL;

	//Override SIGINT handler
	ros::init(argc, argv, nodename, ros::init_options::NoSigintHandler);
	signal(SIGINT, mySigintHandler);

	ros::NodeHandle ns;
	ros::Publisher pub = ns.advertise<sensor_msgs::PointCloud2>("surface_points", 100);
	
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

	//Follow C++ API convention and cast the return of malloc to ProfilePoint to avoid glibc errors
	if ((profileBuffer = (ProfilePoint*)malloc(profilePointCount * sizeof(SurfacePoint))) == kNULL)
	{
		ROS_ERROR("Error: Cannot allocate Surface Data, %d points", profilePointCount);		
	}

/*	if((status = GoPartDetection_EnableEdgeFilter(detection, enable) !=kOK))	
		ROS_INFO("Could not enable edge filtering: %d", status);

	if((status = GoPartDetection_SetThreshold(detection, height)!= kOK))
		ROS_INFO("Could not set part detection height");

	if((status = GoPartDetection_SetMinArea(detection, minArea) !=kOK))
		ROS_INFO("Could not set min are:, %d", status);

	if((status = GoSurfaceGeneration_SetGenerationType(surface, surfaceType))!=kOK)
		ROS_INFO("Error: Could not set surface generation type, %d", status);*/

	// if((status = GoSetup_EnableInputTrigger(setup, kTRUE)!=kOK))
	// 	ROS_INFO("Error: GoSetup_EnableInputTrigger: %d", status);

/*	if((status= GoControl_GetEncoder(control, &encoder) !=kOK))
		ROS_INFO("Could not retrieve encoder values %d ", status);*/

/*	move.SetDefaultParameters();

	if((lResult = move.OpenDevice(&ulErrorCode))!=MMC_SUCCESS && !ros::ok())
	{
		ROS_ERROR_STREAM("OpenDevice" <<", " << lResult <<", " << ulErrorCode);
		return lResult;
	}*/

	if (ros::ok()) {running = true;}

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;	
	//pcl::PointCloud<pcl::PointXYZ>::Ptr surface_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr surface_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	viewer = viz.createViewer();

	for(; running && !viewer->wasStopped() ;)
	{

		if((status = GoSetup_SetScanMode(setup, surface_mode)) == kOK)
		{
			ROS_INFO("GoSetup_SetScanMode, %d", status);
		}
/*		
		if((lResult = move.prepare_motion(&ulErrorCode))!=MMC_SUCCESS)
		{
			ROS_ERROR_STREAM("could not prepare motion " <<", " << lResult << ", " << ulErrorCode);
			return lResult;
		}	

		ROS_INFO_STREAM("\n\nAlright now, rolling to home position");

		if((lResult = move.go_home(&ulErrorCode))!=MMC_SUCCESS && !ros::ok())
		{
			ROS_ERROR_STREAM("Going home" <<", " << lResult <<", " << ulErrorCode);
			return lResult;
		}*/

		for(int i = 0; i < 5; ++i)
		{
			surface_cloud_ptr = retrieve_csurface(argc, argv, pub, system, dataset, &viz);
			//surface_cloud_ptr = retrieve_surface(argc, argv, pub, system, dataset, &viz);
			ROS_INFO_STREAM("surface_cloud_ptr :" << *surface_cloud_ptr);	
			//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> surface_color_handler (surface_cloud_ptr, 255, 0.2, 250);
			pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> surface_color_handler(surface_cloud_ptr);
			viewer->addPointCloud<pcl::PointXYZRGB>(surface_cloud_ptr, surface_color_handler, viewer_id);
			viewer->spinOnce(10);			
			boost::this_thread::sleep(boost::posix_time::microseconds(100));

/*			start_mover(&move);

			ROS_INFO_STREAM("motion iteration: " << i);

			if((lResult = move.prepare_motion(&ulErrorCode))!=MMC_SUCCESS)
			{
				ROS_ERROR_STREAM("could not prepare motion " <<", " << lResult << ", " << ulErrorCode);
			}
*/
			if(updateCloud)
			{
				viewer->removePointCloud(viewer_id);
				viewer->removeCoordinateSystem(1.0);
				viewer->updatePointCloud(surface_cloud_ptr, viewer_id);
			}
			updateCloud = true;
		}

	} //close while	

	// stop Gocator sensor
	if ((status = GoSystem_Stop(system)) != kOK)
	{
		ROS_INFO(" Stopping GoSensor:%d", status);
	}

	if((lResult = move.close_device(&ulErrorCode))!=MMC_SUCCESS)
	{
		ROS_INFO_STREAM("CloseDevice " << lResult << ", " <<  ulErrorCode);
		//return lResult;
	}

	// destroy handlesinfo 
	GoDestroy(system);
	GoDestroy(dataset);
	GoDestroy(api);	
	// delete surface_cloud_ptr;
	std::cout << "\n" << std::endl;
	ROS_INFO("Press Ctrl + C to continue...\n");
	viewer->close();

	ros::spin();

	return 0;
}
