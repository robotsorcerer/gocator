#ifndef _VISUALIZER_H_
#define _VISUALIZER_H_

#define RECEIVE_TIMEOUT			(20000000) 
#define INVALID_RANGE_16BIT		((signed short)0x8000)			// gocator transmits range data as 16-bit signed integers. 0x8000 signifies invalid range data.	
#define DOUBLE_MAX				((k64f)1.7976931348623157e+308)	// 64-bit double - largest positive value.	
#define INVALID_RANGE_DOUBLE	((k64f)-DOUBLE_MAX)				// floating point value to represent invalid range data.	
#define SENSOR_IP			    "192.168.1.10"						

#define NM_TO_MM(VALUE) (((k64f)(VALUE))/1000000.0)
#define UM_TO_MM(VALUE) (((k64f)(VALUE))/1000.0)


#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/common/common_headers.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>

#include <boost/thread/thread.hpp>
#include <sstream>
#include <string>

typedef struct
{
	double x;	// x-coordinate in engineering units (mm) - position along laser line
	double z;	// z-coordinate in engineering units (mm) - height (at the given x position)
	unsigned char intensity;
}ProfilePoint;

typedef struct
{
	double x;	// x-coordinate in engineering units (mm) - position along laser line
	double y;
	double z;	// z-coordinate in engineering units (mm) - height (at the given x position)
	unsigned char intensity;
}SurfacePoint;

typedef struct
{
	int x;	// x-coordinate in engineering units (mm) - position along laser line
	int y;
	int z;	// z-coordinate in engineering units (mm) - height (at the given x position)
	unsigned char intensity;
}VideoPoint;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

kAssembly api = kNULL;
kStatus status;
kIpAddress ipAddress;
GoStamp *stamp =kNULL;
GoDataMsg dataObj;
k32u profilePointCount;

pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
ProfilePoint* profileBuffer = NULL;
SurfacePoint **surfaceBuffer = NULL;
VideoPoint **videoBuffer = NULL;

class Visualizer
{
public:
	//constructor
	Visualizer()
	: pos_x(0), pos_y(0), pos_z(0), view_x(0), 
		view_y(-1), view_z(0), view_up_x(0), view_up_y(1), view_up_z(1), text_id(0),
		viewport(0), running(false), updateCloud(false), save(false), screen_height(640), 
		screen_width(480), viewer("Point cloud"), frame(0)
	{
		ROS_INFO_STREAM("Visualizer object created");
	}

	void start()
	{	
		std::cout <<"\n\n";
		ROS_INFO("=================================================================");
		ROS_INFO("                      GoSDK ROS Resampled Profile                ");
		ROS_INFO("                                                                 ");
		ROS_INFO("               Bridge to the GoSDK version 4.3.3                 ");
		ROS_INFO("                                                                 ");
		ROS_INFO("        Code by Olalekan Ogunmolu <<ogunmolu@amazon.com>>        ");
		ROS_INFO("                                                                 ");
		ROS_INFO("                      Press 'q' to quit                          ");
		ROS_INFO("                                                                 ");
		ROS_INFO("              	      Amazon Robotics License                  ");
		ROS_INFO("=================================================================");
		std::cout << "\n\n";
	}

	void savepoints2d(ProfilePoint* profileBuffer,  unsigned int arrayIndex)
	{
		//save points to csv file
		std::ofstream points2d;
		points2d.open("points3d.csv", std::ofstream::out | std::ofstream::app);
		points2d << profileBuffer[arrayIndex].x <<"\t" << "\t" << profileBuffer[arrayIndex].z << "\n";
		points2d.close();
	}

	void savepoints3d(SurfacePoint* surfaceBuffer,  unsigned int arrayIndex)
	{
		//save points to csv file
		std::ofstream surfacepoints;
		surfacepoints.open("points3d.csv", std::ofstream::out | std::ofstream::app);
		surfacepoints << surfaceBuffer[arrayIndex].x <<"\t" << surfaceBuffer[arrayIndex].y <<"\t" \
				 << surfaceBuffer[arrayIndex].z << "\n";
		surfacepoints.close();
	}

	void saveCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
	{
	  std::ostringstream oss;
	  oss.str("");
	  oss << "./" << std::setfill('0') << std::setw(4) << frame;
	  const std::string baseName = oss.str();
	  const std::string cloudName = baseName + "_cloud.pcd";

	  ROS_INFO_STREAM("saving cloud: " << cloudName);
	  writer.writeBinary(cloudName, *cloud);
	  ++frame;
	}

	void quit()
	{
		this->running = false;
	}

	void help()
	{
		ROS_INFO("You must either activate profile or surface mode by passing the right cmd args");
		ROS_INFO("  To retrieve profile map, pass -p as runtime arg    ");
		ROS_INFO("  To retrieve surface map, pass -f as runtime arg    ");
		ROS_INFO("                                                     ");
		ROS_INFO("	                 RVIZ info :                       ");
		printf("                     \n                                ");
		ROS_INFO("    profile frame name: profile_tf_frame             ");
		ROS_INFO("    surface frame name: surface_tf_frame             ");
	}

	boost::shared_ptr<pcl::visualization::PCLVisualizer> createViewer()
	{

	 	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("PCL Viewer"));
		
	 	viewer->initCameraParameters();
	 	viewer->setBackgroundColor (0.2, 0.3, 0.3);
	 	viewer->setSize(screen_height, screen_width);
	 	viewer->setShowFPS(true);
	 	viewer->setCameraPosition(pos_x, pos_y, pos_z, view_x, view_y, view_z) ;	
	 	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, \
	 		                                      50.0,/* 13.0, 13.0,*/ viewer_id, viewport);
	 	// viewer->addCoordinateSystem(0.5);
	 	viewer->registerKeyboardCallback(&Visualizer::keyboardEventOccurred, *this);	 	

	  return (viewer);
	}

	boost::shared_ptr<pcl::visualization::PCLVisualizer> colorViewer()
	{
	  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Colored Viewer"));
	  viewer->setBackgroundColor (0, 0, 0);
	  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, viewer_id);
	  viewer->setSize(screen_height, screen_width);
	  viewer->setCameraPosition(pos_x, pos_y, pos_z, view_x, view_y, view_z) ;	
	  viewer->setShowFPS(true);
	  viewer->initCameraParameters ();
	  viewer->addCoordinateSystem (1.0);
	 	viewer->registerKeyboardCallback(&Visualizer::keyboardEventOccurred, *this);
	  return (viewer);
	}


	void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
	                            void* )
	{if (event.keyUp())
	  {
	  	switch(event.getKeyCode())
	  	{
	  		case 'q':
	  		  running = false;
	  		  break;
	  		case ' ':	
	  		case 's':
	  			save=true;
	  		  break;
	  	}
	  }
	}

	void mouseEventOccurred(const pcl::visualization::MouseEvent &event,
	                         void* viewer_void)
	{
	  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
	  if (event.getButton () == pcl::visualization::MouseEvent::LeftButton &&
	      event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
	  {
	    std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;

	    char str[512];
	    sprintf (str, "text#%03d", text_id ++);
	    viewer->addText ("clicked here", event.getX (), event.getY (), str);
	  }
	}

	//Destructor
	~Visualizer()
	{
		std::cout << "Visualizer object destroyed" << std::endl;
	}

private:
	double pos_x, pos_y, pos_z, view_x;
	double view_y, view_z, view_up_x, view_up_y, view_up_z;
	unsigned int text_id, viewport;
	bool running, updateCloud, save;

	const int screen_height, screen_width;
	const std::string viewer;
	size_t frame;
	pcl::PCDWriter writer;
};

#endif