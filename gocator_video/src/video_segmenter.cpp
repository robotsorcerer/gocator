#include "ros/ros.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <sensor_msgs/PointCloud2.h>

#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/min_cut_segmentation.h>

const std::string viewer_id = "segmentation_cloud";

class Receiver
{
public:
	Receiver(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
	: limits_min(0.0), limits_max(1.0), 
		running(false), 
		viewer_(viewer)
	{
		ROS_INFO("Receiver created");
		ROS_INFO_STREAM("viewer " << viewer_);
	}

	//void pclCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& msg);
	void pclCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& msg)
	{
		pcl::PCLPointCloud2 pcl_pc2;
		pcl_conversions::toPCL(*msg, pcl_pc2);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
		//ROS_INFO_STREAM("Point Cloud Data Arrived " << *cloud);

		retrieve(cloud);
	}

	void retrieve(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
	{
		ROS_INFO_STREAM("Point Cloud Data Arrived " << *cloud);

	  	pcl::IndicesPtr indices (new std::vector <int>);
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud(cloud);
		pass.setFilterFieldName("zFilt");
		pass.setKeepOrganized(false);
		//pass.setUserFilterValue(0.1);
		pass.setFilterLimits(limits_min, limits_max);
		pass.filter(*indices);

		pcl::MinCutSegmentation<pcl::PointXYZ> seg;
		seg.setInputCloud (cloud);
		seg.setIndices (indices);

		pcl::PointCloud<pcl::PointXYZ>::Ptr foreground_points(new pcl::PointCloud<pcl::PointXYZ> ());
		pcl::PointXYZ point;
		point.x = 68.97;
		point.y = -18.55;
		point.z = 0.57;
		foreground_points->points.push_back(point);
		seg.setForegroundPoints (foreground_points);

		seg.setSigma (0.25);
		seg.setRadius (3.0433856);
		seg.setNumberOfNeighbours (14);
		seg.setSourceWeight (0.8);

		std::vector <pcl::PointIndices> clusters;
		seg.extract (clusters);

		ROS_INFO_STREAM("Maximum flow is " << seg.getMaxFlow ());

		pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = seg.getColoredCloud ();
		//pcl::visualization::CloudViewer viewer ("Cluster viewer");		
		//viewer = Receiver::cloudCreator();
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> seg_color_handler (colored_cloud, 255, 0, 255);
		viewer_->addPointCloud<pcl::PointXYZRGB>(colored_cloud, seg_color_handler, viewer_id);
		viewer_->registerKeyboardCallback(&Receiver::keyboardEventOccurred, *this);

		ROS_INFO_STREAM("colored_cloud: " << *colored_cloud);

		while (!viewer_->wasStopped ())
		{
			viewer_->spinOnce(10);
			boost::this_thread::sleep(boost::posix_time::microseconds(100));
		}
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
	  		// case 's':
	  		// 	save=true;
	  		  break;
	  	}
	  }
	}

	~Receiver()
	{
		ROS_INFO("Destructor called");
	}
private:	
	double limits_min;
	double limits_max;
	bool running;

	ros::NodeHandle vidseg;
	ros::Subscriber sub;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
	
};

boost::shared_ptr<pcl::visualization::PCLVisualizer> cloudCreator()
{
	int screen_height = 640;
	int screen_width = 480;
	double pos_x = 0; double pos_y = 0; double pos_z = 0; 
	double view_x = 0; double view_y = -1; double view_z = 0;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Colored Viewer"));
	viewer->setBackgroundColor (0.4, 0.2, 0.5);
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, viewer_id);
	viewer->setSize(screen_height, screen_width);
	viewer->setCameraPosition(pos_x, pos_y, pos_z, view_x, view_y, view_z) ;	
	viewer->setShowFPS(true);
	viewer->initCameraParameters ();
	viewer->addCoordinateSystem (1.0);
	
	return viewer;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "video_segmenter");
	ros::NodeHandle vidseg;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	viewer = cloudCreator();

	Receiver receiver(viewer);
	ros::Subscriber sub = vidseg.subscribe("video_pixels", 100, \
											&Receiver::pclCallback, &receiver);

	bool running(false);
	if (ros::ok())
	{
		running = true;
	}
	ros::spin();

	return 0;
}