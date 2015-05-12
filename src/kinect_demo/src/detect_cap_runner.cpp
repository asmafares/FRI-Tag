//Robot runs from pink cap!
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <cstdio>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>

#include "geometry_msgs/Twist.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "tf/tf.h"

#include "geometry_msgs/Twist.h"
#include "angles/angles.h"

#include "LearningController.h"
#include "ValueLearner.h"

#include <cmath>

LearningController *controller;
bool caught = false;

std::vector<double> lastState;
geometry_msgs::Twist lastAction;


double r(const std::vector<double>& s, geometry_msgs::Twist& a,  const std::vector<double>& s_prime) {
  
 //reward: -100 if it's caught, +1 if it's not

  double reward;
  
  if(caught)
    reward = -100;
  else
    reward = 1;
    
  ROS_INFO_STREAM("reward: " << reward);

  return reward;
 
}

// camera_link for fixed frame in global option in rviz

/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool new_cloud_available_flag = false;
    geometry_msgs::Twist cmd;

// General point cloud to store the whole image
PointCloudT::Ptr cloud (new PointCloudT);

//Point Cloud to store out neon cap
PointCloudT::Ptr neon_cloud (new PointCloudT);

// Message required to publish the cloud - Convert from pcl to msg
sensor_msgs::PointCloud2 cloud_ros;

void cloud_sub(const sensor_msgs::PointCloud2ConstPtr& msg)
{
        //convert the msg to PCL format
        pcl::fromROSMsg (*msg, *cloud);

        //state that a new cloud is available
        new_cloud_available_flag = true;
      
        /**PointCloudT::iterator myIterator;
        for(myIterator = cloud->begin();
            myIterator != cloud->end();
            myIterator++)
        {
            std::cout<<*myIterator<<" ";
        }**/
}

PointCloudT::Ptr computeNeonVoxels(PointCloudT::Ptr in) {
    int total_neon = 0;
 
    //Point Cloud to store out neon cap
    PointCloudT::Ptr temp_neon_cloud (new PointCloudT);

    for (int i = 0; i < in->points.size(); i++) {
        unsigned int r, g, b;
        r = in->points[i].r;
        g = in->points[i].g;
        b = in->points[i].b;
	//ROS_INFO("The rgb values of the neon cap is: (%d, %d, %d)", points[i].r, points[i].b, points[i].g);
	//ROS_INFO("The rgb values of the neon cap is: (%d, %d, %d)", r, g, b);
        // Look for mostly neon value points.
	if (g < 40 && (r + b) > 250) {//PINK
	    ROS_INFO("The rgb values of the neon cap is: (%d, %d, %d)", r, g, b);
            temp_neon_cloud->push_back(in->points[i]);
        }
    }

    return temp_neon_cloud;
}

ros::Publisher cmd_pub;
void moveRobot(double x, double y, double z, double d, int size){

	
   std::vector<double> state(2,0.);
   double angle = atan2(y,x);
   state[0] = d;
	ROS_INFO("D is: (%f)", d);
	ROS_INFO("Size is: (%d)", size);
 // ???  state[1] = angles::shortest_angular_distance(pose->theta,angle);

  if((d < 0.8 ) && size > 25) {
	return;
	}
	else{

	   geometry_msgs::Twist action = controller->computeAction(state);
   
	   if(!lastState.empty()) {
     		double reward = r(lastState,lastAction,state);
     		controller->learn(lastState,lastAction,reward,state, action);
	   }

	   lastState = state;
   	   lastAction = action;
   
	   cmd_pub.publish(action);

	}
}


int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "kinect_fun");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1000, cloud_sub);

    // Create a publisher object.
    cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 10);

    //debugging publisher --> can create your own topic and then subscribe to it through rviz
    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("detect_cap/cloud", 10);
 
    //refresh rate
    double ros_rate = 3.0;
    ros::Rate r(ros_rate);
 
    //keeps tracking of the time the robot has been moving
    ros::Time start_time = ros::Time::now();

    tf::TransformListener listener;

    double centroidX;
    double centroidY;
    double centroidZ;
    std::vector <pargo::BoundsPair> bounds;
    bounds.push_back(std::make_pair(0., 20.));
    bounds.push_back(std::make_pair(-M_PI, M_PI));
    
    controller = new ValueLearner(bounds, 5);

    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();
        
            ROS_INFO("Before voxel grid filter: %i points",(int)cloud->points.size());
        
        
            // Voxel Grid reduces the computation time. Its a good idea to do it if you will be doing
            //sequential processing or frame-by-frame
            // Create the filtering object: downsample the dataset using a leaf size of 1cm
            pcl::VoxelGrid<PointT> vg;
            pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
            vg.setInputCloud (cloud);
            vg.setLeafSize (0.005f, 0.005f, 0.005f);
            vg.filter (*cloud_filtered);
        
            ROS_INFO("After voxel grid filter: %i points",(int)cloud_filtered->points.size());
        
            int max_num_neon = 0;
        
            //Send the filtered point cloud to be processed in order to get the neon blob
            neon_cloud = computeNeonVoxels(cloud_filtered);
        
            //Publish the cloud with the neon cap
            pcl::toROSMsg(*neon_cloud,cloud_ros);
        
            //Set the frame ID to the first cloud we took in coz we want to replace that one
            cloud_ros.header.frame_id = cloud->header.frame_id;
            cloud_pub.publish(cloud_ros);
        
            // Find the centroid of the neon cap
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*neon_cloud, centroid);
        
            // centroid(0) = x ---- centroid(1) = y ---- centroid(2) = z
            ROS_INFO("The centroid of the neon cap is: (%f, %f, %f)", centroid(0), centroid(1), centroid(2));
        
            //------Compute position of centroid of the cap with respect to base_footprint.
            //------Will need to keep track of centroid by making variables and calling them at moveRobot.
            //------Remember to change locations from frame of reference of the kinect to that of the base
            tf::StampedTransform transform;

            listener.waitForTransform("/base_footprint", "/camera_depth_optical_frame", ros::Time(0), ros::Duration(5.0));
            listener.lookupTransform("/base_footprint", "/camera_depth_optical_frame", ros::Time(0), transform);

            //Creates new vector with reference points from the base instead of the kinect.
            tf::Vector3 framedCentroid(centroid(0), centroid(1), centroid(2));
            framedCentroid = transform * framedCentroid; //transform occurs here from kinect to base
      
            //Creates points
            centroidX = framedCentroid.getX();
            centroidY = framedCentroid.getY();
            centroidZ = framedCentroid.getZ();
	    
            double distance = sqrt((centroidX * centroidX) + (centroidY * centroidY));
		
            int cloudSize = neon_cloud->points.size();
	    moveRobot(centroidX, centroidY, centroidZ, distance, cloudSize);

    //    }
    }
	delete controller;
    return 0;
}

