//This bot reacts to orange!

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

#include <stdlib.h>
#include <time.h>

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

// Variables for smarter movement
bool spun = false;
bool seen = false;
double lastX = -1000;
double lastY = -1000;
double lastZ = -1000;

void cloud_sub(const sensor_msgs::PointCloud2ConstPtr& msg) {
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
     seen = false;
     if (r > 205 && r < 230 && g > 70 && g < 95 && b > 40 && b < 80) {//ORANGEo
	 seen = true;
         ROS_INFO("The rgb values of the neon cap is: (%d, %d, %d)", r, g, b);
         temp_neon_cloud->push_back(in->points[i]);
     }
   }
   return temp_neon_cloud;
}


ros::Publisher cmd_pub;

void moveRobot(double x, double y, double z, double computedLinearVelocity){

   if(computedLinearVelocity > 1.20){//robot to close to cap, must move away from it.
      cmd.linear.x = std::min(computedLinearVelocity, 0.2);
   }
   else if(computedLinearVelocity < 0.80){//robot too far from cap, move towards it.
      double newLinearVelocity = (-1) * computedLinearVelocity;
      cmd.linear.x = std::max(newLinearVelocity, -0.2);
   }

   // Turn towards target
   double computedAngularVelocity = atan2(y, x);
   cmd.angular.z = computedAngularVelocity;

   // Send command to turtle
   cmd_pub.publish(cmd);
}

void moveRobot2(double movementTime, double rotateTime, bool turnLeft){

   //Rotates x degrees
   double startTime = ros::Time::now().toSec();
   double endTime = startTime;

   cmd.angular.z = -1;
   if(turnLeft){
	cmd.angular.z = 1;
   }

   cmd.linear.x = 0.1;

   while(!seen /*!(neon_cloud->points.size() > 15)*/ && (endTime - startTime) < rotateTime){
	endTime = ros::Time::now().toSec();
	cmd_pub.publish(cmd);
   }

   //Moves forward for x times.
   cmd.angular.z = 0;
   cmd.linear.x = .2;

   startTime = ros::Time::now().toSec();
   endTime = startTime;

   while(!seen /*!(neon_cloud->points.size() > 15)*/ && (endTime - startTime) < movementTime){
	endTime = ros::Time::now().toSec();
	cmd_pub.publish(cmd);
   }
}

void turn(){

   double startTime = ros::Time::now().toSec();
   double endTime = startTime;

   while(!seen /*!(neon_cloud->points.size() > 15)*/ && (endTime - startTime) < 8.68){
	endTime = ros::Time::now().toSec();

//------------------------------------------------------------//
	//ROS_INFO("I am spinning.");
	
	cmd.linear.x = 0.1;
	
	// Turn towards target
	cmd.angular.z = 1.;
	
	// Send command to turtle
	cmd_pub.publish(cmd);
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

   while (ros::ok())
   {
	ros::spinOnce();
	r.sleep();
	ROS_INFO("I am currently running.");

	// if (new_cloud_available_flag){
	// new_cloud_available_flag = false;

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
	ROS_INFO("WORK");

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

	if((distance < 0.80 || distance > 1.20) && neon_cloud->points.size() > 25) {
		ROS_INFO("I am trying to get to the cloud.");
		moveRobot(centroidX, centroidY, centroidZ, distance);

		lastX = centroidX;
		lastY = centroidY;
		lastZ = centroidZ;
		spun = false;
	}
	else if (neon_cloud->points.size() <= 25) {
		if(!spun){
			spun = true;
			turn();
		}
		else{
			spun = false;

			time_t seconds;
			time(&seconds);
			srand((unsigned int) seconds);
			int randomVal = rand() % 4 + 1; // Random number between 1 and 10

			if(lastX == -1000){
				lastX = 0;
				lastY = 0;
				lastZ = 0;
			}

			bool moveLeft = false;
			double rotateTime = 2.17;

			if(randomVal != 1){
				if(randomVal == 2){
					moveLeft = true;
				}
				else if(randomVal == 3){
					rotateTime = 0;
				}
				else{
					rotateTime = 4.34;
				}
			}
		}
	}
   }
   return 0;
}

