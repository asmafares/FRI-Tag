//This bot reacts to orange!
extern "C" {
#include "client.h"
}


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

/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
bool stop = false;

bool new_cloud_available_flag = false;
geometry_msgs::Twist cmd;

// General point cloud to store the whole image
PointCloudT::Ptr cloud (new PointCloudT);

//Point Cloud to store out neon cap
PointCloudT::Ptr neon_cloud (new PointCloudT);

// Message required to publish the cloud - Convert from pcl to msg
sensor_msgs::PointCloud2 cloud_ros;

bool moveSwitch = false;
int moveTimer = 0;
int randomVal = 0;
int stopTimes = 0;
int backTimes = 0;

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
   PointCloudT::Ptr temp_obstacle_cloud (new PointCloudT);
   for (int i = 0; i < in->points.size(); i++) {
      unsigned int r, g, b;
      double y, z;
      r = in->points[i].r;
      g = in->points[i].g;
      b = in->points[i].b;
      y = in->points[i].y;
      z = in->points[i].z;
      //ROS_INFO("The rgb values of the neon cap is: (%d, %d, %d)", points[i].r, points[i].b, points[i].g);
      //ROS_INFO("The rgb values of the neon cap is: (%d, %d, %d)", r, g, b);
      // Look for mostly neon value points.
    
	if (r > 205 && r < 230 && g > 70 && g < 95 && b > 40 && b < 80) {//ORANGE
        	ROS_INFO("The rgb values of the neon cap is: (%d, %d, %d)", r, g, b);
        	temp_neon_cloud->push_back(in->points[i]);
        }
	else if((-.5 < y) && (y < .5) && (z > 0) && (z < .45)){
        	ROS_INFO("The y z of the obstacle is: (%f, %f)", y, z);
		temp_obstacle_cloud->push_back(in->points[i]);
	

		if(temp_obstacle_cloud->points.size() > 50) {
			ROS_INFO("I see an obstacle.");
			stop = true;
		}
     	}
   }
   return temp_neon_cloud;
}

ros::Publisher cmd_pub;

void moveRobot(double x, double y, double z, double computedLinearVelocity){
   if(computedLinearVelocity > 0.75){//robot to close to cap, must move away from it.
	if(stop){
		if(stopTimes <= 1){
ROS_INFO("stop %d", stopTimes);
	 	  cmd.linear.x = 0.0;
		  cmd.angular.z = 0.0;
		  stopTimes++;
		}
		else if(backTimes <= 5){

ROS_INFO("back %d", backTimes);
		  cmd.linear.x = -0.1;
		  cmd.angular.z = 0.0;
		  backTimes++;
		}
		else{

ROS_INFO("turn to cap");
			cmd.angular.z = atan2(y, x);
			cmd.linear.x = 0.1;
			stop = false;
		}
	}
	else{
		backTimes = 0;
		stopTimes = 0;
      		cmd.linear.x = std::min(computedLinearVelocity, 0.2);
    		cmd.angular.z = atan2(y, x);
	}

      // Send command to turtle
      cmd_pub.publish(cmd);
   }
   else{
	//Do something here. Like send caught signal here.
	
	sendMessage();

	double startTime = ros::Time::now().toSec();
	double endTime = startTime;

	while((endTime - startTime) < 10){
	   endTime = ros::Time::now().toSec();
 	}
   }
}

void turn(){
   if(stop){
	if(stopTimes <= 1){
ROS_INFO("stop %d", stopTimes);
	  cmd.linear.x = 0.0;
	  cmd.angular.z = 0.0;
	  stopTimes++;
	}
	else if(backTimes <= 5){

ROS_INFO("back %d", backTimes);
	  cmd.linear.x = -0.1;
	  cmd.angular.z = 0.0;
	  backTimes++;
	}
	else{

ROS_INFO("turn");
	  cmd.linear.x = 0.1;
	  cmd.angular.z = 1.0;
	}
   }
   else{
	stopTimes = 0;
	backTimes = 0;
      if(!moveSwitch) {
         ROS_INFO("I am spinning.");
         cmd.linear.x = 0.1;
         cmd.angular.z = 1.;
      }
      else{//moveSwitch is true
         if(moveTimer == 0){
	   time_t seconds;
	   time(&seconds);
	   srand((unsigned int) seconds);
	   randomVal = rand() % 4 + 1; // Random number between 1 and 4
         }

         if(randomVal == 1 && moveTimer <= 4){ //Move right
	      ROS_INFO("Move right == randomVal: %d  moveTimer: %d", randomVal, moveTimer);	   
	      if(moveTimer == 4){
	  	  moveTimer = 8;
              }
	      cmd.angular.z = -1.;
	      cmd.linear.x = 0.1;
         }
         else if(randomVal == 2 && moveTimer <= 4){ //Move left
	      ROS_INFO("Move left == randomVal: %d  moveTimer: %d", randomVal, moveTimer);
	      if(moveTimer == 4){
		   moveTimer = 8;
              }
	      cmd.angular.z = 1.;
	      cmd.linear.x = 0.1;
         }
         else if(randomVal == 3 && moveTimer <= 8){ //Move back
	      ROS_INFO("Move back == randomVal: %d  moveTimer: %d", randomVal, moveTimer);
	      if(moveTimer == 8){
		   moveTimer = 8;
	      }
	      cmd.angular.z = 1.;
	      cmd.linear.x = 0.1;
         }
         else if(randomVal == 4 && moveTimer <= 8){ //Move forward 
	      ROS_INFO("Move up == randomVal: %d  moveTimer: %d", randomVal, moveTimer);
   	      if(moveTimer == 0){
		   moveTimer = 8;
	      }
	      cmd.angular.z = 0;
              cmd.linear.x = .2;
         }
         else{ // Finished turning, now go straight.
   	   ROS_INFO("randomVal: %d, moveTimer: %d", randomVal, moveTimer);
           cmd.angular.z = 0;
           cmd.linear.x = .2;
         }
      }
      moveTimer++;
   }
   // Send command to turtle
   cmd_pub.publish(cmd);
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
   double ros_rate = 2.;
   ros::Rate r(ros_rate);

   //keeps tracking of the time the robot has been moving
   ros::Time start_time = ros::Time::now();
   tf::TransformListener listener;

   double centroidX;
   double centroidY;
   double centroidZ;

   while (ros::ok())
   {
	
	stop = false;
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

	if(moveTimer >= 16){
	   moveSwitch = !moveSwitch;
	   moveTimer = 0;
	}
	ROS_INFO("I got here.");
	if(neon_cloud->points.size() > 35) {
		moveSwitch = !moveSwitch;
		moveTimer = 0;

		ROS_INFO("I am trying to get to the cloud.");
		moveRobot(centroidX, centroidY, centroidZ, distance);
	}
	else{ // Distance <= 0.75 and cloud size is <= 35
		turn();
	}
   }
   return 0;
}
