#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

// Define pick_up and drop_off goals
float pickup_x = 2.0;
float pickup_y = 2.0;
float dropoff_x = -2.0;
float dropoff_y = 3.0;

bool pick_up = false;
bool drop_off = false;

float tolerance = 0.6;

// Define callback function for odometry
void odometry_cb(const nav_msgs::Odometry::ConstPtr& msg) 
{
	float robot_pose_x = msg->pose.pose.position.x;
	float robot_pose_y = msg->pose.pose.position.y;
	
	float pickup_dis;
	float dropoff_dis;
	
	if(!pick_up && !drop_off)
	{
		pickup_dis = sqrt(pow((pickup_x - robot_pose_x), 2) + pow((pickup_y - robot_pose_y), 2));
		ROS_INFO("Distance to pick up zone = %f", pickup_dis);
		
		if(pickup_dis <= tolerance)
		{
			ROS_INFO("Arrived at the pick up zone");
			pick_up = true;
		}
	}
	if(pick_up && !drop_off)
	{
		dropoff_dis = sqrt(pow((dropoff_x - robot_pose_x), 2) + pow((dropoff_y - robot_pose_y), 2));
		ROS_INFO("Distance to drop off zone = %f", dropoff_dis);
		
		if(dropoff_dis <= tolerance)
		{
			ROS_INFO("Arrived at the drop off zone");
			drop_off = true;
		}
	}
}

int main( int argc, char** argv )
{
	ros::init(argc, argv, "add_markers");
  	ros::NodeHandle n;
  	ros::Rate r(1);
  	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  	// Subscribe to odometry values
  	ros::Subscriber odometry_sub = n.subscribe("/odom", 1000, odometry_cb);

  	// Set our initial shape type to be a cube
  	uint32_t shape = visualization_msgs::Marker::CUBE;

  	// Initialize marker
  	visualization_msgs::Marker marker;

  	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
  	marker.header.frame_id = "map";
  	marker.header.stamp = ros::Time::now();

  	// Set the namespace and id for this marker.  This serves to create a unique ID
  	// Any marker sent with the same namespace and id will overwrite the old one
  	marker.ns = "basic_shapes";
  	marker.id = 0;

  	// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  	marker.type = shape;

  	// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  	marker.action = visualization_msgs::Marker::ADD;

  	// Set the scale of the marker -- 1x1x1 here means 1m on a side
  	marker.scale.x = 1.0;
  	marker.scale.y = 1.0;
  	marker.scale.z = 1.0;
	
  	// Set the color -- be sure to set alpha to something non-zero!
  	marker.color.r = 0.0f;
  	marker.color.g = 1.0f;
  	marker.color.b = 0.0f;
  	marker.color.a = 1.0;
	
  	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.position.x = pickup_x;
  	marker.pose.position.y = pickup_y;
  	marker.pose.position.z = 0;
  	marker.pose.orientation.x = 0.0;
  	marker.pose.orientation.y = 0.0;
  	marker.pose.orientation.z = 0.0;
  	marker.pose.orientation.w = 1.0;
	
	marker.lifetime = ros::Duration();

  	while (ros::ok())
  	{
		// Publish the marker
		while (marker_pub.getNumSubscribers() < 1)
		{
			if (!ros::ok())
			{
				return 0;
			}
			ROS_WARN_ONCE("Please create a subscriber to the marker");
			sleep(1);
		}
		
		if(pick_up)
		{
			marker.action = visualization_msgs::Marker::DELETE;
			ROS_INFO("Object was pickerd up");
			ros::Duration(2.0).sleep();
		}
		
		if(drop_off)
		{
			marker.pose.position.x = dropoff_x;
			marker.pose.position.y = dropoff_y;
			marker.action = visualization_msgs::Marker::ADD;
			ROS_INFO("Object was dropped off");
			ros::Duration(2.0).sleep();
		}
		
		marker_pub.publish(marker);
		
		ros::spinOnce();
	}
	
	return 0;
}