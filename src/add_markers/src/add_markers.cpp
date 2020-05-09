#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

// Goal poses
float coord[2][2] = { {-1.0,1.0},{3.0,-2.0} };

bool goal_pickUp = false;
bool goal_dropOff = false;
bool dropOff_confirmed = false;
bool pickUp_confirmed = false;

float x_tolerance = 0.15;
float y_tolerance = 0.15;

void poseAMCLCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)

	{

            // Odom poses
	    float robot_x_pos             = msg->pose.pose.position.x;
	    float robot_y_pos             = msg->pose.pose.position.y;
  	    
	    ROS_INFO("robot_x_pos ... dist = %f", robot_x_pos);
	    ROS_INFO("robot_y_pos ... dist = %f", robot_y_pos);
	    float delta_x = std::abs(robot_x_pos - coord[goal_pickUp][0]);
	    float delta_y = std::abs (robot_y_pos - coord[goal_pickUp][1]);
	
	   if (delta_x < x_tolerance && delta_y < y_tolerance)  
		
		  {
		     if (!pickUp_confirmed)
			  goal_pickUp = true;
              	     else if (!dropOff_confirmed)
			  goal_dropOff = true;
		}
	   else
		ROS_INFO ("GOAL NOT YET REACHED");
        }

int main( int argc, char** argv )
{
ros::init(argc, argv, "add_markers");   
ros::NodeHandle n;
ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
ros::Subscriber sub_amcl = n.subscribe("amcl_pose", 100, poseAMCLCallBack);
// Set our initial shape type to be a cube
        uint32_t shape = visualization_msgs::Marker::CUBE;

       	while (ros::ok())
  	 {
	    visualization_msgs::Marker marker;
	    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
	    marker.header.frame_id = "map";
	    marker.header.stamp = ros::Time::now();

	    // Set the namespace and id for this marker.  This serves to create a unique ID
	    // Any marker sent with the same namespace and id will overwrite the old one
	    marker.ns = "add_markers";
	    marker.id = 0;

	    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	    marker.type = shape;

	    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	    marker.action = visualization_msgs::Marker::ADD;

	    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	    marker.pose.position.x = coord[0][0];
	    marker.pose.position.y = coord[0][1];
	    marker.pose.position.z = 0;
	    marker.pose.orientation.x = 0.0;
	    marker.pose.orientation.y = 0.0;
	    marker.pose.orientation.z = 0.0;
	    marker.pose.orientation.w = 1.0;

	    // Set the scale of the marker -- 1x1x1 here means 1m on a side
	    marker.scale.x = 0.45;
	    marker.scale.y = 0.45;
	    marker.scale.z = 0.45;

	    // Set the color -- be sure to set alpha to something non-zero!
	    marker.color.r = 0.0f;
	    marker.color.g = 1.0f;
	    marker.color.b = 0.0f;
	    marker.color.a = 1.0;

	    marker.lifetime = ros::Duration();

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

	    
            ROS_INFO("DISPLAY MARKER");
            marker_pub.publish(marker);
	
	    while(!goal_pickUp)
		{
			ros::spinOnce();
		}
		   
	   if (goal_pickUp and !pickUp_confirmed)
		{
		    marker.action = visualization_msgs::Marker::DELETE;
		    ROS_INFO("HIDE MARKER");
		    marker_pub.publish(marker);
	       
		    pickUp_confirmed = true;
		}
	    
	    while(!goal_dropOff)
		{
			ros::spinOnce();
		}
	   if (goal_dropOff && pickUp_confirmed)
		{
		    marker.action = visualization_msgs::Marker::ADD;
		    marker.pose.position.x = coord[1][0];
		    marker.pose.position.y = coord[1][1];
		   
		    marker_pub.publish(marker);

		    ROS_INFO("DROPPED THE BOX");
		    ros::Duration(5.0).sleep();
	    //r.sleep();
  		}
	}   


 ros::spin();
 return 0;
}
