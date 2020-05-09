#include <ros/ros.h>
#include <tf/tf.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
 
 int number_of_goals = 2;
 float coord[2][3] = { {-1.0,1.0,1.57},{3.0,-2.0,-1.57} };
 
 for (int i = 0; i< number_of_goals;i++)
  {
	  goal.target_pose.pose.position.x = coord[i][0];
          goal.target_pose.pose.position.y = coord[i][1];
      	  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(coord[i][2]);

          
	  ROS_INFO("Sending goal");
	  ac.sendGoal(goal);

	  ac.waitForResult();

	  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	    ROS_INFO("Goal reached!");
	  else
	    ROS_INFO("Failed to reach goal");
	  ros::Duration(5.0).sleep();
  }

	  return 0;
}
