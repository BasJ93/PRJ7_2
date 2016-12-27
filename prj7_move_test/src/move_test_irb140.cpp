#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <geometry_msgs/Pose.h>

//This application planes solvable positions for an ABB IRB140 arm.

int main(int argc, char **argv)
{
	ros::init(argc, argv, "prj7_move_test");

	ros::AsyncSpinner spinner(1);
	spinner.start();
	
	moveit::planning_interface::MoveGroup group("manipulator");
	ros::Duration(2.0).sleep();
	
	geometry_msgs::Pose pose;

	float x,y,z,roll,pitch,yaw;
	
	int i = 0;

	group.allowLooking(true);
	group.allowReplanning(true);
	
	while(ros::ok())
	{
//		if(i == 2)
//		{
//			break;
//		}
		moveit::planning_interface::MoveGroup::Plan my_plan;
		x = 0.2;
		y = 0;
		z = 0.7;
		roll = 0;
		pitch = 0;
		yaw = 0;
		ROS_INFO("Planning for target %f, %f, %f", x, y, z);
		pose.position.x = x;
		pose.position.y = y;
		pose.position.z = z;
		pose.orientation.x = roll;
		pose.orientation.y = pitch;
		pose.orientation.z = yaw;
		pose.orientation.w = 1.0;
		group.setPoseTarget(pose, "");
		bool success = group.plan(my_plan);
		while(!success)
		{
			ros::Duration(1.0).sleep();
			success = group.plan(my_plan);
		}
		ROS_INFO("Planning result: %i", success);
		ros::Duration(2.0).sleep();
		if(success)
		{
			ROS_INFO("Moving to target %f, %f, %f", x, y, z);
			group.execute(my_plan);
		}
	
		ros::Duration(3.0).sleep();
	
		moveit::planning_interface::MoveGroup::Plan my_plan2;
		x = 0.8;
		y = 0;
		z = 0.4;
		ROS_INFO("Planning for target %f, %f, %f", x, y, z);
		pose.position.x = x;
		pose.position.y = y;
		pose.position.z = z;
		pose.orientation.x = roll;
		pose.orientation.y = pitch;
		pose.orientation.z = yaw;
		pose.orientation.w = 1.0;
		group.setPoseTarget(pose, "");
		success = group.plan(my_plan2);
		while(!success)
		{
			ros::Duration(1.0).sleep();
			success = group.plan(my_plan2);
		}
		ROS_INFO("Planning result: %i", success);
		ros::Duration(2.0).sleep();
		if(success)
		{
			ROS_INFO("Moving to target %f, %f, %f", x, y, z);
			group.execute(my_plan2);
		}
	
		ros::Duration(3.0).sleep();
/*	
		x = 0.2;
		y = -0.4;
		z = 0.7;
		ROS_INFO("Planning for target %f, %f, %f", x, y, z);
		group.setPositionTarget(x, y, z, "");
		success = group.plan(my_plan);
		ROS_INFO("Planning result: %i", success);
		ros::Duration(2.0).sleep();
		if(success)
		{
			ROS_INFO("Moving to target %f, %f, %f", x, y, z);
			group.execute(my_plan);
		}
	
//		ros::Duration(3.0).sleep();
		
		x = 0;
		y = -0.6;
		z = 0.5;
		ROS_INFO("Planning for target %f, %f, %f", x, y, z);
		group.setPositionTarget(x, y, z, "");
		success = group.plan(my_plan);
		ROS_INFO("Planning result: %i", success);
		ros::Duration(2.0).sleep();
		if(success)
		{
			ROS_INFO("Moving to target %f, %f, %f", x, y, z);
			group.execute(my_plan);
		}
	
//		ros::Duration(3.0).sleep();
*/		i++;
	}	
	return 0;
}
