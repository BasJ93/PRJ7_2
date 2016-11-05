#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>

//This application planes solvable positions for an ABB IRB120 arm.

int main(int argc, char **argv)
{
	ros::init(argc, argv, "prj7_move_test");

	ros::AsyncSpinner spinner(1);
	spinner.start();
	
	moveit::planning_interface::MoveGroup group("manipulator");
	moveit::planning_interface::MoveGroup::Plan my_plan;
	ros::Duration(2.0).sleep();
	
	float x,y,z;
	x = 0;
	y = 0.3;
	z = 0.8;
	
	int i = 0;
	
	while(ros::ok())
	{
		if(i == 2)
		{
			break;
		}
		x = 0;
		y = 0.3;
		z = 0.8;
		ROS_INFO("Planning for target %f, %f, %f", x, y, z);
		group.setPositionTarget(x, y, z, "");
		bool success = group.plan(my_plan);
		ROS_INFO("Planning result: %i", success);
		ros::Duration(2.0).sleep();
		if(success)
		{
			ROS_INFO("Moving to target %f, %f, %f", x, y, z);
			group.execute(my_plan);
		}
	
//		ros::Duration(3.0).sleep();
	
		x = -0.4;
		y = -0.3;
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
		i++;
	}	
	return 0;
}
