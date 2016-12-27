#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometric_shapes/shapes.h>
#include <geometry_msgs/Pose.h>
#include <string.h>
#include <prj7_vision_test/prj7_box.h>
#include <math.h>


//Currend fov is 90x65cm at 1024x768 -> 0.9mm/pixel

ros::Publisher collis_pub;

std::string frame_id_ = "base_link";

void marker_place_block(ros::Publisher vis_pub, int id, float x_scale, float y_scale, float z_scale, float x, float y, float z)
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = frame_id_;
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.id = id;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = x;
	marker.pose.position.y = y;
	marker.pose.position.z = z;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	//scale = size, scale 1 = 1m
	marker.scale.x = x_scale;
	marker.scale.y = y_scale;
	marker.scale.z = z_scale;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;
	vis_pub.publish( marker );
}

void collision_place_block(ros::Publisher collis_pub, int id, float x_scale, float y_scale, float z_scale, float x, float y, float z, float rotation)
{
	//add the box into the collision space
	moveit_msgs::CollisionObject col_box_msg;
	geometry_msgs::Pose pose;
	col_box_msg.id = "collision_box_" + std::to_string(id);
	col_box_msg.header.frame_id = frame_id_;
	col_box_msg.operation = moveit_msgs::CollisionObject::ADD;
	shape_msgs::SolidPrimitive box;
	box.type = shape_msgs::SolidPrimitive::BOX;
	box.dimensions.push_back(x_scale);
	box.dimensions.push_back(y_scale);
	box.dimensions.push_back(z_scale);

	pose.position.x = x;
	pose.position.y = y;
	pose.position.z = z;
	pose.orientation.x = 0;
	pose.orientation.y = 0;
	pose.orientation.z = rotation * (M_PI / 180);
	pose.orientation.w = 1.0;

	col_box_msg.primitives.push_back(box);
	col_box_msg.primitive_poses.push_back(pose);

	collis_pub.publish(col_box_msg);
  
}

void collision_remove_block(ros::Publisher collis_pub, int id)
{
	moveit_msgs::CollisionObject col_box_msg;
	col_box_msg.id = "collision_box_" + std::to_string(id);
	col_box_msg.header.frame_id = frame_id_;
	col_box_msg.operation = moveit_msgs::CollisionObject::REMOVE;
	
	collis_pub.publish(col_box_msg);
}

void collision_move_block(ros::Publisher collis_pub, int id, float x, float y, float z)
{
	//add the box into the collision space
	moveit_msgs::CollisionObject col_box_msg;
	geometry_msgs::Pose pose;
	col_box_msg.id = "collision_box_" + std::to_string(id);
	col_box_msg.header.frame_id = frame_id_;
	col_box_msg.operation = moveit_msgs::CollisionObject::MOVE;

	pose.position.x = x;
	pose.position.y = y;
	pose.position.z = z;
	pose.orientation.w = 1.0;

	col_box_msg.primitive_poses.push_back(pose);

	collis_pub.publish(col_box_msg);
  
}

void objectCallback(const prj7_vision_test::prj7_box::ConstPtr& msg)
{
	collision_remove_block(collis_pub, msg->id);
	collision_place_block(collis_pub, msg->id, msg->width * 0.001, msg->height * 0.001, 2, msg->x * 0.001, msg->y * 0.001, 0, msg->rotation); 
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "prj7_place_object");

	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::NodeHandle n;
	ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
	collis_pub = n.advertise<moveit_msgs::CollisionObject>("collision_object", 0);
	ros::Duration(2.0).sleep();// This delay is so critical, otherwise the first published object may not be added in the collision_space by the environment_server
	ros::Subscriber obj_sub = n.subscribe("prj7_box_bluefox", 1000, objectCallback);

	while(ros::ok())
	{
		ROS_INFO("Cleaning up objects");
		for(int i=0; i<200; i++)
		{
			collision_remove_block(collis_pub, i);
		}
		ros::Duration(1.0).sleep();
	}
	return 0;
}
