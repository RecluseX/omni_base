#include <iostream>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include "sensor_msgs/LaserScan.h"
#include "tf2_ros/transform_broadcaster.h"	


class Node {

	public:
		Node();
		~Node();
		bool initialize();
		void spinForever();
		
		float vx_;
		float vy_;
		float vw_;	
		float xx_;
		float yy_;
		float theta_;
		ros::Time current_time, last_time;

	private:
		ros::Publisher odom_pub_;
		ros::Publisher scan_pub_;
		ros::Subscriber cmd_sub_;
	        tf2_ros::TransformBroadcaster odom_broadcaster_;
		ros::NodeHandle node_handle_;
		ros::NodeHandle private_nh_;	
		
	        std::vector<ros::WallTimer> wallTimers_;
		
		
       		void PublishOdom(const ros::WallTimerEvent& event);
	        void PublishScan(const ros::WallTimerEvent& event);
        	void HandleVelCMD(const geometry_msgs::TwistConstPtr&  vel_cmd);

};

Node::Node()
{

}

Node::~Node()
{

}

bool Node::initialize()
{
      wallTimers_.push_back(node_handle_.createWallTimer(ros::WallDuration(0.02),&Node::PublishOdom,this));


	odom_pub_ = node_handle_.advertise<nav_msgs::Odometry>("/odom",10);
	cmd_sub_ = node_handle_.subscribe("/cmd_vel", 1, &Node::HandleVelCMD, this);
	
	ROS_INFO("node initialize");
	
	return true;
}


void Node::HandleVelCMD(const geometry_msgs::TwistConstPtr&  vel_cmd)
{
	geometry_msgs::Twist msg = *vel_cmd;
	vx_ = msg.linear.x;
	vy_ = msg.linear.y;
	vw_ = msg.angular.z; 
}


void Node::PublishOdom(const ros::WallTimerEvent& event)
{
	current_time = ros::Time::now();
	
	double dt = current_time.toSec() - last_time.toSec();
		
	float deltaX = vx_ * dt;
	float deltaY = vy_ * dt;
	float deltaYaw = vw_ * dt;
	
	float dx_world = std::cos(theta_)*deltaX - std::sin(theta_)*deltaY;
	float dy_world = std::sin(theta_)*deltaX + std::cos(theta_)*deltaY;

	xx_ += dx_world;
	yy_ += dy_world;
	theta_ += deltaYaw;

	//publish tf
     	geometry_msgs::Quaternion yaw_quat = tf::createQuaternionMsgFromYaw(theta_);
     	geometry_msgs::TransformStamped odom_trans;
     	odom_trans.header.stamp = current_time;
     	odom_trans.header.frame_id = "odom";
     	odom_trans.child_frame_id = "base_link";
     	odom_trans.transform.translation.x = xx_;
     	odom_trans.transform.translation.y = yy_;
     	odom_trans.transform.translation.z = 0.0;
     	odom_trans.transform.rotation = yaw_quat;

     	//publish odometry
     	nav_msgs::Odometry odometry;
     	odometry.header.stamp = current_time;
     	odometry.header.frame_id = "odom";

     	odometry.pose.pose.position.x = xx_;
     	odometry.pose.pose.position.y = yy_;
     	odometry.pose.pose.position.z = 0.0;
     	odometry.pose.pose.orientation = yaw_quat;

     	odometry.child_frame_id = "base_link";
     	odometry.twist.twist.linear.x = vx_;
     	odometry.twist.twist.linear.y = vy_;
     	odometry.twist.twist.linear.z = 0;

     	odometry.twist.twist.angular.x = 0;
     	odometry.twist.twist.angular.y = 0 ;
     	odometry.twist.twist.angular.z = vw_;

     	odom_pub_.publish(odometry);
     	odom_broadcaster_.sendTransform(odom_trans);

	last_time = current_time;
}

void Node::spinForever()
{
	ros::spin();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "base_control_node");
	Node nd;
	
	if(nd.initialize() == false) {
		ROS_INFO("init failed");
		return 1;
	}

	nd.spinForever();

	return 0;
}

