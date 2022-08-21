#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
void odom_clbk(nav_msgs::Odometry msg){
   static tf2_ros::TransformBroadcaster tfb;
   geometry_msgs::TransformStamped transformStamped;

  
    transformStamped.header.frame_id = "odom";
    transformStamped.child_frame_id = "base_footprint";
    transformStamped.transform.translation.x = msg.pose.pose.position.x;
    transformStamped.transform.translation.y = msg.pose.pose.position.y;
    transformStamped.transform.translation.z = msg.pose.pose.position.z;
    transformStamped.transform.rotation.x = msg.pose.pose.orientation.x;
    transformStamped.transform.rotation.y = msg.pose.pose.orientation.y;
    transformStamped.transform.rotation.z = msg.pose.pose.orientation.z;
    transformStamped.transform.rotation.w = msg.pose.pose.orientation.w;

    // ros::Rate rate(10.0);
    transformStamped.header.stamp = ros::Time::now();
    tfb.sendTransform(transformStamped);
    // rate.sleep();
    // std::cout<<transformStamped<<std::endl;
    printf("sending\n");
}
int main(int argc, char** argv){
    ros::init(argc, argv, "my_tf2_broadcaster");
    ros::NodeHandle node;

    ros::Subscriber sub= node.subscribe("/ground_truth",100,odom_clbk);
    ros::spin();
}