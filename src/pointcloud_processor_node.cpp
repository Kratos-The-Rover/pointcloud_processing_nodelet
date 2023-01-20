#include <nodelet/loader.h>
#include <nodelet/NodeletLoad.h>
#include <ros/ros.h>
#include <string>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud_processor_node");
  ros::NodeHandle private_nh("~");
  int concurrency_level;
  private_nh.param<int>("concurrency_level", concurrency_level, 0);

  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;
  std::string nodelet_name = ros::this_node::getName();
  nodelet_name = ros::this_node::getName();
  nodelet.load(nodelet_name, "processing_nodelet/pointcloud_processor_nodelet", remap, nargv);

  // nodelet.load()

  boost::shared_ptr<ros::MultiThreadedSpinner> spinner;
  if (concurrency_level)
  {
    spinner.reset(new ros::MultiThreadedSpinner(concurrency_level));
  }
  else
  {
    spinner.reset(new ros::MultiThreadedSpinner(8));
  }
  spinner->spin();
  return 0;
}
