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
  for(auto i:remap){
    std::cout<<i.first<<" "<<i.second<<std::endl;
  }
  for(auto i: nargv){
  

    std::cout<<i<<std::endl;
  }
  std::string nodelet_name = ros::this_node::getName();
  std::cout<<nodelet_name<<std::endl;
  // nodelet.load();
  // nodelet::M_string remap(ros::names::getRemappings());
  // nodelet::V_string nargv;
  // std::string nodelet_name = ros::this_node::getName();
  // nodelet_name = ros::this_node::getName();
  std::cout<<nodelet_name<<std::endl;
  // nodelet.load();

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
