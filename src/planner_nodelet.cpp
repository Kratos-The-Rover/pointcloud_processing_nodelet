#include <processing_nodelet/planning_nodelet.h>
#include <grid_map_msgs/GridMap.h>
#include <pluginlib/class_list_macros.h>
#include <pluginlib/class_list_macros.hpp>
// #include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
namespace planner
{
    Plan::Plan(){}

    void Plan::onInit(){
        nh_=getMTNodeHandle();
        map_sub.registerCallback(boost::bind(&Plan::mapCallback,this,_1));
        map_sub.subscribe(nh_,"clouder",1);
        count=0;
        map_ptr.reset(new grid_map::GridMap);
        begin=begin.now();
    }

    void Plan::mapCallback(const grid_map_msgs::GridMapConstPtr& map_msg_ptr){
        
        grid_map::GridMapRosConverter::fromMessage(*map_msg_ptr,*map_ptr);
        
        std::cout<<"frequency: "<<(1/(ros::Time::now()-begin).toSec())<<" begin: "<<begin<<std::endl;
        begin=begin.now();
    }

    void Plan::plan(){
        float begin_x=0;
        float begin_y=0;

        float goal_x=20;
        float goal_y=20;

        Node start;
        start.x=begin_x;
        


    }

    std::vector <Node>* getNeighbours(Node* Node){

    }
} // namespace 
PLUGINLIB_EXPORT_CLASS(planner::Plan, nodelet::Nodelet)
