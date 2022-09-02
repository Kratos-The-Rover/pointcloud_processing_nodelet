#include <nodelet/nodelet.h>
#include <grid_map_msgs/GridMap.h>
#include <boost/thread/mutex.hpp>
#include <processing_nodelet/pointcloud_nodelets.h>
#include <ros/ros.h>
#include <grid_map_core/GridMap.hpp>
struct Node{
                float x;
                float y;
                float z;
                float cost;
                int index;
                Node * parent;
                bool processed;
            };
namespace planner{
    class Plan : public nodelet::Nodelet{
        public:
            Plan();
        private:
            virtual void onInit();
            Subscriber<grid_map_msgs::GridMap> map_sub;
            void mapCallback(const grid_map_msgs::GridMapConstPtr& map_msg_ptr);
            void plan();
            ros::NodeHandle nh_;
            ros::ServiceServer plan_server;
            int count;
            boost::shared_ptr <grid_map::GridMap>  map_ptr;

            ros::Time begin;
            
            std::vector<Node>* getNeighbours(Node* node);



    };
}
