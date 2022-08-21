#include <nodelet/nodelet.h>
#include <grid_map_msgs/GridMap.h>
#include <boost/thread/mutex.hpp>
#include <processing_nodelet/pointcloud_nodelets.h>
namespace map_subscriber{
    class MapSub:nodelet::Nodelet{
        public:
            MapSub();
        private:
            virtual void onInit();
            void mapCallback(grid_map_msgs::GridMapConstPtr& msg_ptr);
            Subscriber<grid_map_msgs::GridMap> map_sub;
            

    };
}
