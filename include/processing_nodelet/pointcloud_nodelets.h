#ifndef POINTCLOUD_TO_LASERSCAN_POINTCLOUD_TO_LASERSCAN_NODELET_H
#define POINTCLOUD_TO_LASERSCAN_POINTCLOUD_TO_LASERSCAN_NODELET_H


#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>


#include <nodelet/nodelet.h>
#include <boost/thread/mutex.hpp>


#include <message_filters/subscriber.h>


#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"


#include <string>


#include <Eigen/Dense>



#include <cublas_v2.h>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


class SubscriberBase
 {
 public:
   virtual ~SubscriberBase() {}
   virtual void subscribe(ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size, const ros::TransportHints& transport_hints = ros::TransportHints(), ros::CallbackQueueInterface* callback_queue = 0) = 0;
   virtual void subscribe() = 0;
   virtual void unsubscribe() = 0;
 };
 typedef boost::shared_ptr<SubscriberBase> SubscriberBasePtr;
  
 template<class M>
 class Subscriber : public SubscriberBase, public message_filters::SimpleFilter<M>
 {
 public:
   typedef boost::shared_ptr<M const> MConstPtr;
   typedef ros::MessageEvent<M const> EventType;
  
   Subscriber(ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size, const ros::TransportHints& transport_hints = ros::TransportHints(), ros::CallbackQueueInterface* callback_queue = 0)
   {
     subscribe(nh, topic, queue_size, transport_hints, callback_queue);
   }
  
   Subscriber()
   {
   }
  
   ~Subscriber()
   {
     Subscriber::unsubscribe();
   }
  
   void subscribe(ros::NodeHandle& nh, const std::string& topic, uint32_t queue_size, const ros::TransportHints& transport_hints = ros::TransportHints(), ros::CallbackQueueInterface* callback_queue = 0)
   {
     unsubscribe();
  
     if (!topic.empty())
     {
       ops_.template initByFullCallbackType<const EventType&>(topic, queue_size, boost::bind(&Subscriber<M>::cb, this, boost::placeholders::_1));
       ops_.callback_queue = callback_queue;
       ops_.transport_hints = transport_hints;
       ops_.allow_concurrent_callbacks=true;
       sub_ = nh.subscribe(ops_);
       nh_ = nh;
     }
   }
  
   void subscribe()
   {
     unsubscribe();
  
     if (!ops_.topic.empty())
     {
       sub_ = nh_.subscribe(ops_);
     }
   }
  
   void unsubscribe()
   {
     sub_.shutdown();
   }
  
   std::string getTopic() const
   {
     return ops_.topic;
   }
  
   const ros::Subscriber& getSubscriber() const { return sub_; }
  
   template<typename F>
   void connectInput(F& f)
   {
     (void)f;
   }
  
   void add(const EventType& e)
   {
     (void)e;
   }
  
 private:
  
   void cb(const EventType& e)
   {
     this->signalMessage(e);
   }
  
   ros::Subscriber sub_;
   ros::SubscribeOptions ops_;
   ros::NodeHandle nh_;
 };
  

















namespace Pointcloud_Nodelet_learn
{


  typedef tf2_ros::MessageFilter<sensor_msgs::PointCloud2> CloudMessageFilter;
  typedef tf2_ros::MessageFilter<nav_msgs::Odometry> OdomMessageFilter;


  class PointcloudProcessorNodelet : public nodelet::Nodelet
  {
    public:
        PointcloudProcessorNodelet();
    private:
        virtual void onInit();

    // void CloudFailureCallback(const pcl::PointCloud <pcl::PointXYZRGB>::Ptr& cloud_msg, tf2_ros::filter_failure_reasons::FilterFailureReason reason);
    void connectCallback();
    void disconnectCallback();
    void createHostVector(std::vector<float>* HostVec, const sensor_msgs::PointCloud2ConstPtr& cloud_msg, ros::NodeHandle* nh);

    pcl::PointCloud <pcl::PointXYZRGB>::Ptr cleanCloud(const pcl::PointCloud <pcl::PointXYZRGB>::ConstPtr& pclCloud);
    void transformCloud(pcl::PointCloud <pcl::PointXYZRGB>::Ptr pclCloud);
    
    ros::NodeHandle nh, private_nh;
    ros::Publisher pub;
    


    boost::mutex connect_mutex;
    boost::shared_ptr<tf2_ros::Buffer> tf2;
    boost::shared_ptr<tf2_ros::TransformListener> tf2_listener;


    Subscriber<pcl::PointCloud <pcl::PointXYZRGB>> cloud_sub_;
    boost::shared_ptr<CloudMessageFilter> cloud_message_filter_;
    
    cublasHandle_t h;


    void CloudCallBack(const pcl::PointCloud <pcl::PointXYZRGB>::ConstPtr& pclCloud);


    // ROS Parameters

    unsigned int input_queue_size_;
    std::string cloud_target_frame_;
    double tolerance_;
    bool use_inf_;
    double inf_epsilon_;

  };
}
#endif  // POINTCLOUD_TO_LASERSCAN_POINTCLOUD_TO_LASERSCAN_NODELET_H