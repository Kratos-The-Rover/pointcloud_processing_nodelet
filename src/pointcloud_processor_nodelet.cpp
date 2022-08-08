#include <pluginlib/class_list_macros.hpp>
#include <processing_nodelet/pointcloud_nodelets.h>
#include <pluginlib/class_list_macros.h>

#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_field_conversion.h>
#include <string>


#include <tf2_ros/buffer.h>
#include <tf2/buffer_core.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>


#include <Eigen/Dense>


#include <geometry_msgs/TransformStamped.h>



#include <thrust/device_vector.h>
#include <cublas_v2.h>
#include <thrust/copy.h>
#include <cuda_runtime.h>
 /*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/






namespace Pointcloud_Nodelet_learn
{
    PointcloudProcessorNodelet::PointcloudProcessorNodelet(){
            
    }




    void PointcloudProcessorNodelet::onInit()
    {

                    cublasCreate(&h);
        NODELET_INFO("initialized");
        boost::mutex::scoped_lock lock(connect_mutex);
        private_nh=getPrivateNodeHandle();



        int concurrency_level;


        private_nh.param<std::string>("cloud_target_frame", cloud_target_frame_, "");
        private_nh.param<double>("transform_tolerance", tolerance_, 0.01);
        private_nh.param<double>("inf_epsilon", inf_epsilon_, 1.0);
        private_nh.param<int>("concurrency_level", concurrency_level, 1);
        private_nh.param<bool>("use_inf", use_inf_, true);



        if (concurrency_level == 1)
        {
            nh= getNodeHandle();
        }
        
        else
        {
            nh= getMTNodeHandle();
        }


        

        if (concurrency_level > 0)
        {
            input_queue_size_ = concurrency_level;
        }
        else
        {
            input_queue_size_ = boost::thread::hardware_concurrency();
        }
        


        if (!cloud_target_frame_.empty())
        {
            tf2.reset(new tf2_ros::Buffer());
            tf2_listener.reset(new tf2_ros::TransformListener(*tf2));
            cloud_message_filter_.reset(new CloudMessageFilter(cloud_sub_, *tf2, cloud_target_frame_, input_queue_size_, nh));
            cloud_message_filter_->registerFailureCallback(boost::bind(&PointcloudProcessorNodelet::CloudFailureCallback,this,_1,_2));
        }
        
        
        
        else
        {
            tf2.reset(new tf2_ros::Buffer());
            tf2_listener.reset(new tf2_ros::TransformListener(*tf2));
            cloud_sub_.registerCallback(boost::bind(&PointcloudProcessorNodelet::CloudCallBack,this,_1));
        }
            


            
        pub = nh.advertise<sensor_msgs::PointCloud2>("clouder", 10, boost::bind(&PointcloudProcessorNodelet::connectCallback, this), boost::bind(&PointcloudProcessorNodelet::disconnectCallback, this));
    }









    void PointcloudProcessorNodelet::connectCallback()
    {
        
        boost::mutex::scoped_lock lock(connect_mutex);

        if (pub.getNumSubscribers() > 0 && cloud_sub_.getSubscriber().getNumPublishers()==0)
        {
            NODELET_INFO("Got a subscriber to scan, starting subscriber to pointcloud");
            cloud_sub_.subscribe(nh, "cloud_in", 2*input_queue_size_);
        }

    }









    void PointcloudProcessorNodelet::disconnectCallback()
    {
        boost::mutex::scoped_lock lock(connect_mutex);
        if (pub.getNumSubscribers() == 0)
        {
            NODELET_INFO("No subscibers to scan, shutting down subscriber to pointcloud");
            cloud_sub_.unsubscribe();
        }
    }
    





    void PointcloudProcessorNodelet::CloudFailureCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, tf2_ros::filter_failure_reasons::FilterFailureReason reason)
    {
    
    NODELET_WARN_STREAM_THROTTLE(1.0, "Can't transform pointcloud from frame " << cloud_msg->header.frame_id << " to "<< cloud_message_filter_->getTargetFramesString()<< " at time " << cloud_msg->header.stamp<< ", reason: " << reason);
    
    }


    
    








    void PointcloudProcessorNodelet::CloudCallBack(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {

    boost::shared_ptr <sensor_msgs::PointCloud2> cloud;
    ros::Time begin=ros::Time::now();
    Eigen::Affine3d transMat;







    try
    {



    geometry_msgs::TransformStamped trans= tf2->lookupTransform("odom",cloud_msg->header.frame_id,ros::Time::now()-ros::Duration(0.06));
    transMat=tf2::transformToEigen(trans);
    Eigen::MatrixXf trmt=((Eigen::MatrixXd)transMat.matrix()).cast<float>();
    trmt.resize(1,16);
    
    std::vector<float> HostVec;

    this->createHostVector(&HostVec,cloud_msg,&nh);
    

    std::vector<float> rand(HostVec.size(),0);


    thrust::device_vector<float> DeviceVec(HostVec.begin(),HostVec.end());
    thrust::device_vector<float> DeviceMat(trmt.data(),trmt.data()+trmt.size());
    thrust::device_vector<float> result(rand.begin(),rand.end());
    int x=HostVec.size()/4;

    float alpha = 1.0f;
    float beta = 0.0f;
    cudaDeviceSynchronize();
    cublasSgemm(this->h,CUBLAS_OP_N, CUBLAS_OP_N, 4,x,4 , &alpha, thrust::raw_pointer_cast(DeviceMat.data()), 4, thrust::raw_pointer_cast(DeviceVec.data()), 4, &beta, thrust::raw_pointer_cast(result.data()), 4);
    cudaDeviceSynchronize();
    std::vector<float> resVec(result.size());
    thrust::copy(result.begin(),result.end(),resVec.begin());

    sensor_msgs::PointCloud2 cloud_out;
    

    cloud_out.header=cloud_msg->header;
    cloud_out.fields.push_back(cloud_msg->fields[0]);
    cloud_out.fields.push_back(cloud_msg->fields[1]);
    cloud_out.fields.push_back(cloud_msg->fields[2]);
    cloud_out.is_bigendian=cloud_msg->is_bigendian;
    cloud_out.is_dense=true;
    cloud_out.point_step=12;
    cloud_out.height=1;
    cloud_out.width=resVec.size()/4;
    cloud_out.row_step=cloud_out.width*12;
    cloud_out.data.reserve(resVec.size()*3);


    std::vector<float>::iterator itr=resVec.begin();
    for(sensor_msgs::PointCloud2Iterator <float> iter_x(cloud_out, "x"), iter_y(cloud_out, "y"),iter_z(cloud_out, "z");iter_x != iter_x.end();++iter_x, ++iter_y, ++iter_z){

        if (itr>=resVec.end())
        {
            *iter_x=0;
            *iter_y=0;
            *iter_z=0;
            continue;
        }
            *iter_x=*itr;
        itr++;
        *iter_y=*itr;
        itr++;
        *iter_z=*itr;
        itr++;
        itr++;
        
    }
    cloud_out.data.resize(resVec.size()*3);
    std::cout<<"resVec size: "<<resVec.size()<<" data size: "<<cloud_out.data.size()<<" HostVec size: "<<HostVec.size()<<std::endl;
    cloud_out.header.frame_id="odom";
    pub.publish(cloud_out);
    }
    catch (tf2::TransformException& ex)
    {
      NODELET_ERROR_STREAM("Transform failure: " << ex.what());
      return;
    }

    std::cout<<"header:: "<<std::endl;


    std::cout<<ros::Time::now()-begin<<std::endl;

    }

    void PointcloudProcessorNodelet::createHostVector(std::vector<float>* HostVec,const sensor_msgs::PointCloud2ConstPtr& cloud_msg,ros::NodeHandle *nh){
        for (sensor_msgs::PointCloud2ConstIterator<float> iter_x_in(*cloud_msg, "x"), iter_y_in(*cloud_msg, "y"),iter_z_in(*cloud_msg, "z");iter_x_in != iter_x_in.end();++iter_x_in, ++iter_y_in, ++iter_z_in){
                if(!nh->ok()){
                    break;
                }

                if (std::isnan(*iter_x_in) || std::isnan(*iter_y_in) || std::isnan(*iter_z_in))
                {
                continue;
                }

                HostVec->push_back(*iter_x_in);
                HostVec->push_back(*iter_y_in);
                HostVec->push_back(*iter_z_in);
                HostVec->push_back(1);
                // HostVec->push_back(*iter_rgb_in);

            }
    }

}


PLUGINLIB_EXPORT_CLASS(Pointcloud_Nodelet_learn::PointcloudProcessorNodelet, nodelet::Nodelet)