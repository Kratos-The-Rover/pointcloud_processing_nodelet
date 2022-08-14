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



#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
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
        private_nh.param<int>("concurrency_level", concurrency_level, 0);
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
            // cloud_message_filter_.reset(new CloudMessageFilter(cloud_sub_, *tf2, cloud_target_frame_, input_queue_size_, nh));
            // cloud_message_filter_->registerFailureCallback(boost::bind(&PointcloudProcessorNodelet::CloudFailureCallback,this,_1,_2));
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
    





    // void PointcloudProcessorNodelet::CloudFailureCallback(const pcl::PointCloud <pcl::PointXYZRGB>::Ptr& cloud_msg, tf2_ros::filter_failure_reasons::FilterFailureReason reason)
    // {
    
    // NODELET_WARN_STREAM_THROTTLE(1.0, "Can't transform pointcloud from frame " << cloud_msg->header.frame_id << " to "<< cloud_message_filter_->getTargetFramesString()<< " at time " << cloud_msg->header.stamp<< ", reason: " << reason);
    
    // }


    
    








    void PointcloudProcessorNodelet::CloudCallBack(const pcl::PointCloud <pcl::PointXYZRGB>::ConstPtr& pclCloud)
    {

        ros::Time begin=ros::Time::now();
        // std::cout<<cloud_msg->points[0].x<<std::endl;


        int cloud_size=pclCloud->height*pclCloud->width;
        pcl::PointCloud <pcl::PointXYZRGB>::Ptr x=cleanCloud(pclCloud);
        transformCloud(x);
        pub.publish(*x);
        

        std::cout<<ros::Time::now()-begin<<std::endl;


    }

  



























    pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointcloudProcessorNodelet::cleanCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& pclCloud){
        pcl::PassThrough<pcl::PointXYZRGB> pass_through;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr x(new pcl::PointCloud<pcl::PointXYZRGB>);
        pass_through.setInputCloud(pclCloud);
        pass_through.setFilterFieldName("z");
        float min=0.0;
        float max=10.0;
        pass_through.getFilterLimits(min,max);
        pass_through.filter(*x);
        return x;

    }



    void PointcloudProcessorNodelet::transformCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclCloud){
        Eigen::MatrixXf trmt;

        try
        {
            geometry_msgs::TransformStamped trans= tf2->lookupTransform("odom",pclCloud->header.frame_id,ros::Time::now()-ros::Duration(0.06));
            Eigen::MatrixXf trmt=((Eigen::MatrixXd)tf2::transformToEigen(trans).matrix()).cast<float>();
            trmt.resize(1,16);
            
            std::vector<float> HostVec;
            thrust::device_vector<float> DeviceMat(trmt.data(),trmt.data()+trmt.size());

            for(int i=0;i<pclCloud->points.size();i++){
            if(! nh.ok()){
                break;
            }
            HostVec.push_back(pclCloud->points[i].x);
            HostVec.push_back(pclCloud->points[i].y);
            HostVec.push_back(pclCloud->points[i].z);
            HostVec.push_back(1);
            }

            thrust::device_vector<float> DeviceVec(HostVec.begin(),HostVec.end());

            std::vector<float> rand(HostVec.size(),0);


            thrust::device_vector<float> result(rand.begin(),rand.end());
            int x=HostVec.size()/4;

            float alpha = 1.0f;
            float beta = 0.0f;
            cudaDeviceSynchronize();
            cublasSgemm(this->h,CUBLAS_OP_N, CUBLAS_OP_N, 4,x,4 , &alpha, thrust::raw_pointer_cast(DeviceMat.data()), 4, thrust::raw_pointer_cast(DeviceVec.data()), 4, &beta, thrust::raw_pointer_cast(result.data()), 4);
            cudaDeviceSynchronize();
            std::vector<float> resVec(result.size());
            thrust::copy(result.begin(),result.end(),resVec.begin());
            int m=0;
            for(int i=0;i<resVec.size();i+=4){
                pclCloud->points[m].x=resVec[i];
                pclCloud->points[m].y=resVec[i+1];
                pclCloud->points[m].z=resVec[i+2];
                m++;
            }
            pclCloud->header.frame_id="odom";


            
        }
        catch (tf2::TransformException& ex)
        {
            NODELET_ERROR_STREAM("Transform failure: " << ex.what());
            return;
        }
        
        
    }








































































































    void PointcloudProcessorNodelet::createHostVector(std::vector<float>* HostVec,const sensor_msgs::PointCloud2ConstPtr& cloud_msg,ros::NodeHandle *nh){
        int i=0;
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
                i++;

            }
    }

}


PLUGINLIB_EXPORT_CLASS(Pointcloud_Nodelet_learn::PointcloudProcessorNodelet, nodelet::Nodelet)