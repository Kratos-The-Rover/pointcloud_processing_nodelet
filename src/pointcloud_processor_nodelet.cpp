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

#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <std_msgs/Float32MultiArray.h>
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
            // cloud_message_filter_.reset(new CloudMessageFilter(cloud_sub_, *tf2, cloud_target_frame_, input_queue_size_, nh));
            // cloud_message_filter_->registerFailureCallback(boost::bind(&PointcloudProcessorNodelet::CloudFailureCallback,this,_1,_2));
        }
        
        
        
        else
        {
            tf2.reset(new tf2_ros::Buffer());
            tf2_listener.reset(new tf2_ros::TransformListener(*tf2));
            cloud_sub_.registerCallback(boost::bind(&PointcloudProcessorNodelet::CloudCallBack,this,_1));
            std::cout<<"am here"<<std::endl;
        }
            
        createMap();

        createMsg();
        counter=0;
        pub = nh.advertise<grid_map_msgs::GridMap>("clouder", 10, boost::bind(&PointcloudProcessorNodelet::connectCallback, this), boost::bind(&PointcloudProcessorNodelet::disconnectCallback, this));
        std::cout<<"done initializing"<<std::endl;
    }






    void PointcloudProcessorNodelet::createMsg(){
        std::cout<<"creating message"<<std::endl;
        msg_ptr.reset(new grid_map_msgs::GridMap);
        std::cout<<"step1 done"<<std::endl;


        msg_ptr->info.header.stamp.fromNSec(map_ptr->getTimestamp());
        msg_ptr->info.header.frame_id = map_ptr->getFrameId();
        msg_ptr->info.resolution = map_ptr->getResolution();
        msg_ptr->info.length_x = map_ptr->getLength().x();
        msg_ptr->info.length_y = map_ptr->getLength().y();
        msg_ptr->info.pose.position.x =map_ptr->getPosition().x();
        msg_ptr->info.pose.position.y = map_ptr->getPosition().y();
        msg_ptr->info.pose.position.z = 0.0;
        msg_ptr->info.pose.orientation.x = 0.0;
        msg_ptr->info.pose.orientation.y = 0.0;
        msg_ptr->info.pose.orientation.z = 0.0;
        msg_ptr->info.pose.orientation.w = 1.0;
        
        msg_ptr->data.clear();
        msg_ptr->layers={"elevation"};
        msg_ptr->basic_layers=map_ptr->getBasicLayers();

        std_msgs::Float32MultiArray dataArray;

        msg_ptr->outer_start_index=map_ptr->getStartIndex()(0);
        msg_ptr->inner_start_index=map_ptr->getStartIndex()(1);
        std::cout<<"main message created"<<std::endl;

        dataArray.layout.dim.reserve(2);
        dataArray.layout.dim.resize(2);

        dataArray.layout.dim[0].stride = 100000000;
        dataArray.layout.dim[0].size = 10000;
        dataArray.layout.dim[1].stride = 10000;
        dataArray.layout.dim[1].size = 10000;
        dataArray.layout.dim[0].label=grid_map::storageIndexNames[grid_map::StorageIndices::Column];
        dataArray.layout.dim[1].label=grid_map::storageIndexNames[grid_map::StorageIndices::Row];
        std::vector<float> vec(100000000,NAN);
        dataArray.data=vec;
        msg_ptr->data.push_back(dataArray);
        std::cout<<"message created"<<std::endl;
    }


    void PointcloudProcessorNodelet::connectCallback()
    {
        
        boost::mutex::scoped_lock lock(connect_mutex);

        if (pub.getNumSubscribers() > 0 && cloud_sub_.getSubscriber().getNumPublishers()==0)
        {
            NODELET_INFO("Got a subscriber to scan, starting subscriber to pointcloud");
            // beginner.now();
            cloud_sub_.subscribe(nh, "cloud_in", 2*input_queue_size_);
        }

    }



    void PointcloudProcessorNodelet::createMap(){
        map_ptr.reset(new grid_map::GridMap);
        map_ptr->add("elevation");
        map_ptr->add("count");
        map_ptr->setFrameId("odom");
        map_ptr->setGeometry(grid_map::Length(2000,2000),0.2);
        map_ptr->clearAll();

    }





    void PointcloudProcessorNodelet::disconnectCallback()
    {

        boost::mutex::scoped_lock lock(connect_mutex);
        if (pub.getNumSubscribers() == 0)
        {
            NODELET_INFO("No subscibers to scan, shutting down subscriber to pointcloud");
            counter=0;
            cloud_sub_.unsubscribe();
        }
    }
    





    // void PointcloudProcessorNodelet::CloudFailureCallback(const pcl::PointCloud <pcl::PointXYZRGB>::Ptr& cloud_msg, tf2_ros::filter_failure_reasons::FilterFailureReason reason)
    // {
    
    // NODELET_WARN_STREAM_THROTTLE(1.0, "Can't transform pointcloud from frame " << cloud_msg->header.frame_id << " to "<< cloud_message_filter_->getTargetFramesString()<< " at time " << cloud_msg->header.stamp<< ", reason: " << reason);
    
    // }


    
    








    void PointcloudProcessorNodelet::CloudCallBack(const pcl::PointCloud <pcl::PointXYZRGB>::ConstPtr& pclCloud)
    {
        beginner=beginner.now();


        // std::cout<<"begin callback"<<std::endl;
        
        // std::cout<<cloud_msg->points[0].x<<std::endl;
        

        int cloud_size=pclCloud->height*pclCloud->width;
        pcl::PointCloud <pcl::PointXYZRGB>::Ptr pointcloud_pointer=cleanCloud(pclCloud);
        transformCloud(pointcloud_pointer);
        


        ros::Time time = ros::Time::now();

        // map_ptr->setTimestamp(time.toNSec());

        

        // msg_ptr->data[0].





	// ros::Time convert=ros::Time::now();
    // grid_map_msgs::GridMapPtr msg_ptr(new grid_map_msgs::GridMap);
        // grid_map::GridMapRosConverter::toMessage(*map_ptr,{"elevation"},*msg_ptr);
	// std::cout<<"pointxloud_convert: "<<ros::Time::now()-begin<<"converting: "<<ros::Time::now()-convert<<" ";
        ros::Time bef_pub=ros::Time::now();
        // std::cout<<"processing: "<<bef_pub-begin<<" ";
        // std::cout<<msg_ptr->data[0].layout.data_offset<<std::endl;
        pub.publish(msg_ptr);
        // std::cout<<"publisher count: "<<counter<<std::endl;
        // if(!(counter%100)){
        //     std::cout<<"publisher counter: "<<counter<<", frequency: "<<(float)counter/((ros::Time::now()-beginner).toSec())<<", time: "<<ros::Time::now()-beginner<<std::endl;
        // }
        // std::cout<<"processing"<<time-begin<<"publishing: "<<ros::Time::now()-bef_pub<<" net: "<<ros::Time::now()-begin<<std::endl;
        // pub.publish(*x);
        std::cout<<"publish freq: "<<(1/(ros::Time::now()-beginner).toSec())<<std::endl;
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
        counter++;
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

            
            // std::cout<<map_ptr->get("elevation").IsRowMajor<<std::endl;

            // for(int i=0;i<resVec.size();i+=4){
            //     if(!nh.ok() ){
            //         // ros::Duration(5).sleep();
            //         break;
            //     }
            //     grid_map::Position coordinate(resVec[i],resVec[i+1]);
            //     if(isnan(map_ptr->atPosition("elevation",coordinate))){
            //         map_ptr->atPosition("elevation",coordinate)=resVec[i+2];
            //         map_ptr->atPosition("count",coordinate)=1;

            //     }else {
            //         float prev=map_ptr->atPosition("elevation",coordinate);
            //         map_ptr->atPosition("elevation",coordinate)*=map_ptr->atPosition("count",coordinate);
            //         map_ptr->atPosition("elevation",coordinate)+=resVec[i+2];
            //         map_ptr->atPosition("elevation",coordinate)/=(++(map_ptr->atPosition("count",coordinate)));//(map_ptr->atPosition("elevation",coordinate)+resVec[i+2])/counter;
            //         // std::cout<<map_ptr->atPosition("elevation",coordinate)<<std::endl;
            //         // std::cout<<"count: " <<map_ptr->atPosition("count",coordinate)<<" prev: "<<prev<<" elev: "<< map_ptr->atPosition("elevation",coordinate)<<" resvec: "<<resVec[i+2]<<" diff: "<<map_ptr->atPosition("elevation",coordinate)-resVec[i+2]<<" "<<counter<<std::endl;

            //     }



                for(int i=0;i<resVec.size();i+=4){
                if(!nh.ok() ){
                    // ros::Duration(5).sleep();
                    break;
                }
                grid_map::Position coordinate(resVec[i],resVec[i+1]);
                grid_map::Index index;
                map_ptr->getIndex(coordinate,index);
   
                if(isnan(msg_ptr->data[0].data[10000*index[1]+index[0]])){
                    msg_ptr->data[0].data[10000*index[1]+index[0]]=resVec[i+20];
                    // map_ptr->atPosition("elevation",coordinate)=resVec[i+2];
                    map_ptr->atPosition("count",coordinate)=1;

                }
                else {
                    float prev=map_ptr->atPosition("elevation",coordinate);
                    msg_ptr->data[0].data[10000*index[1]+index[0]]*=map_ptr->atPosition("count",coordinate);
                    msg_ptr->data[0].data[10000*index[1]+index[0]]+=resVec[i+2];
                    
                    // std::cout<<msg_ptr->data[0].data[5000*index[0]+index[1]]<<std::endl;
                    msg_ptr->data[0].data[10000*index[1]+index[0]]/=(++(map_ptr->atPosition("count",coordinate)));//(map_ptr->atPosition("elevation",coordinate)+resVec[i+2])/counter;
                    // std::cout<<map_ptr->atPosition("elevation",coordinate)<<std::endl;
                    // std::cout<<"count: " <<map_ptr->atPosition("count",coordinate)<<" prev: "<<prev<<" elev: "<< map_ptr->atPosition("elevation",coordinate)<<" resvec: "<<resVec[i+2]<<" diff: "<<map_ptr->atPosition("elevation",coordinate)-resVec[i+2]<<" "<<counter<<std::endl;

                }




                //n-> avg(n)...... sum(n)/n.... -> avg(n)*n== sum(n)+obs(n+1)/n+1
                

                // pclCloud->points[m].x=resVec[i];
                // pclCloud->points[m].y=resVec[i+1];
                // pclCloud->points[m].z=resVec[i+2];
                m++;
            }
            // pclCloud->header.frame_id="odom";


            
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
