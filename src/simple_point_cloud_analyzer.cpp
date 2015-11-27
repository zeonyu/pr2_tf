/*****************************************************************************************
 *
 *  Copyright (c) 2014, Bharath Sankaran,
 *  Computational Learning and Motor Control Laboratory, University of Southern California
 *  (bsankara@usc.edu)
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
 *   * Neither the name of Bharath Sankaran nor CLMC
 *     may be used to endorse or promote products derived
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
 ******************************************************************************************/
/**
  * Details for all functions declared in the SimplePointCloudAnalyzer class
  */
#include "pr2_tf/simple_point_cloud_analyzer.hpp"
namespace point_cloud_analyzer{

SimplePointCloudAnalyzer::SimplePointCloudAnalyzer(ros::NodeHandle &nh):
    xyz_cld_ptr_(new pcl::PointCloud<pcl::PointXYZ>),original_cld_ptr(new pcl::PointCloud<pcl::PointXYZ>),
    //disp_cld_ptr_(new pcl::PointCloud<pcl::PointXYZRGB>),
    cloud_updated_(false), goal_completion_time_(ros::Time::now()),
    listener_(ros::Duration(180.0)),
    nh_(nh), private_nh_("~")
{
    srand((unsigned)time(NULL));
}

void SimplePointCloudAnalyzer::CloudCallBack(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud){

    // We receive continuous clouds from the kinect here but will not use them!
    if((!cloud_updated_) && (goal_completion_time_ + ros::Duration(6.0) < cloud->header.stamp))
    {
       try
        {
            // update the pose
            listener_.waitForTransform( opt_frm_, fx_frm_, cloud->header.stamp, ros::Duration(5.0));
            listener_.lookupTransform( fx_frm_, opt_frm_, cloud->header.stamp, optical2base_);

            // ASSUMES optical2map and sensor2map have the same translation!!!
            tf::Vector3 position( optical2base_.getOrigin() );
            position_.x() = position.x();
            position_.y() = position.y();
            position_.z() = position.z();

            // We dont need to look up kinect2map
            tf::Quaternion opt_quat_snsr(0.5, -0.5, 0.5, 0.5);
            tf::Quaternion orientation( optical2base_ * opt_quat_snsr );
            orientation_.x() = orientation.x();
            orientation_.y() = orientation.y();
            orientation_.z() = orientation.z();
            orientation_.w() = orientation.w();

            ROS_INFO_STREAM("position = " << position_.transpose() );
            ROS_INFO_STREAM("orientation = " << orientation_.transpose() );

            // update the cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr cld_tmp(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*cloud, *cld_tmp);

            // cut far away points
            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setFilterFieldName ("z"); // check if this cut is in the right direction before rotating the cloud!
            pass.setFilterLimits (0.1, 2.0);
            pass.setInputCloud(cld_tmp);
            pass.filter(*xyz_cld_ptr_);

            cloud_updated_ = true;
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s", ex.what());
        }
    }
}

void SimplePointCloudAnalyzer::SimplePointCloudAnalyzerInit(){

    std::string cld_topic_name;
    private_nh_.param( "cld_topic_name", cld_topic_name, std::string("/camera/depth/points") );
    private_nh_.param( "fx_frm", fx_frm_, std::string("/odom_combined") );
    //private_nh_.param( "snsr_frm", snsr_frm_, std::string("/XTION_RGB") );
    private_nh_.param( "opt_frm", opt_frm_, std::string("/camera_depth_optical_frame") ); //
    cloud_sub_ = nh_.subscribe( cld_topic_name, 1, &SimplePointCloudAnalyzer::CloudCallBack, this);

}

void SimplePointCloudAnalyzer::SimplePointCloudAnalyzerSpin(){

    // 0. Get pose and cloud
    // Wait until time stamps are bigger than goal completion time
    while( !cloud_updated_ )
        ros::spinOnce();

    // convert cloud to /BASE frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr base_cld_ptr( new pcl::PointCloud<pcl::PointXYZ>);
    pcl_ros::transformPointCloud( *xyz_cld_ptr_, *base_cld_ptr, optical2base_);

    base_cld_ptr->header.frame_id = fx_frm_;
    base_cld_ptr->header.stamp = xyz_cld_ptr_->header.stamp;

    //TODO: Dummy process state, Here we simply classify all Objects in the list

    // test by zeon: try to change pcd saving path but in vain
    //pcl::PCDWriter writer;
    //writer.writeASCII("/home/vv/groovy/pr2_tf/pr2_tf/data"+boost::lexical_cast<std::string>(goal_completion_time_.toSec())+"pcd_from_kinect.pcd", *xyz_cld_ptr_)

    std::string filename(boost::lexical_cast<std::string>(goal_completion_time_.toSec())+"pcd_from_kinect.pcd");
   // ros::Duration(3).sleep();
    pcl::io::savePCDFileASCII(filename, *base_cld_ptr);
 //   ros::Duration(3).sleep();
    update_Original_cloud(base_cld_ptr);
    goal_completion_time_ = ros::Time::now();
    cloud_updated_ = false;
}

void SimplePointCloudAnalyzer::update_Original_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cld_ptr){
            // remove NaNs
       //     std::vector<int> indices;
        //    pcl::removeNaNFromPointCloud(*cld_ptr, *cld_ptr, indices);

            if(original_cld_ptr->empty() )
            {

                original_cld_ptr = cld_ptr;
            }else
            {


                // Get an initial guess for the transform
                double fitness_score = 1000;
                Eigen::Matrix4f fTrans = Eigen::Matrix4f::Identity();

                pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;
                icp.setMaxCorrespondenceDistance(0.1);	// 0.1
                icp.setTransformationEpsilon(1e-7);	// transformation convergence epsilon
                //icp.setMaximumIterations(1000);
                //icp.setEuclideanFitnessEpsilon(1.0); // maximum allowed error between two consecutive steps
                icp.setInputCloud( cld_ptr );
                icp.setInputTarget( original_cld_ptr );

                pcl::PointCloud<pcl::PointXYZ>::Ptr final_cld (new pcl::PointCloud<pcl::PointXYZ>);

                icp.align(*final_cld);		// final cloud contains the transformed query cloud
                //*final_cld += *base_cld;		// add the two clouds

                if( icp.hasConverged() )
                {
                    fitness_score = icp.getFitnessScore();
                    ROS_INFO("[pcd_utils] ICP has converged with score: %f", fitness_score);
                    fTrans = icp.getFinalTransformation();
                }else{
                    ROS_WARN("[pcd_utils] ICP did not converge!");
                }

//                guess = fTrans;
//                Eigen::Matrix4f best_trans = fTrans;


//                pcl::PointCloud<pcl::PointXYZ>::Ptr best_ptr(new pcl::PointCloud<pcl::PointXYZ>);
//                pcl::transformPointCloud ( *cld_ptr, *best_ptr, best_trans );

                *original_cld_ptr=*final_cld+*original_cld_ptr;
                std::string filename("register_"+boost::lexical_cast<std::string>(goal_completion_time_.toSec())+".pcd");
                pcl::io::savePCDFileASCII(filename, *original_cld_ptr);
            }

}

} // end of namespace: point_cloud_analyzer


int SimplePointCloudAnalyzerMain(int argc, char **argv)
{
    ros::init(argc, argv,"simple_point_cloud_analyzer");
    ros::NodeHandle nh;
    point_cloud_analyzer::SimplePointCloudAnalyzer pc(nh);
    pc.SimplePointCloudAnalyzerInit();

    while (ros::ok())
    {
        pc.SimplePointCloudAnalyzerSpin();
        ros::spinOnce();
    }
    return 0;
}

int main(int argc, char **argv)
{

    return SimplePointCloudAnalyzerMain(argc, argv);
}

