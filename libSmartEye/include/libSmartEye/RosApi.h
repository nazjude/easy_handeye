//
// Created by dzp on 2020/9/17.
//

#ifndef SMART_EYE_ROS_API_H
#define SMART_EYE_ROS_API_H

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

// PCL
#include <pcl/filters/passthrough.h>


// SmartEye
#include "SmartEyeAPI/ProcessController.h"

// Services
#include <smarteye/GetPointCloud.h>
#include <smarteye/GetSpacePosition.h>

//OpenCv
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Header.h>

namespace smart_eye {

  class RosApi {
  public:
    RosApi(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    ~RosApi();

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ProcessController controller_;
    bool init_;

    float min_z_;
    float max_z_;

    std::string camera_frame_;
    ros::Publisher pointcloud_pub_;
    ros::ServiceServer get_pointcloud_srv_;
    ros::ServiceServer get_spaceposition_srv_;
    ros::ServiceServer get_imagewithdepth_srv_;
    ros::Publisher image_pub_;
    ros::Publisher depth_image_pub_;
    ros::Publisher camera_info_pub_;

    std::vector<cv::Point2i> pos2d_tmp;
    geometry_msgs::Point point_tmp;

    bool getPointCloudSrvCb(smarteye::GetPointCloud::Request& req,
                            smarteye::GetPointCloud::Response& res);

    bool getImageWithDepthSrvCb(smarteye::GetPointCloud::Request& req,
                                smarteye::GetPointCloud::Response& res);

    bool getSpacePositionSrvCb(smarteye::GetSpacePosition::Request& req,
                               smarteye::GetSpacePosition::Response& res);

    bool initConnect();

    bool convert2PCLPointCloud(const PointCloud_SE_Ptr& se_cloud,
                               const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud);

    void getCloudInZRange(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                          pcl::PointIndices::Ptr &inliers,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out,
                          float z_min, float z_max);

    bool CornerPointReceived = false;

  };

}

#endif //SMART_EYE_ROS_API_H