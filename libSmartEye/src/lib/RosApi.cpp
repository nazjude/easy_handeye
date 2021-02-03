//
// Created by dzp on 2020/9/17.
//

#include <cmath>
#include <libSmartEye/RosApi.h>
#include <unistd.h>
//#include <pcl/io/ply_io.h>

namespace smart_eye {
  RosApi::RosApi(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
      nh_(nh), pnh_(pnh), init_(false), min_z_(1.), max_z_(2.)
  {
    initConnect();

    // Initialize get point cloud service
    std::string get_pointcloud_srv_id;
    pnh_.getParam("get_pointcloud_srv_id", get_pointcloud_srv_id);
    if (get_pointcloud_srv_id.empty()) {
      throw std::runtime_error("get_pointcloud_srv_id is None");
    }
    get_pointcloud_srv_ = nh_.advertiseService(get_pointcloud_srv_id, &RosApi::getPointCloudSrvCb, this);
    ROS_INFO_STREAM("Advertising service " << get_pointcloud_srv_id);

    // Initialize pointcloud publisher
    std::string pub_pointcloud_msg_id;
    pnh_.getParam("pub_pointcloud_msg_id", pub_pointcloud_msg_id);
    if (pub_pointcloud_msg_id.empty()) {
      throw std::runtime_error("pub_pointcloud_msg_id is None");
    }
    pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(pub_pointcloud_msg_id, 1);
    ROS_INFO_STREAM("Publish point cloud to " << pub_pointcloud_msg_id);

    // Initialize camera frame
    pnh_.getParam("camera_frame", camera_frame_);

    // Initialize image publisher
    std::string pub_image_msg_id;
    pnh_.getParam("pub_image_msg_id", pub_image_msg_id);
    if (pub_image_msg_id.empty()) {
      throw std::runtime_error("pub_image_msg_id is None");
    }
    image_pub_ = nh_.advertise<sensor_msgs::Image>(pub_image_msg_id, 1);
    ROS_INFO_STREAM("Publish camera image to " << pub_image_msg_id);

    // Initialize depth image publisher @@@added
    depth_image_pub_ = nh_.advertise<sensor_msgs::Image>("rect_camera/depth_image", 1);
    ROS_INFO_STREAM("Publish camera image to rect_camera/depth_image");

    // Initialize get_imageWithDepth service @@@added
    get_imagewithdepth_srv_ = nh_.advertiseService("get_imageWithDepth", &RosApi::getImageWithDepthSrvCb, this);
//    ROS_INFO_STREAM("Advertising service " << get_pointcloud_srv_id);

    // Initialize get space position service @@@added
    get_spaceposition_srv_ = nh_.advertiseService("get_spacePosition", &RosApi::getSpacePositionSrvCb, this);
//    ROS_INFO_STREAM("Advertising service " << get_pointcloud_srv_id);

    // Initialize camera_info publisher
    std::string pub_camera_info_msg_id;
    pnh_.getParam("pub_camera_info_msg_id", pub_camera_info_msg_id);
    if (pub_camera_info_msg_id.empty()) {
      throw std::runtime_error("pub_camera_info_msg_id is None");
    }
    camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(pub_camera_info_msg_id, 1);
    ROS_INFO_STREAM("Publish camera info to " << pub_camera_info_msg_id);

  }


  RosApi::~RosApi() {}

  bool RosApi::initConnect() {
    if (SE_STATUS_SUCCESS == controller_.initDevice()) {
      float min_z, max_z, minZ, maxZ;
      controller_.getZRange(minZ, maxZ);  // in millimeter
      ROS_INFO("Default depth range: %.3f, %.3f mm", minZ, maxZ);
      pnh_.getParam("min_depth", min_z);  // in meter
      pnh_.getParam("max_depth", max_z);  // in meter
      min_z_ = fmax(min_z, minZ * 0.001);
      max_z_ = fmin(max_z, maxZ * 0.001);
      ROS_INFO("Initialized depth range: %.3f, %.3f m", min_z_, max_z_);
      init_ = true;
      ROS_WARN("Initialize device succeed.");
    } else {
      init_ = false;
      ROS_ERROR("Initialize device failed.");
    }
  }

  bool RosApi::getPointCloudSrvCb(smarteye::GetPointCloud::Request& req,
                                  smarteye::GetPointCloud::Response& res)
  {
    if (!init_) {
      res.result_status = res.FAILED;
      return true;
    }

    // publish sensor/Image
    cv::Mat img, img_right;
    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg, img_msg_raw, img_msg_depth;
    std_msgs::Header header;

    controller_.captureUnrectifiedImages(img,img_right);
    int img_rows = img.rows;
    int img_cols = img.cols;
    ROS_WARN("SIZE OF IMAGE: %d,%d",img.rows,img.cols);
    header.stamp = ros::Time::now();
    header.frame_id = camera_frame_;
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, img);
    img_bridge.toImageMsg(img_msg);
    depth_image_pub_.publish(img_msg);



    // publish sensor/CameraInfo
    sensor_msgs::CameraInfo cam_info;

    cam_info.header.stamp = ros::Time::now();
    cam_info.header.frame_id = camera_frame_;

    cam_info.height = img_rows;
    cam_info.width = img_cols;

    std::string distortion_model = "plumb_bob";
    cam_info.distortion_model = distortion_model;


//  new calibration results in hkcenter with a score of 90
    std::vector<double> D={ -8.8032664233567162e-02, 2.0810498424442800e-01,
       1.4842302327704646e-03, 1.0714103870387453e-03,
       -5.0327757888961966e-01 };
    cam_info.D = D;

    boost::array<double, 9> K={ 2.4054420249380441e+03, 0., 1.2365285628068989e+03, 0.,
       2.4101340845563623e+03, 1.0842925599075431e+03, 0., 0., 1.};
    cam_info.K = K;

    boost::array<double, 9> R={9.9982063997472215e-01, -1.0779402463805640e-02,
       -1.5572166293118403e-02, 1.0688227229398762e-02,
       9.9992531686915753e-01, -5.9264224290425712e-03,
       1.5634886607518786e-02, 5.7589206139710631e-03,
       9.9986118294197823e-01 };
    cam_info.R = R;

    boost::array<double, 12> P={2.2684802243158815e+03, 0., 9.9820935058593750e+02, 0., 0.,
       2.2684802243158815e+03, 1.0959599685668945e+03, 0., 0., 0., 1.,
       0.};
    cam_info.P = P;


//    std::vector<double> D={ -8.6193209738583507e-02, 6.9727708614065770e-02,
//       -1.7174878471021811e-03, 3.1616522791387487e-04,
//       3.4390078480913444e-01 };
//    cam_info.D = D;
//
//    boost::array<double, 9> K={ 2.4012871090876170e+03, 0., 1.2302780275290656e+03, 0.,
//       2.3989354426852187e+03, 1.0448437859392129e+03, 0., 0., 1. };
//    cam_info.K = K;
//
//    boost::array<double, 9> R={9.9995208040717187e-01, -9.7433113844223214e-03,
//       -9.5119537165930246e-04, 9.7399667415055975e-03,
//       9.9994658018365368e-01, -3.4597437607775763e-03,
//       9.8485391974881265e-04, 3.4503133599806421e-03,
//       9.9999356267951778e-01 };
//    cam_info.R = R;
//
//    boost::array<double, 12> P={2.2823992780578592e+03, 0., 9.3747891998291016e+02, 0., 0.,
//       2.2823992780578592e+03, 1.0294147796630859e+03, 0., 0., 0., 1.,
//       -1. };
//    cam_info.P = P;

    camera_info_pub_.publish(cam_info);

    // capture 3d model
//    sleep(5);
    controller_.captureThreeModel();
    PointCloud_SE_Ptr pointCloud;
    controller_.getPointCloud(pointCloud);


    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (convert2PCLPointCloud(pointCloud, pcl_cloud)) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr f_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
      getCloudInZRange(pcl_cloud, inliers, f_cloud, min_z_, max_z_);

    // save point cloud as .ply
    //  pcl::io::savePLYFile("/home/hkclr/pointcloudsaver/mypointcloud.ply", *f_cloud);
    //  ROS_WARN("Save ply file succeed!!!!!!!!!!!!!!!!!!!");

      sensor_msgs::PointCloud2 msg;
      pcl::toROSMsg(*f_cloud, msg);

      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = camera_frame_;
      res.points = msg;
      res.result_status = res.SUCCEEDED;
      // Publish point cloud
      pointcloud_pub_.publish(msg);
    } else {
      res.result_status = res.FAILED;
    }
    return true;
  }

  bool RosApi::getImageWithDepthSrvCb(smarteye::GetPointCloud::Request& req,
                                      smarteye::GetPointCloud::Response& res)
  {
    if (!init_) {
      res.result_status = res.FAILED;
      return true;
    }

    // publish sensor/Image
    cv::Mat img, img_right;
    cv_bridge::CvImage img_bridge, img_bridge_raw, img_bridge_depth;
    sensor_msgs::Image img_msg, img_msg_raw, img_msg_depth;
    std_msgs::Header header;
    controller_.captureUnrectifiedImages(img,img_right);
    int img_rows = img.rows;
    int img_cols = img.cols;
    ROS_WARN("SIZE OF IMAGE: %d,%d",img.rows,img.cols);
    header.stamp = ros::Time::now();
    header.frame_id = camera_frame_;
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, img);
    img_bridge.toImageMsg(img_msg);
    image_pub_.publish(img_msg);

    // publish sensor/CameraInfo
    sensor_msgs::CameraInfo cam_info;

    cam_info.header.stamp = ros::Time::now();
    cam_info.header.frame_id = camera_frame_;

    cam_info.height = img_rows;
    cam_info.width = img_cols;

    std::string distortion_model = "plumb_bob";
    cam_info.distortion_model = distortion_model;
//  use the calibration parameters of the new camera (calibrated at 2021/1/20)
    std::vector<double> D={ -8.3485843721762959e-02, 1.5340175093273248e-01,
       -5.6065061163484249e-04, -1.8124479491787000e-04,
       1.6434518250341707e-03 };
    cam_info.D = D;

    boost::array<double, 9> K={ 2.4183900904408229e+03, 0., 1.2770094420481869e+03, 0.,
       2.4184181113678883e+03, 1.0192528130448799e+03, 0., 0., 1. };
    cam_info.K = K;

    boost::array<double, 9> R={ 9.9992845177925904e-01, -7.9182599749377941e-03,
       8.9661854377037359e-03, 7.7830134934936982e-03,
       9.9985690497240953e-01, 1.5019796268731298e-02,
       -9.0838330727775612e-03, -1.4948937686785561e-02,
       9.9984699491409312e-01 };
    cam_info.R = R;

    boost::array<double, 12> P={ 2.3305020729993912e+03, 0., 9.4052457427978516e+02, 0., 0.,
       2.3305020729993912e+03, 1.0460448837280273e+03, 0., 0., 0., 1.,
       0. };
    cam_info.P = P;

    camera_info_pub_.publish(cam_info);

    // capture 3d model
//    sleep(5);
    controller_.captureThreeModel();
    PointCloud_SE_Ptr pointCloud;
    controller_.getPointCloud(pointCloud);

    //////////////////////////////////////////////////////////
    // publish sensor/Depth Image
//    cv::Mat img_trans_depth = controller_.getDepthImage();
    cv::Mat img_trans_depth;
    controller_.captureOneImage(img_trans_depth, false);
    header.stamp = ros::Time::now();
    header.frame_id = camera_frame_;
    img_bridge_depth = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, img_trans_depth);
    img_bridge_depth.toImageMsg(img_msg_depth);
    depth_image_pub_.publish(img_msg_depth);
    // publish sensor/Raw Image
//    cv::Mat img_trans_left, img_trans_right;
//    controller_.getUnrectifiedImages(img_trans_left,img_trans_right);
//    header.stamp = ros::Time::now();
//    header.frame_id = camera_frame_;
//    img_bridge_raw = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, img_trans_left);
//    img_bridge_raw.toImageMsg(img_msg_raw);
//    depth_image_pub_.publish(img_msg_depth);


//
    // while (!CornerPointReceived)
    // {
    //     ros::Duration(0.5).sleep();
    // }
    // pos2d_tmp.push_back(cv::Point2i (405, 721));
    // cv::Point3f pos3d_tmp = controller_.map2DImagePointTo3DSpace(cv::Point2i (405, 721));
    // ROS_WARN("%f, %f, %f", pos3d_tmp.x, pos3d_tmp.y, pos3d_tmp.z );
    // CornerPointReceived = false;

    //calculate the space coordinate in 2D image
//    pos2d_tmp.push_back(cv::Point2i (352, 546));
//    cv::Point3f pos3d_tmp = controller_.map2DImagePointTo3DSpace(pos2d_tmp[0]);


    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (convert2PCLPointCloud(pointCloud, pcl_cloud)) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr f_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
      getCloudInZRange(pcl_cloud, inliers, f_cloud, min_z_, max_z_);

    // save point cloud as .ply
    //  pcl::io::savePLYFile("/home/hkclr/pointcloudsaver/mypointcloud.ply", *f_cloud);
    //  ROS_WARN("Save ply file succeed!!!!!!!!!!!!!!!!!!!");

      sensor_msgs::PointCloud2 msg;
      pcl::toROSMsg(*f_cloud, msg);

      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = camera_frame_;
      res.points = msg;
      res.result_status = res.SUCCEEDED;
      // Publish point cloud
      pointcloud_pub_.publish(msg);
    } else {
      res.result_status = res.FAILED;
    }
    return true;
  }

  bool RosApi::getSpacePositionSrvCb(smarteye::GetSpacePosition::Request& req,
                                  smarteye::GetSpacePosition::Response& res)
  {
    //calculate the space position in 2D image
    pos2d_tmp.clear();
    CornerPointReceived = true;
    for (int i = 0; i < req.point2d.size(); ++i)
    {
        pos2d_tmp.push_back(cv::Point2i ((int)req.point2d[0].x, (int)req.point2d[0].y));
        point_tmp = req.point2d[i];
        res.point3d.push_back(point_tmp);
    }
    return true;
  }

  bool RosApi::convert2PCLPointCloud(const PointCloud_SE_Ptr& se_cloud,
                                     const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud)
  {
    if (nullptr == se_cloud || nullptr == pcl_cloud) {
      return false;
    }

    pcl_cloud->points.clear();
    int numOfPoints = se_cloud->points.size();
    for (int i = 0; i < numOfPoints; i++) {
      pcl::PointXYZ point;
      point.x = se_cloud->points[i].x;
      point.y = se_cloud->points[i].y;
      point.z = se_cloud->points[i].z;
      pcl_cloud->points.push_back(point);
    }
    return true;
  }

  void RosApi::getCloudInZRange(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                                pcl::PointIndices::Ptr &inliers,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out,
                                float z_min, float z_max)
  {
    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_in);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_min, z_max);
    //pass.setFilterLimitsNegative(true);
    pass.filter(inliers->indices);
    pass.filter(*cloud_out);
  }
}