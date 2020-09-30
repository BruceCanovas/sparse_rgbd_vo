#include <sparse_rgbd_vo/sparse_rgbd_vo_node.hpp>
#include <iostream>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/rgbd.hpp>



namespace sparse_rgbd_vo
{

SparseRGBDVONode::SparseRGBDVONode()
    : privateNh("~"),
      it(nh),
      rgbSub(it, "/image_color", 1),
      depthSub(it, "/image_depth", 1),
      sync(RGBDSyncPolicy(10), rgbSub, depthSub),
      camInfoReceived(false),
      stamp(0)
{
    privateNh.param("map_frame_id", mapFrameId, std::string("map"));
    pathPub = nh.advertise<nav_msgs::Path>("trajectory", 1);

    featuresImPub = it.advertise("features", 1);
    colorImPub = it.advertise("color", 1);
    depthImPub = it.advertise("depth", 1);

    camInfoSub = nh.subscribe("/camera_info", 1, &SparseRGBDVONode::camInfoCallback, this);

    odomPub = nh.advertise<nav_msgs::Odometry>("vo", 1);

    sync.registerCallback(boost::bind(&SparseRGBDVONode::RGBDCallback, this, _1, _2));

    localMapPub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("local_map", 1);
    frameCloudPub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("frame_cloud", 1);
}

SparseRGBDVONode::~SparseRGBDVONode()
{
    delete vo;
}

void SparseRGBDVONode::RGBDCallback(const sensor_msgs::ImageConstPtr& msg_rgb, const sensor_msgs::ImageConstPtr& msg_depth)
{
    if(camInfoReceived)
    {
        cv_bridge::CvImageConstPtr cv_ptr_rgb = cv_bridge::toCvShare(msg_rgb);
        cv_bridge::CvImageConstPtr cv_ptr_depth = cv_bridge::toCvShare(msg_depth);

        cv::Mat rgb = cv_ptr_rgb->image.clone();
        cv::Mat bgr;
        cv::cvtColor(rgb, bgr, CV_RGB2BGR);
        cv::Mat depth_16U = cv_ptr_depth->image.clone();
        cv::Mat depth;
        depth_16U.convertTo(depth, CV_32FC1, depthScale);

        vo->setFrame(bgr, depth);
        vo->extractFeatures();
        vo->matchFeatures();
        bool vo_success = vo->track();

        if(!vo_success)
            std::cout<<"Tracking failed! Pose predicted using constant velocity motion model instead..."<<std::endl;

        Eigen::Isometry3f pose = vo->getPose();
        tf::Transform opt_to_map(tf::Matrix3x3(pose.matrix()(0,0), pose.matrix()(0,1), pose.matrix()(0,2),
                                               pose.matrix()(1,0), pose.matrix()(1,1), pose.matrix()(1,2),
                                               pose.matrix()(2,0), pose.matrix()(2,1), pose.matrix()(2,2)),
                                 tf::Vector3(pose.matrix()(0,3), pose.matrix()(1,3), pose.matrix()(2,3)));

        tf::StampedTransform stamped_opt_to_map(opt_to_map, ros::Time::now(), mapFrameId, msg_rgb->header.frame_id);
        br.sendTransform(stamped_opt_to_map);

        path.header = msg_rgb->header;
        path.header.frame_id = mapFrameId;
        geometry_msgs::PoseStamped pose_stamped_msg;
        tf::poseStampedTFToMsg(tf::Stamped<tf::Transform>(opt_to_map, ros::Time::now(), mapFrameId), pose_stamped_msg);
        path.poses.push_back(pose_stamped_msg);

        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = ros::Time::now();
        odom_msg.header.frame_id = mapFrameId;
        odom_msg.child_frame_id = msg_rgb->header.frame_id;
        tf::poseTFToMsg (opt_to_map, odom_msg.pose.pose);

        if(stamp > 0)
        {
            tf::Transform delta_base = prevOptToMap.inverse() * opt_to_map;
            double delta_t = (msg_rgb->header.stamp - prevTime).toSec();

            odom_msg.twist.twist.linear.x = delta_base.getOrigin().getX() / delta_t;
            odom_msg.twist.twist.linear.y = delta_base.getOrigin().getY() / delta_t;
            odom_msg.twist.twist.linear.z = delta_base.getOrigin().getZ() / delta_t;
            tf::Quaternion delta_rot = delta_base.getRotation();
            tfScalar angle = delta_rot.getAngle();
            tf::Vector3 axis = delta_rot.getAxis();
            tf::Vector3 angular_twist = axis * angle / delta_t;
            odom_msg.twist.twist.angular.x = angular_twist.x();
            odom_msg.twist.twist.angular.y = angular_twist.y();
            odom_msg.twist.twist.angular.z = angular_twist.z();
        }

        //odometry_msg.pose.covariance = ;
        //odometry_msg.twist.covariance = ;

        prevTime = msg_rgb->header.stamp;
        prevOptToMap = opt_to_map;

        if(odomPub.getNumSubscribers() > 0)
            odomPub.publish(odom_msg);

        if(pathPub.getNumSubscribers() > 0)
            pathPub.publish(path);

        if(colorImPub.getNumSubscribers() > 0)
        {
            colorImMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bgr).toImageMsg();
            colorImPub.publish(colorImMsg);
        }

        if(depthImPub.getNumSubscribers() > 0)
        {
            depthImMsg = cv_bridge::CvImage(std_msgs::Header(), "mono16", depth_16U).toImageMsg();
            depthImPub.publish(depthImMsg);
        }

        if(featuresImPub.getNumSubscribers() > 0)
        {
            featuresImMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", vo->getVisMat()).toImageMsg();
            featuresImPub.publish(featuresImMsg);
        }

        if(localMapPub.getNumSubscribers() > 0)
            publishLocalMapPoints(msg_rgb->header);

        if(frameCloudPub.getNumSubscribers() > 0)
            publishFrameCloud(msg_rgb->header, bgr, depth);

        vo->updateLocalMap();

        stamp++;
    }
}

void SparseRGBDVONode::camInfoCallback(const sensor_msgs::CameraInfo& msg_cam_info)
{
    cam.fx = float(msg_cam_info.K[0]);
    cam.cx = float(msg_cam_info.K[2]);
    cam.fy = float(msg_cam_info.K[4]);
    cam.cy = float(msg_cam_info.K[5]);
    cam.height = msg_cam_info.height;
    cam.width = msg_cam_info.width;

    float range_min, range_max;
    int nb_features, features_nb_levels, ini_th_fast, min_th_fast;
    float features_scale_factor;
    int untracked_threshold;

    privateNh.param("range_min", range_min, 0.2f);
    privateNh.param("range_max", range_max, 5.0f);
    privateNh.param("nb_features", nb_features, 1000);
    privateNh.param("features_scale_factor", features_scale_factor, 1.2f);
    privateNh.param("features_nb_levels", features_nb_levels, 8);
    privateNh.param("ini_th_fast", ini_th_fast, 20);
    privateNh.param("min_th_fast", min_th_fast, 5);
    privateNh.param("untracked_threshold", untracked_threshold, 10);
    privateNh.param("depth_scale", depthScale, 0.001f);

    vo = new SparseRGBDVO(nb_features,
                          features_scale_factor,
                          features_nb_levels,
                          ini_th_fast,
                          min_th_fast,
                          cam,
                          range_min,
                          range_max,
                          untracked_threshold);

    camInfoSub.shutdown();

    camInfoReceived = true;

    std::cout<<"Camera info received"<<std::endl;
}

void SparseRGBDVONode::publishLocalMapPoints(const std_msgs::Header& header)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr local_map_cloud(new  pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZRGB point_rgb;

    for(size_t i = 0; i < vo->getLocalMapPoints().size(); i++)
    {
        point_rgb.r = 255;
        point_rgb.g = 255;
        point_rgb.b = 255;

        point_rgb.x = vo->getLocalMapPoints()[i](0);
        point_rgb.y = vo->getLocalMapPoints()[i](1);
        point_rgb.z = vo->getLocalMapPoints()[i](2);

        local_map_cloud->points.push_back(point_rgb);
    }

    local_map_cloud->header.frame_id =  mapFrameId;
    pcl_conversions::toPCL(header.stamp, local_map_cloud->header.stamp);
    localMapPub.publish(local_map_cloud);
}

void SparseRGBDVONode::publishFrameCloud(const std_msgs::Header& header, const cv::Mat& bgr, const cv::Mat& depth)
{
    cv::Mat pts3D;
    cv::Mat K = cv::Mat::zeros(3, 3, CV_32F);
    K.at<float>(0,0) = cam.fx;
    K.at<float>(0,2) = cam.cx;
    K.at<float>(1,1) = cam.fy;
    K.at<float>(1,2) = cam.cy;
    K.at<float>(2,2) = 1.0f;

    cv::rgbd::depthTo3d(depth, K, pts3D);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr frame_cloud(new  pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZRGB point_color;

    cv::Vec3f point;
    cv::Vec3b color;

    for(int r = 0; r < pts3D.rows; r++)
    {
        for(int c = 0; c < pts3D.cols; c++)
        {
            point = pts3D.at<cv::Vec3f>(r, c);
            color = bgr.at<cv::Vec3b>(r, c);

            point_color.r = color(2);
            point_color.g = color(1);
            point_color.b = color(0);

            point_color.x = point(0);
            point_color.y = point(1);
            point_color.z = point(2);

            frame_cloud->points.push_back(point_color);
        }
    }

    frame_cloud->header.frame_id =  header.frame_id;
    pcl_conversions::toPCL(header.stamp, frame_cloud->header.stamp);
    frameCloudPub.publish(frame_cloud);
}

void SparseRGBDVONode::run()
{
    ros::spin();
}

} // namespace sparse_rgbd_vo
