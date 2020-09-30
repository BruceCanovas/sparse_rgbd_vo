#pragma once

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sparse_rgbd_vo/sparse_rgbd_vo.hpp>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/cache.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Path.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>



namespace sparse_rgbd_vo
{

class SparseRGBDVONode
{

public:
    SparseRGBDVONode();
    ~SparseRGBDVONode();
    void RGBDCallback(const sensor_msgs::ImageConstPtr& msg_rgb, const sensor_msgs::ImageConstPtr& msg_depth);
    void camInfoCallback(const sensor_msgs::CameraInfo& info);
    void run();
    void publishLocalMapPoints(const std_msgs::Header& header);
    void publishFrameCloud(const std_msgs::Header& header, const cv::Mat& bgr, const cv::Mat& depth);


private:
    ros::NodeHandle nh, privateNh;

    SparseRGBDVO* vo;

    float depthScale;

    image_transport::ImageTransport it;
    image_transport::SubscriberFilter rgbSub, depthSub;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> RGBDSyncPolicy;
    message_filters::Synchronizer<RGBDSyncPolicy> sync;
    ros::Subscriber camInfoSub;

    tf::TransformBroadcaster br;
    nav_msgs::Path path;
    ros::Publisher pathPub;
    image_transport::Publisher featuresImPub, depthImPub, colorImPub;
    sensor_msgs::ImagePtr featuresImMsg, depthImMsg, colorImMsg;

    ros::Publisher odomPub;
    tf::Transform prevOptToMap;
    ros::Time prevTime;

    ros::Publisher localMapPub, frameCloudPub;
    bool camInfoReceived;

    std::string mapFrameId;

    CamParam cam;

    int stamp;
};

} // namespace sparse_rgbd_vo
