#include <sparse_rgbd_vo/sparse_rgbd_vo.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>



namespace sparse_rgbd_vo
{

SparseRGBDVO::SparseRGBDVO()
{
    orb = new ORB_SLAM2::ORBextractor(1000, 1.2f, 8, 20, 7);

    rangeMin = 0.2f;
    rangeMax = 5.0f;

    cam.fx = 0.0f;
    cam.fy = 0.0f;
    cam.cx = 0.0f;
    cam.cy = 0.0f;
    cam.width = -1;
    cam.height = -1;

    pose.setIdentity();

    prevNbMatch = 0;

    lastPosition.setZero();
    linearVelocity.setZero();
    lastQuaternion = Eigen::Quaternionf(1.0f, 0.0f, 0.0f, 0.0f);
    angularVelocity = Eigen::Quaternionf(1.0f, 0.0f, 0.0f, 0.0f);
}

SparseRGBDVO::SparseRGBDVO(int nb_features,
                           float scale_factor,
                           int nb_levels,
                           int ini_th_fast,
                           int min_th_fast,
                           const CamParam& cam_param,
                           float range_min,
                           float range_max,
                           int untracked_threshold)
{
    orb = new ORB_SLAM2::ORBextractor(nb_features, scale_factor, nb_levels, ini_th_fast, min_th_fast);

    cam = cam_param;

    rangeMin = range_min;
    rangeMax = range_max;

    localMap = new LocalMap(untracked_threshold);

    pose.setIdentity();

    prevNbMatch = 0;

    lastPosition.setZero();
    linearVelocity.setZero();
    lastQuaternion = Eigen::Quaternionf(1.0f, 0.0f, 0.0f, 0.0f);
    angularVelocity = Eigen::Quaternionf(1.0f, 0.0f, 0.0f, 0.0f);
}

SparseRGBDVO::~SparseRGBDVO()
{
    delete orb;
    delete localMap;
}

void SparseRGBDVO::extractFeatures()
{
    keypoints.clear();
    descriptors.release();

    orb->operator()(gray, cv::noArray(), keypoints, descriptors);
}

void  SparseRGBDVO::setFrame(const cv::Mat& im_bgr, const cv::Mat& im_depth)
{
    bgr = im_bgr.clone();
    depth = im_depth.clone();

    cv::cvtColor(bgr, gray, CV_BGR2GRAY);
    gray.convertTo(gray, CV_8UC1);
}

void SparseRGBDVO::matchFeatures()
{
    matchedMapPositions.clear();
    matchedKeypointsPositions.clear();
    keypointsMatchesIdx.clear();

    if(localMap->size() > 0)
    {
        vis = bgr.clone();

        localMap->findMatches(keypoints,
                              descriptors,
                              matchedMapPositions,
                              matchedKeypointsPositions,
                              keypointsMatchesIdx,
                              pose.inverse(),
                              cam,
                              rangeMin,
                              rangeMax,
                              vis);
    }
    else
        keypointsMatchesIdx.resize(keypoints.size(), -1);
}

bool SparseRGBDVO::track()
{
    bool valid = false;

    Eigen::Vector3f new_lin_velocity = pose.translation() - lastPosition;
    new_lin_velocity = (new_lin_velocity + linearVelocity) * 0.5f;

    Eigen::Quaternionf quat(pose.linear());
    Eigen::Quaternionf ang_vel_diff = quat * lastQuaternion.inverse();
    Eigen::Quaternionf new_ang_vel = ang_vel_diff.slerp(0.5f, angularVelocity);
    new_ang_vel.normalize();

    lastQuaternion = quat;
    angularVelocity = new_ang_vel;
    lastPosition = pose.translation();
    linearVelocity = new_lin_velocity;

    if(matchedMapPositions.size() >= 30)
    {
        valid = solver.computePose(pose,
                                   matchedMapPositions,
                                   matchedKeypointsPositions,
                                   cam.fx,
                                   cam.fy,
                                   cam.cx,
                                   cam.cy);
    }

    if(!valid)
    {
        pose.translation() += linearVelocity;
        Eigen::Quaternionf updated_quat = quat * new_ang_vel;
        updated_quat.normalize();
        pose.linear() = updated_quat.toRotationMatrix();
    }

    return valid;
}

void SparseRGBDVO::computeInlierFeature3DCorrespondences(std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& map_features3D,
                                                         std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& frame_features3D,
                                                         float range_min,
                                                         float range_max)
{
    std::vector<int> inlier_marks = solver.getInlierMarks();

    map_features3D.clear();
    frame_features3D.clear();

    for(size_t i = 0; i < inlier_marks.size(); i++)
    {
        if(inlier_marks[i])
        {
            float z = depth.at<float>(matchedKeypointsPositions[i](1), matchedKeypointsPositions[i](0));

            if(z > range_min && z < range_max)
            {
                Eigen::Vector3f frame_feature3D(z * (matchedKeypointsPositions[i](0) - cam.cx) / cam.fx,
                                                z * (matchedKeypointsPositions[i](1) - cam.cy) / cam.fy,
                                                z);

                map_features3D.push_back(frame_feature3D);
                frame_features3D.push_back(matchedMapPositions[i]);
            }
        }
    }
}

void SparseRGBDVO::updateLocalMap()
{
    localMap->update(pose,
                     cam,
                     rangeMin,
                     rangeMax,
                     depth,
                     keypoints,
                     descriptors,
                     keypointsMatchesIdx);
    localMap->clean();

    prevNbMatch = matchedMapPositions.size();
}

void SparseRGBDVO::computeFilteredKeypoints3D(std::vector<cv::KeyPoint>& filtered_keypoints,
                                              cv::Mat& filtered_descriptors,
                                              std::vector<cv::Point3f>& points3D,
                                              float range_min,
                                              float range_max)
{
    filtered_keypoints.clear();
    filtered_descriptors.release();
    points3D.clear();

    for(size_t i = 0; i < keypoints.size(); i++)
    {
        float z = depth.at<float>(keypoints[i].pt.y, keypoints[i].pt.x);

        if(z > range_min && z < range_max)
        {
            points3D.push_back(cv::Point3f(z * (keypoints[i].pt.x - cam.cx) / cam.fx,
                                           z * (keypoints[i].pt.y - cam.cy) / cam.fy,
                                           z));
            filtered_keypoints.push_back(keypoints[i]);
            filtered_descriptors.push_back(descriptors.row(i).clone());
        }
    }
}

void SparseRGBDVO::reset(const Eigen::Isometry3f& reset_pose)
{
    pose = reset_pose;
    localMap->reset(pose,
                    cam,
                    rangeMin,
                    rangeMax,
                    depth,
                    keypoints,
                    descriptors);
}

} // sparse_rgbd_vo
