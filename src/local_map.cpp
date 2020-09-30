#include <sparse_rgbd_vo/local_map.hpp>
#include "gms_matcher.h"



namespace sparse_rgbd_vo
{

LocalMap::LocalMap()
    : untrackedThresh(10)
{
    matcher = cv::cuda::DescriptorMatcher::createBFMatcher(cv::NORM_HAMMING);
}

LocalMap::LocalMap(int untracked_thresh)
    : untrackedThresh(untracked_thresh)
{
    matcher = cv::cuda::DescriptorMatcher::createBFMatcher(cv::NORM_HAMMING);
}

void LocalMap::update(const Eigen::Isometry3f cam_to_map,
                      const CamParam& cam_param,
                      float range_min,
                      float range_max,
                      const cv::Mat& depth,
                      const std::vector<cv::KeyPoint>& frame_keypoints,
                      const cv::Mat& frame_descriptors,
                      const std::vector<int>& matches_idx)
{
    for(size_t i = 0; i < matches_idx.size(); i++)
    {
        float z = depth.at<float>(frame_keypoints[i].pt.y, frame_keypoints[i].pt.x);

        if(z > range_min && z < range_max)
        {
            Eigen::Vector3f p(z * (frame_keypoints[i].pt.x - cam_param.cx) / cam_param.fx,
                              z * (frame_keypoints[i].pt.y - cam_param.cy) / cam_param.fy,
                              z);
            p = cam_to_map * p;

            if(matches_idx[i] > 0) // replace map point with new match
            {
                positions[matches_idx[i]] = p;
                //positions[matches_idx[i]] = (p + positions[matches_idx[i]]) / 2.0f;
                frame_descriptors.row(i).copyTo(descriptors.row(matches_idx[i]));
                //counters[matches_idx[i]] = 0;
            }
            else // insert new point
            {
                positions.push_back(p);
                counters.push_back(0);
                descriptors.push_back(frame_descriptors.row(i).clone());
            }
        } 
    }
}

void LocalMap::reset(const Eigen::Isometry3f cam_to_map,
                     const CamParam& cam_param,
                     float range_min,
                     float range_max,
                     const cv::Mat& depth,
                     const std::vector<cv::KeyPoint>& frame_keypoints,
                     const cv::Mat& frame_descriptors)
{
    positions.clear();
    counters.clear();
    descriptors.release();

    for(size_t i = 0; i < frame_keypoints.size(); i++)
    {
        float z = depth.at<float>(frame_keypoints[i].pt.y, frame_keypoints[i].pt.x);

        if(z > range_min && z < range_max)
        {
            Eigen::Vector3f p(z * (frame_keypoints[i].pt.x - cam_param.cx) / cam_param.fx,
                              z * (frame_keypoints[i].pt.y - cam_param.cy) / cam_param.fy,
                              z);
            p = cam_to_map * p;

            positions.push_back(p);
            counters.push_back(0);
            descriptors.push_back(frame_descriptors.row(i).clone());
        }
    }
}

void LocalMap::clean()
{
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> cleaned_positions;
    std::vector<int> cleaned_counters;
    cv::Mat cleaned_descriptors;

    for(size_t i = 0; i < positions.size(); i++)
    {
        if(counters[i] < untrackedThresh)
        {
            cleaned_counters.push_back(counters[i]);
            cleaned_positions.push_back(positions[i]);
            cleaned_descriptors.push_back(descriptors.row(i).clone());
        }
    }

    std::swap(cleaned_positions, positions);
    std::swap(cleaned_counters, counters);
    descriptors = cleaned_descriptors.clone();
}

void LocalMap::insert(const Eigen::Vector3f& position, const cv::Mat& descriptor)
{
    positions.push_back(Eigen::Vector3f(position));
    counters.push_back(0);
    descriptors.push_back(descriptor.clone());
}

void LocalMap::replace(int id, const Eigen::Vector3f& position, const cv::Mat& descriptor)
{
    positions[id] = position;
    descriptor.copyTo(descriptors.row(id));
    counters[id] = 0;
}

void LocalMap::findMatches(const std::vector<cv::KeyPoint>& frame_keypoints,
                           const cv::Mat& frame_descriptors,
                           std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& matched_map_positions,
                           std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>& matched_frame_positions,
                           std::vector<int>& keypoints_matches_idx,
                           const Eigen::Isometry3f map_to_cam,
                           const CamParam& cam_param,
                           float range_min,
                           float range_max,
                           cv::Mat& vis)
{

    keypoints_matches_idx.resize(frame_keypoints.size(), -1);

    cv::Mat map_descriptors;
    std::vector<cv::KeyPoint> map_keypoints;
    std::vector<int> map_idx;

    for(size_t i = 0; i < positions.size(); i++)
    {
        Eigen::Vector3f p_view = map_to_cam * positions[i];
        float z = p_view(2);

        if(z > range_min && z < range_max)
        {
           cv::Point2f proj(cam_param.fx * p_view(0) / z + cam_param.cx,
                            cam_param.fy * p_view(1) / z + cam_param.cy);

            if(proj.x >= 0 && proj.x < cam_param.width && proj.y >= 0 && proj.y < cam_param.height) // && proj not in dynamic part
            {
                cv::KeyPoint kp;
                kp.pt = proj;
                map_keypoints.push_back(kp);
                map_descriptors.push_back(descriptors.row(i).clone());
                map_idx.push_back(i);

                cv::drawMarker(vis, cv::Point(int(proj.x), int(proj.y)), cv::Scalar(0,0,255), cv::MARKER_CROSS, 10/* marker_size*/, 1/* thickness*/, 8/* line_type*/);
            }
            else
                counters[i]++;
        }
        else
            counters[i]++;
    }

    std::vector<cv::DMatch> matches_bf;
    cv::cuda::GpuMat map_descriptors_d(map_descriptors), frame_descriptors_d(frame_descriptors);
    matcher->match(map_descriptors_d, frame_descriptors_d, matches_bf);

    std::vector<bool> inliers_states;
    cv::Size img_size(cam_param.width, cam_param.height);
    gms_matcher gms(map_keypoints, img_size, frame_keypoints, img_size, matches_bf);
    //int nb_inliers = gms.GetInlierMask(inliers_states, true, true);
    int nb_inliers = gms.GetInlierMask(inliers_states, false, false);

    for(size_t j = 0; j < inliers_states.size(); j++)
    {
        if(inliers_states[j])
        {
            matched_map_positions.push_back(positions[map_idx[matches_bf[j].queryIdx]]);
            cv::Point2f pt = frame_keypoints[matches_bf[j].trainIdx].pt;
            matched_frame_positions.push_back(Eigen::Vector2f(pt.x, pt.y));
            keypoints_matches_idx[matches_bf[j].trainIdx] = map_idx[matches_bf[j].queryIdx];

            cv::Point2f proj = map_keypoints[matches_bf[j].queryIdx].pt;
            cv::drawMarker(vis, cv::Point(int(proj.x), int(proj.y)), cv::Scalar(0,255,0), cv::MARKER_SQUARE, 10/* marker_size*/, 1/* thickness*/, 8/* line_type*/);
        }
        else
            counters[map_idx[matches_bf[j].queryIdx]]++;
    }

    //cv::imshow("Local map features", vis);
    //cv::waitKey(1);
}

} // sparse_rgbd_vo
