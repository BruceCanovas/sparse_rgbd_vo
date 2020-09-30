#pragma once

#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <sparse_rgbd_vo/cam_param.hpp>
#include <opencv2/cudafeatures2d.hpp>
#include <Eigen/StdVector>



namespace sparse_rgbd_vo
{

class LocalMap
{

public:
    LocalMap();
    LocalMap(int untracked_thresh);
    void update(const Eigen::Isometry3f cam_to_map,
                const CamParam& cam_param,
                float range_min,
                float range_max,
                const cv::Mat& depth,
                const std::vector<cv::KeyPoint>& frame_keypoints,
                const cv::Mat& frame_descriptors,
                const std::vector<int>& matches_idx);
    void clean();
    void insert(const Eigen::Vector3f& position, const cv::Mat& descriptor);
    void replace(int id, const Eigen::Vector3f& position, const cv::Mat& descriptor);
    void findMatches(const std::vector<cv::KeyPoint>& frame_keypoints,
                     const cv::Mat& frame_descriptors,
                     std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& matched_map_positions,
                     std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>& matched_frame_positions,
                     std::vector<int>& keypoints_matches_idx,
                     const Eigen::Isometry3f map_to_cam,
                     const CamParam& cam_param,
                     float range_min,
                     float range_max,
                     cv::Mat& vis);
    void reset(const Eigen::Isometry3f cam_to_map,
               const CamParam& cam_param,
               float range_min,
               float range_max,
               const cv::Mat& depth,
               const std::vector<cv::KeyPoint>& frame_keypoints,
               const cv::Mat& frame_descriptors);

    inline const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& getPositions() const {return positions;}
    inline const cv::Mat& getDescriptors() const {return descriptors;}
    inline const std::vector<int>& getCounters() const {return counters;}
    inline std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& setPositions() {return positions;}
    inline cv::Mat& setDescriptors() {return descriptors;}
    inline std::vector<int>& setCounters() {return counters;}
    inline int size() const {return positions.size();}

private:
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> positions;
    cv::Mat descriptors;
    std::vector<int> counters;
    int untrackedThresh;
    cv::Ptr<cv::cuda::DescriptorMatcher> matcher;

}; // class LocalMap

} // sparse_rgbd_vo
