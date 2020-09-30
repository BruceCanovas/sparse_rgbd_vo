#pragma once

#include "ORBextractor.h"
#include <sparse_rgbd_vo/local_map.hpp>
#include <Eigen/StdVector>
#include <sparse_rgbd_vo/pnp_solver.hpp>



namespace sparse_rgbd_vo
{

class SparseRGBDVO
{

public:
    SparseRGBDVO();
    SparseRGBDVO(int nb_features,
                 float scale_factor,
                 int nb_levels,
                 int ini_th_fast,
                 int min_th_fast,
                 const CamParam& cam_param,
                 float range_min,
                 float range_max,
                 int untracked_threshold);
    ~SparseRGBDVO();
    void setFrame(const cv::Mat& im_rgb, const cv::Mat& im_depth);
    void extractFeatures();
    void matchFeatures();
    bool track();
    void updateLocalMap();
    void computeFilteredKeypoints3D(std::vector<cv::KeyPoint>& filtered_keypoints,
                                    cv::Mat& filtered_descriptors,
                                    std::vector<cv::Point3f>& points3D,
                                    float range_min,
                                    float range_max);
    void reset(const Eigen::Isometry3f& reset_pose);

    inline const std::vector<cv::KeyPoint>& getKeypoints() const {return keypoints;}
    inline const cv::Mat& getDescriptors() const {return descriptors;}
    inline std::vector<cv::KeyPoint>& setKeypoints() {return keypoints;}
    inline cv::Mat& setDescriptors() {return descriptors;}
    inline const Eigen::Isometry3f& getPose() const {return pose;}
    inline Eigen::Isometry3f& setPose() {return pose;}
    inline const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& getLocalMapPoints() const {return localMap->getPositions();}
    inline const cv::Mat& getVisMat() const {return vis;}

    void computeInlierFeature3DCorrespondences(std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& map_features3D,
                                               std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& frame_features3D,
                                               float range_min,
                                               float range_max);

private:
    cv::Mat bgr, depth, gray;
    ORB_SLAM2::ORBextractor *orb;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> matchedMapPositions;
    std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>> matchedKeypointsPositions;
    std::vector<int> keypointsMatchesIdx;
    CamParam cam;
    float rangeMin, rangeMax;
    LocalMap* localMap;
    Eigen::Isometry3f pose;
    Eigen::Vector3f lastPosition, linearVelocity;
    Eigen::Quaternionf lastQuaternion, angularVelocity;
    PnPSolver solver;
    int prevNbMatch;
    cv::Mat vis;

}; // class SparseRGBDVO

} // sparse_rgbd_vo
