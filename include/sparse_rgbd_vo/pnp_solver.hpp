#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>



namespace g2o
{

class SparseOptimizer;

}

namespace sparse_rgbd_vo
{

class PnPSolver
{

  public:
    PnPSolver();
    ~PnPSolver();

    bool computePose(Eigen::Isometry3f& cam_pose,
                     const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& points3D,
                     const std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>& points2D,
                     float fx,
                     float fy,
                     float cx,
                     float cy);
    bool computePoseWithPrior(Eigen::Isometry3f& cam_pose,
                              Eigen::Isometry3f const& cam_pose_prior,
                              const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& points3D,
                              const std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>& points2D,
                              float fx,
                              float fy,
                              float cx,
                              float cy);

    inline const std::vector<int>& getInlierMarks() const {return inlierMarks;}

  private:
    g2o::SparseOptimizer *optimizer;

    std::vector<int> inlierMarks;

}; // class PnPSolver

} // sparse_rgbd_vo
