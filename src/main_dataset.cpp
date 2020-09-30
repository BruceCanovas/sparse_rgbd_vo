#include <ros/ros.h>
#include <sparse_rgbd_vo/sparse_rgbd_vo_dataset_node.hpp>

using namespace sparse_rgbd_vo;



int main(int argc, char** argv)
{
  ros::init(argc, argv, "sparse_rgbd_vo_dataset_node");

  SparseRGBDVODatasetNode node;

  dynamic_reconfigure::Server<sparse_rgbd_vo::SparseRGBDVODatasetConfig> srv;
  dynamic_reconfigure::Server<sparse_rgbd_vo::SparseRGBDVODatasetConfig>::CallbackType cb;
  cb = boost::bind(&SparseRGBDVODatasetNode::configCallback, &node, _1, _2);
  srv.setCallback(cb);

  node.run();

  return 0;
}
