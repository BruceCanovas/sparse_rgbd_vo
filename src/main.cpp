#include <ros/ros.h>
#include <sparse_rgbd_vo/sparse_rgbd_vo_node.hpp>

using namespace sparse_rgbd_vo;



int main(int argc, char** argv)
{
  ros::init(argc, argv, "sparse_rgbd_vo_node");

  SparseRGBDVONode node;

  node.run();
  
  return 0;
}
