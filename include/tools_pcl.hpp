/*
Copyright © 2023 InterDigital
All Rights Reserved
This program contains proprietary information which is a trade secret/business secret of InterDigital R&D france is protected, even if unpublished, under applicable Copyright laws (including French droit d’auteur) and/or may be subject to one or more patent(s).
*/

#ifndef tools_pcl_hpp
#define tools_pcl_hpp

#include <pcl/point_types.h>                //PointXYZRGB
#include <pcl/visualization/cloud_viewer.h> //PointCloud, Eigen

#include "nodes.hpp" //vector, nodes

pcl::PointCloud<pcl::PointXYZRGB>::Ptr snodesToCloud(
  const std::vector<SNode>& snodes, const unsigned char colors[]);

std::vector<SNode> simplifyNodes(const std::vector<SNode>& snodes, char type);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterModel(
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,
  const float leafSize=0.25f, const int limits=100);

void filterModelinPlace(
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,
  const float leafSize=0.25f, const int limits=100);

void modelProperties(pcl::PointCloud<pcl::PointXYZRGB>::Ptr model,
  const Eigen::Matrix4f& align,
  Eigen::Vector3f& dimensions, Eigen::Vector4f& center);

#endif /* tools_pcl_hpp */
