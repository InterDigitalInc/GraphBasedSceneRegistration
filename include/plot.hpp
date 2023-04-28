/*
Copyright © 2023 InterDigital
All Rights Reserved
This program contains proprietary information which is a trade secret/business secret of InterDigital R&D france is protected, even if unpublished, under applicable Copyright laws (including French droit d’auteur) and/or may be subject to one or more patent(s).
*/

#ifndef plot_hpp
#define plot_hpp

#include <opencv2/core.hpp>
#include <pcl/point_types.h>                //PointXYZRGB
#include <pcl/visualization/cloud_viewer.h> //PointCloud
#include <eigen3/Eigen/Dense>               //Eigen types

//P: projection/camera intrinsics matrix
//M: camera extrinsics matrix
//R: registration rotation vector
//T: registration translation vectior
//A: alignment matrix
#include "utils.hpp" //cloud types

void visualizeGG(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud1, Eigen::MatrixXi edges1,
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud2, Eigen::MatrixXi edges2,
  const int ctype=MEANC, Eigen::MatrixX2i matches=Eigen::MatrixX2i(),
  Eigen::Matrix4f A=Eigen::Matrix4f::Identity(),
  Eigen::Matrix3f R=Eigen::Matrix3f::Identity(),
  Eigen::Vector3f T=Eigen::Vector3f::Zero(), const char* name=nullptr);

void visualizeGM(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud1, Eigen::MatrixXi edges,
  const int ctype, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud2,
  Eigen::Matrix3f P, Eigen::Matrix4f M=Eigen::Matrix4f::Identity(),
  Eigen::Matrix4f A=Eigen::Matrix4f::Identity(),
  Eigen::Matrix3f R=Eigen::Matrix3f::Identity(),
  Eigen::Vector3f T=Eigen::Vector3f::Zero(), const char* name=nullptr);

void visualizeMM(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud1,
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud2,
  Eigen::Matrix4f A=Eigen::Matrix4f::Identity(),
  Eigen::Matrix3f R=Eigen::Matrix3f::Identity(),
  Eigen::Vector3f T=Eigen::Vector3f::Zero(), const char* name=nullptr);

void visualizeAM(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
  Eigen::Matrix3f P, cv::Mat pose, const char* name=nullptr);

void visualize2D(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
  Eigen::MatrixXi edges, const int ctype=MEANC,
  Eigen::Matrix4f A=Eigen::Matrix4f::Identity(), const char* name=nullptr);

#endif /* plot_hpp */
