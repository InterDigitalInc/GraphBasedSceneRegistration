/*
Copyright © 2023 InterDigital
All Rights Reserved
This program contains proprietary information which is a trade secret/business secret of InterDigital R&D france is protected, even if unpublished, under applicable Copyright laws (including French droit d’auteur) and/or may be subject to one or more patent(s).
*/

#ifndef vo_hpp
#define vo_hpp

#include <opencv2/core.hpp>   //Mat
#include <eigen3/Eigen/Dense>
  //#include <eigen3/Eigen/SVD>

#include "pointCloudExtraction.hpp"  //pointCloudExtraction, loadImage, nodes

void readPoses(const char* name, const int type, cv::Mat& pose,
  const int skip=0);

Eigen::Matrix4f getTransformMatrix(cv::Mat pose, int i);

//cv::Mat loadImage(const char* name, int idx=0, int width=640, bool depth=false);

void extractBlobs(Blobs &blobs, cv::Mat poses,
  cv::Mat Label, const uchar* fuse,
  std::vector<float> camera, const uchar rotated, const char* name,
  const bool esanet=false);

void convertBlobs(Blobs &blobs, bool legacy, Graph& snodes,
  const int min_frame=0, const int max_frame=-1, float* ari=nullptr,
  const int thresh=10, const bool real=true);

void extractPointCloud(Graph& snodes, cv::Mat poses, cv::Mat Label,
  const uchar* fuse, std::vector<float> camera, uchar rotated,
  const char* name, bool type, int offset=0, float* ari=nullptr,
  const int thresh=10);

#endif /* vo_hpp */
