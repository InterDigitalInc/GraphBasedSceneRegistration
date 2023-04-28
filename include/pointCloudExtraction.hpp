/*
Copyright © 2023 InterDigital
All Rights Reserved
This program contains proprietary information which is a trade secret/business secret of InterDigital R&D france is protected, even if unpublished, under applicable Copyright laws (including French droit d’auteur) and/or may be subject to one or more patent(s).
*/

#ifndef pointCloudExtraction_hpp
#define pointCloudExtraction_hpp

#include <opencv2/core.hpp>   //Mat
#include <eigen3/Eigen/Dense> //Eigen types
#include "nodes.hpp"          //vector

std::string bitDepthName(int depth);

#define LABELMAP 0
#define DEPTHMAP 1
#define CHANGMAP 2
cv::Mat loadImage(const char* name, int idx=0, int width=640,
  const uchar* fuse=nullptr, char type=LABELMAP);

//How much was the image rotated ? (so we can apply the reverse transform)
#define ROTATE_NONE 3

struct pointCloudExtraction {
public:
    pointCloudExtraction(std::vector<float> &camera, uchar rotated);
    //Flood-fill-based
    void insertValue(cv::Mat &depthImg, cv::Mat &labelImg,
      const cv::Mat &extraImg, const uchar* fuse=0);
    void RawBlobExtraction(Blobs &blobs,
      const bool channels, const int bits, Eigen::Matrix4f& pose, int frame_num,
      const bool real=true, const bool debug=false);
    void RegionExtractionB(Graph &snodes, const bool legacy,
      const bool channels, const int bits, Eigen::Matrix4f& pose, int frame_num,
      const int thr, const bool covariance=true,
      const bool real=true, const bool debug=false);
    //Contours-based
    /*void insertValueC(cv::Mat &depthImg, cv::Mat &labelImg, const uchar* fuse);
    void RegionExtractionC(std::vector<GNode> &points,
      Eigen::Matrix4f& pose) const;
    void RegionExtractionM(std::vector<GNode> &points,
      Eigen::Matrix4f& pose) const;
    void RegionExtractionF(std::vector<GNode> &points,
      Eigen::Matrix4f& pose) const;
    void RegionExtractionT(std::vector<GNode> &points,
      Eigen::Matrix4f& pose) const;*/
private:
    cv::Mat depthMap;
    cv::Mat labelMap;
    cv::Mat extraMap; //changes or instances
    cv::Mat threshX;
    cv::Mat threshY;
    float fx, fy, cx, cy, s;
    uchar rot;
};

#endif /* pointCloudExtraction_hpp */
