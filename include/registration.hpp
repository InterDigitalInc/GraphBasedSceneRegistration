/*
Copyright © 2023 InterDigital
All Rights Reserved
This program contains proprietary information which is a trade secret/business secret of InterDigital R&D france is protected, even if unpublished, under applicable Copyright laws (including French droit d’auteur) and/or may be subject to one or more patent(s).
*/

#ifndef registration_hpp
#define registration_hpp

#include <eigen3/Eigen/Dense> //Eigen types

#include "utils.hpp"
#include "nodes.hpp" //vector, nodes

class Registration {
public:
  Registration(Graph nodes1, Graph nodes2, Eigen::MatrixX2i matcherID);
  Registration(std::vector<GNode> nodes1, std::vector<GNode> nodes2,
    Eigen::MatrixX2i changesID, int count);
  ~Registration();
  void matcherRANSAC(const float thresh, char type);
  void Alignment();
  void Alignment(const int iter);
  Eigen::MatrixXf evaluate(Eigen::Matrix3f R_gt, Eigen::Vector3f T_gt) const;
  Eigen::MatrixX2i inlierID; //potentially useful in main, subset of matcherID
  Eigen::Matrix3f Rotation;
  Eigen::Vector3f Translation;
private:
  bool super;
  std::vector<unsigned int> inliersMID; //matches ids, might be redundant
  std::vector<unsigned int> inliersSID, inliersTID; //gnode ids
  Eigen::MatrixX3f sourcePoints,  targetPoints;
  Eigen::MatrixX3f sourceInliers, targetInliers;
  Eigen::VectorXi  sourceNodes,   targetNodes; //data separators
  Eigen::MatrixX2i totalID;
  Eigen::VectorXi labelVector; //for the evaluation
};

#endif /* registration_hpp */
