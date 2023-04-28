/*
Copyright © 2023 InterDigital
All Rights Reserved
This program contains proprietary information which is a trade secret/business secret of InterDigital R&D france is protected, even if unpublished, under applicable Copyright laws (including French droit d’auteur) and/or may be subject to one or more patent(s).
*/

#ifndef matcher_hpp
#define matcher_hpp

#include "descriptor.hpp" //Descriptor
  //#include <eigen3/Eigen/Dense>

class Matcher {
public:
    Matcher(const Descriptor& Ds1, const Descriptor& Ds2);
    Matcher(const Descriptor& Ds1, const Descriptor& Ds2, const int type);
    ~Matcher();
    Eigen::MatrixX2i bestMatches();
    Eigen::MatrixX2i bestMatches(const float thresh);
    Eigen::MatrixX2i getChangesSorted(unsigned int& count) const;
    Eigen::MatrixX2i getChanges(unsigned int& count) const;

private:
    Eigen::MatrixXf scoreMatrix;
    int size1, size2;
    VectorXb noNei;
    VectorXb noNej; //NEW: do not try to match orphan nodes
};

//void initMatcher(Matcher* &matcher,
//  const char type, const int depth, const int labelSize, const Topology &Top);

Eigen::MatrixX2i match(const char type,
  const Descriptor& Ds1, const Descriptor& Ds2, const int ver=3);

Eigen::MatrixX2i changeD(const char type,
  const Descriptor& Ds1, const Descriptor& Ds2,
  unsigned int& count, const int ver=3);

#endif /* matcher_hpp */
