/*
Copyright © 2023 InterDigital
All Rights Reserved
This program contains proprietary information which is a trade secret/business secret of InterDigital R&D france is protected, even if unpublished, under applicable Copyright laws (including French droit d’auteur) and/or may be subject to one or more patent(s).
*/

#ifndef descriptor_hpp
#define descriptor_hpp

#include "topology.hpp"
  //#include <vector> //vector
#include <eigen3/Eigen/Dense>  //Eigen types

//descriptor types
#include "utils.hpp"

#define RANDOM_UNIFORM 0
#define RANDOM_AVOIDING 1
#define RANDOM_WEIGHTED 2
#define RANDOM_NON_RETURNING 3

class Descriptor {
public:
    Descriptor(const Topology &Top);
    ~Descriptor();
    Eigen::MatrixXf getNode(int idx) const;
    int size() const;
    unsigned int getLen() const { return len; };
    VectorXb isOrphan;
    Eigen::VectorXi labels;
protected:
    Eigen::MatrixXi neighbor;
    std::vector<Eigen::MatrixXf> DescriptorVector;
    unsigned int len;
    unsigned int nodes;
};

/*
RandomWalkDescriptor based on the X-View implementation (replaces the original).
*/
class RandomWalkDescriptor : public Descriptor {
  public:
    RandomWalkDescriptor(const unsigned int depth, const unsigned int count,
      const Topology &Top);
    // void compare(const RandomWalkDescriptor &other,
      // Eigen::MatrixXf &scoreMatrix) const;
    unsigned int SN() const { return walks.size() ? walks.front().size() : 0; };
    unsigned int SL() const { return SN() ? walks.front().front().size() : 0; };
  private:
    const unsigned int nextVertex(
      const unsigned int current, const unsigned int previous, char type) const;
    void generateRandomWalks(const unsigned int count,
      const unsigned int depth, const char type);
    Eigen::VectorXi degrees;
    std::vector<std::vector<unsigned int> > neighborhood; //nodes*degree(node)
    std::vector<std::vector<std::vector<unsigned int> > > walks; //nodes*SN*SL
};

/*
Fast walk histogram, the first variant stops at neighbors or neighbors.
The "depth" variant allows for increased depth.
The "merge" variant allows for a more incremental histogram size increase, by
merging labels for the last step.
*/
class FastDescriptor : public Descriptor {
  public:
    //FastDescriptor(const Topology &Top, const int labelSize);
    FastDescriptor(const int depth, const int labelSize, const Topology &Top);
    FastDescriptor(const int depth, const int merge, const int labelSize,
      const Topology &Top); // merge labels on the last step for reduced size
};

/*
Not implemented. Walk Histogram descriptor with "recursive" properties
*/
/*class NewDescriptor : public Descriptor {
  public:
    NewDescriptor(const int depth, const int labelSize, const Topology &Top);
};*/

/*
Descriptor based on powers of the adjacency matrix, counting the number of paths
between the node and labels ("how many paths of length n are there?")
The "strat" variant approximates the exploration with no return available in the
random walk descriptor.
*/
class AdjacencyDescriptor : public Descriptor {//overflows
  public:
    AdjacencyDescriptor(const int depth, const int labelSize,
      const Topology &Top, const bool normalize=true);
    AdjacencyDescriptor(const int depth, const int strat, const int labelSize,
      const Topology &Top, const bool normalize=true);
};

/*
Like the AdjacencyDescriptor but work with boolean values as to not overflow and
speed-up the generation ("is there a path of length n?")
*/
class BooleanDescriptor : public Descriptor {
  public:
    BooleanDescriptor(const int depth, const int labelSize,
      const Topology &Top, const bool normalize=true);
    BooleanDescriptor(const int depth, const int strat, const int labelSize,
      const Topology &Top, const bool normalize=true);
};

/*
Adjacency descriptor with graph exploration. Slower than matrix-based
*/
class ElementaryDescriptor : public Descriptor {
  public:
    ElementaryDescriptor(const int depth, const int labelSize,
      const Topology &Top, const bool normalize=true);
    ElementaryDescriptor(const int depth, const int strat, const int labelSize,
      const Topology &Top, const bool normalize=true);
};

class SnakeDescriptor : public Descriptor {
  public:
    SnakeDescriptor(const int depth, const int labelSize,
      const Topology &Top, const bool normalize=true);
    // SnakeDescriptor(const int depth, const int strat, const int labelSize,
      // const Topology &Top, const bool normalize=true);
};

void initDescriptor(Descriptor* &Des,
  const char type, const int depth, const int labelSize, const Topology &Top);

#endif /* descriptor_hpp */
