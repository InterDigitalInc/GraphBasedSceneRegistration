#ifndef topology_hpp
#define topology_hpp

#include <eigen3/Eigen/Dense>  //Eigen types
#include "nodes.hpp"           //vector, nodes

typedef Eigen::Matrix<bool, Eigen::Dynamic, 1> VectorXb;

class Topology {
  public:
    Topology(const Graph &snodes, const float thres);
    Topology(const Graph &snodes);
    Topology(const Eigen::MatrixXi raw, const float thres);
    ~Topology();
    std::vector<std::vector<unsigned int>> neighborhood() const;
    Eigen::VectorXi getLabels() const { return labels; };
    unsigned int getCount() const { return count; };
    Eigen::MatrixXi getEdges() const { return edges; };
    Eigen::VectorXi getDegrees() const { return degrees; };
    VectorXb getOrphans() const { return orphans; };
    float getThreshold() const { return threshold; };
  private:
    Eigen::VectorXi labels;
    unsigned int count;
    Eigen::MatrixXi edges;
    Eigen::VectorXi degrees;
    VectorXb orphans;
    float threshold;
};

#endif /* topology_hpp */
