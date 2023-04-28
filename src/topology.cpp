#include "topology.hpp"

using namespace std;
using namespace Eigen;

#include <iostream> //debug

Topology::Topology(const Graph &snodes, const float thres): threshold(thres) {
  const unsigned int size(snodes.size());
  edges.setZero(size, size);
  degrees.setZero(size);
  orphans.setZero(size);
  labels.resize(size);
  count = 0;
  const float thres2 = thres*thres;
  for (unsigned int i = 0; i < size; i++) {//go up to "size" for the orphan nodes
    labels(i) = (int)snodes[i][0][3];
    for (unsigned int j = i+1; j < size; j++)
      for (const auto& p : snodes[i]) {
        float d2(0); //if snodes[j] is empty just exit
        for (const auto& q : snodes[j]) {
          d2 = pow(q[0]-p[0],2)+pow(q[1]-p[1],2)+pow(q[2]-p[2],2);
          if (d2 < thres2) {
            ++degrees(i);
            ++degrees(j);
            edges(j,i) = 1;
            edges(i,j) = 1;
            ++count;
            break; //exit snodes[j] loop
          }
        }
        if (d2 < thres2) break; //exit snodes[i] loop
      }
    if (degrees(i) == 0) orphans(i) = true;
  }
}

Topology::Topology(const Graph &snodes) {
  const unsigned int size(snodes.size());
  MatrixXf distances2 = MatrixXf::Zero(size, size);
  labels.resize(size);

  for (unsigned int i = 0; i < size; i++) { //go up to "size" for the orphan nodes(?)
    labels(i) = (int)snodes[i][0][3]-1; //HOTFIX -1 only for ScanNet ?
    for (unsigned int j = i+1; j < size; j++) {
      float mind2(-1);
      for (const GNode& p : snodes[i])
        for (const GNode& q : snodes[j]) {
          float d2(pow(q[0]-p[0],2)+pow(q[1]-p[1],2)+pow(q[2]-p[2],2));
          if (d2 < mind2 || mind2 < 0)
            mind2 = d2;
        }
      distances2(i,j) = max(mind2, 0.0f);
      distances2(j,i) = distances2(i,j);
    }
  }

  const float mean(distances2.sum()/(size*(size-1))); //ignore diagonal
  threshold = sqrt(0.75*mean);
  edges = (distances2.array() < 0.75*mean).cast<int>();
  edges.diagonal().array() = false; //no self-reference
  degrees = edges.colwise().sum();
  orphans = degrees.array() == 0;
  count = degrees.sum();
}

Topology::Topology(const Eigen::MatrixXi raw, const float thres) {
  const unsigned int size(raw.cols());
  edges = raw.topRows(size);
  labels = raw.row(size);
  degrees = edges.colwise().sum();
  orphans = degrees.array() == 0;
  count = degrees.sum();
  threshold = thres;
}

Topology::~Topology() {
}

vector<vector<unsigned int> > Topology::neighborhood() const {
    const unsigned int size(edges.rows());
    vector<vector<unsigned int> > neighborhood;
    neighborhood.reserve(size);
    for (unsigned int i = 0; i < size; i++) {
      vector<unsigned int> neighbors;
      neighbors.reserve(degrees(i));
      for (unsigned int j = 0; j < size; j++) {
        if (edges(i,j))
          neighbors.push_back(j);
      }
      neighborhood.push_back(neighbors);
    }
    return neighborhood;
}
