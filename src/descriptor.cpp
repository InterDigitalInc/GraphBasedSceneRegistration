#include "descriptor.hpp"

#include <iostream> //cout
#include <ctime>    //clock

//using namespace cv;
using namespace std;
using namespace Eigen;

Descriptor::Descriptor(const Topology &Top) {
    neighbor = Top.getEdges();
    labels = Top.getLabels();
    nodes = labels.size();
    isOrphan = Top.getOrphans();
}

Descriptor::~Descriptor() {
}

MatrixXf Descriptor::getNode(int idx) const {
    if (idx < DescriptorVector.size() && idx >= 0)
        return DescriptorVector[idx];

    cerr << "Descriptor's ID out of bounds" << endl;
    MatrixXf errorMat = MatrixXf::Zero(DescriptorVector[0].rows(),
                                       DescriptorVector[0].cols());
    return errorMat;
}

int Descriptor::size() const {
    return (int)DescriptorVector.size();
}

// Random walk descriptor
RandomWalkDescriptor::RandomWalkDescriptor(const unsigned int depth,
  const unsigned int count, const Topology &Top) : Descriptor(Top) {
    len = depth*count;
    degrees = Top.getDegrees();
    neighborhood = Top.neighborhood();
    generateRandomWalks(count, depth, RANDOM_UNIFORM);
    for (unsigned int i = 0; i < nodes; ++i) {
      MatrixXf SingleDescriptor(count, depth);
      for (unsigned int w = 0; w < count; ++w)
        for (unsigned int d = 0; d < depth; ++d)
          SingleDescriptor(w, d) = labels(walks[i][w][d]);
      DescriptorVector.push_back(SingleDescriptor);
    }
}

//Reimplementation of the Matcher code
/*void RandomWalkDescriptor::compare(const RandomWalkDescriptor& other,
  MatrixXf &scoreMatrix) const {
    const int size1 =       DescriptorVector.size();
    const int size2 = other.DescriptorVector.size();
    scoreMatrix.setZero(size1, size2);

    const int matrixRow(DescriptorVector[0].rows());
    for (int i = 0; i < size1; i++) {
      if (isOrphan(i) == true) continue;
      for (int j = 0; j < size2; j++) {
        if (other.isOrphan(j) == true) continue;
        if (DescriptorVector[i](0, 0) != other.DescriptorVector[j](0, 0))
          continue; //check node label
        // int scoreCount1(0), scoreCount2(0);
        // for (int row1 = 0; row1 < matrixRow; row1++)
        //   for (int row2 = 0; row2 < matrixRow; row2++)
        //     if (other.DescriptorVector[j].row(row2)
        //            == DescriptorVector[i].row(row1)) {
        //       scoreCount1++;
        //       break;
        //     }
        // for (int row1 = 0; row1 < matrixRow; row1++)
        //   for (int row2 = 0; row2 < matrixRow; row2++)
        //     if (other.DescriptorVector[j].row(row1)
        //            == DescriptorVector[i].row(row2)) {
        //       scoreCount2++;
        //       break;
        //     }
        // //use the smaller score as the final score value
        // if (scoreCount1 < scoreCount2)
        //   scoreMatrix(i, j) = (float)scoreCount1/matrixRow;
        // else
        //   scoreMatrix(i, j) = (float)scoreCount2/matrixRow;
        int scoreCount(0);
        //SPEED: DO NOT CREATE INTERMEDIARY EIGEN VECTORS
        for (int row1 = 0; row1 < matrixRow; row1++) {
          bool duplicate(false); //check for duplicate walks //OLIVIER
          for (int row0 = 0; row0 < row1; row0++)
            if (DescriptorVector[i].row(row1)
             == DescriptorVector[i].row(row0)) {
              duplicate = true;
              break;
            }
            if (duplicate) continue;
            for (int row2 = 0; row2 < matrixRow; row2++)
              if (other.DescriptorVector[j].row(row2)
                     == DescriptorVector[i].row(row1)) {
                scoreCount++;
                break;
              }
        }
        scoreMatrix(i, j) = (float)scoreCount/matrixRow;
      }
    }
}*/

const unsigned int RandomWalkDescriptor::nextVertex(
  const unsigned int current, const unsigned int previous, char type) const {

  // Sample from a uniform probability distribution p \in [0,1).
  //const real_t p = random_distribution_(random_engine_);
  const int p{rand()};

  const int degree{degrees(current)};
  if (degree == 0) return current; //no neighbors

  // Get a list of neighbor vertices of the current vertex.
  const vector<unsigned int> neighbors = neighborhood[current];
  if (degree == 1) return neighbors.front(); //only one neighbor

  // Depending on the sampling type, the next vertex is chosen differently.
  switch (type) {
    case RANDOM_UNIFORM: {
      const int advance_step = p % degree; //static_cast<int>(p * degree);
      return neighbors[advance_step];
    }
    case RANDOM_AVOIDING: { // avoid same label
      const int current_label = labels[current];
      vector<unsigned int> new_neighbors;
      new_neighbors.reserve(degree);
      for (unsigned int j : neighbors)
        if (labels[j] != current_label)
          new_neighbors.push_back(j);
      const unsigned int count = new_neighbors.size();
      if (count > 0) {
        const int advance_step = p % count; //static_cast<int>(p * count);
        return new_neighbors[advance_step];
      } else { // all same label: back to uniform sampling
        const int advance_step = p % degree; //static_cast<int>(p * degree);
        return neighbors[advance_step];
      }
    }
    case RANDOM_WEIGHTED: {
      vector<uint64_t> weights(degree);
      uint64_t weight_sum = 0;
      unsigned int count = 0;
      for (unsigned int j : neighbors) {
        weights[count] = neighbor(current, j);
        weight_sum += weights[count++];
      }
      uint64_t p_w = p % weight_sum; //static_cast<uint64_t>(p * weight_sum);
      for (int j = 0; j < count; ++j) {
        if (p_w < weights[j])
          return neighbors[j];
        p_w -= weights[j];
      }
    }
    case RANDOM_NON_RETURNING: { // do not return to previous node
      // Generate a list of neighbors without the previous node
      vector<unsigned int> new_neighbors(degree); //instead of degree-1 for the first step
      unsigned int count = 0;
      for (unsigned int j : neighbors) {
        if (j == previous) continue;
        new_neighbors[count++] = j;
      }
      const int advance_step = p % count; //static_cast<int>(p * count);
      return new_neighbors[advance_step];
    }
    default:
      cerr << "Unrecognized random walk sampling type.";
      return current;
  }
}

void RandomWalkDescriptor::generateRandomWalks(
  const unsigned int count, const unsigned int depth, const char type) {
  srand(time(NULL));
  walks.clear();
  walks.resize(nodes);

  for (unsigned int i = 0; i < nodes; ++i) { //for each node
    for (unsigned int w = 0; w < count; ++w) { //create count random walks.
      vector<unsigned int> walk(depth);
      unsigned int current{i}; //walk's seed
      unsigned int previous{nodes}; //impossible node: first step is free
      for (unsigned int step = 0; step < depth; ++step) {
        const unsigned int next = nextVertex(current, previous, type);
        walk[step] = next; // Add the vertex to the walk.
        previous = current;
        current = next;
      }
      walks[i].push_back(walk); //Add the generated walk to the walks container.
    }
  }
}

// New Fast descriptor (should be replaced with "neighbor vector" representation)
/*FastDescriptor::FastDescriptor(const Topology &Top,
  int labelSize) : Descriptor(Top) {
    len = labelSize*labelSize*labelSize;
    for (unsigned int i = 0; i < nodes; ++i) {
      MatrixXf SingleDescriptor = MatrixXf::Zero(len, 1);
      const int label0{labels(i)*labelSize*labelSize};
      for (unsigned int j = 0; j < nodes; ++j)
        if (neighbor(i, j)) {
          const int label1{labels(j)*labelSize};
          for (unsigned int k = 0; k < nodes; ++k)
            if (neighbor(j, k))
              ++SingleDescriptor(label0+label1+labels(k), 0);
        }
      DescriptorVector.push_back(SingleDescriptor);
    }
}*/

FastDescriptor::FastDescriptor(const int depth, const int labelSize,
  const Topology &Top) : Descriptor(Top) {
    len = labelSize;
    for (int j = 1; j < depth; j++) len *= labelSize;

    int* path = new int[depth];
    for (unsigned int i = 0; i < nodes; ++i) {
      MatrixXf SingleDescriptor = MatrixXf::Zero(len, 1);
      path[0] = i;
      if (depth > 1) path[1] = 0; //start search from 0
      int d(1);
      while (d > 0) { //sequentially contruct paths from "i"
        while (d < depth && d > 0) { //create a path of length "depth", skipped if depth==1
          while (neighbor(path[d-1], path[d]) == false && path[d] < nodes)
            path[d]++;          //look for the next step
          if (path[d] == nodes)//we have explored all the neighbors of "path[d-1]"
            path[--d]++;        //go back to previous step and change node
          else
            path[++d] = 0;      //otherwise go on to next step (start search from 0)
        }
        if (d > 0) { //record in histogram, otherwise we explored every path from "i" and should stop the search
          int labelPath(labels(i)); //histogram row
          for (int j = 1; j < depth; j++)
            labelPath = labelSize*labelPath + labels(path[j]);
          SingleDescriptor(labelPath, 0)++;
          d = depth-1;    //if depth==1 then d==0 and the search is stopped
          path[d]++;      //look for the next neighbor, useless/harmless for depth==1
        }
      }
      DescriptorVector.push_back(SingleDescriptor);
    }
    delete path;
}

FastDescriptor::FastDescriptor(const int depth, const int merge,
  const int labelSize, const Topology &Top) : Descriptor(Top) {
    const int lastSize(labelSize/merge);
    len = lastSize;
    for (int j = 0; j < depth-1; j++) len *= labelSize;

    int* w = new int[depth];
    for (unsigned int i = 0; i < nodes; ++i) {
      MatrixXf SingleDescriptor = MatrixXf::Zero(len, 1);
      w[0] = i;
      if (depth > 1) w[1] = 0; //start search from 0
      int d(1);
      while (d > 0) { //sequentially contruct walks from "i"
        while (d < depth && d > 0) { //create a walk of length "depth", skipped if depth==1
          while (neighbor(w[d-1], w[d]) == false && w[d] < nodes) //check if edge
            w[d]++;          //look for the next step
          if (w[d] == nodes)//we have explored all the neighbors of "w[d-1]"
            w[--d]++;        //go back to previous step and change node
          else
            w[++d] = 0;      //otherwise go on to next step (start search from 0)
        }
        if (d > 0) { //record in histogram, otherwise we explored every walk from "i" and should stop the search
          int row(0);//(labels(i)); //histogram row
          for (int j = 0; j < depth-1; j++)
            row = labelSize*row + labels(w[j]);
          row = lastSize*row + labels(w[depth-1])/merge; //last step with merge
          SingleDescriptor(row, 0)++;
          d = depth-1;    //if depth==1 then d==0 and the search is stopped
          w[d]++;      //look for the next neighbor, useless/harmless for depth==1
        }
      }
      DescriptorVector.push_back(SingleDescriptor);
    }
    delete w;
}

// New descriptor
/*NewDescriptor::NewDescriptor(const int depth, const int labelSize,
  const Topology &Top) : Descriptor(Top) {
    // depth == 1: the histogram is null except at the (node label)
    // depth == 2: the histogram is null except at the (node label, neighbor label)
    // ...
    vector<vector<MatrixXf> > HistVectors; //vector<MatrixXf> *HistVectors = new vector<MatrixXf>[depth];
    vector<MatrixXf> HistVector; // first column is starting point
    for (int i = 0; i < nodes; i++) { //d = 0
      MatrixXf hist = MatrixXf::Zero(labelSize, 1);
      hist(labels(i), 0) = 1;
      HistVector.push_back(hist);
    }
    HistVectors.push_back(HistVector);

    int histStep(1);                        //previous histogram start-node-step
    for (int d = 1; d < depth; d++) {       //with increasing walk length
      int histSize(histStep*labelSize);     //previous histogram size
      HistVector.clear();
      for (int i = 0; i < nodes; i++) {    //for each node
        MatrixXf hist = MatrixXf::Zero(histSize*labelSize, 1);  //new histogram
        if (isOrphan[i] == 0) {           //do not bother looping if the node has no neighbor
          int labeli(labels(i)*histSize);
          for (int j = 0; j < nodes; j++)  //...
            if (neighbor(i,j))              //for each neighbor (automatically checks isOrphan[j]?)
              hist.block(labeli + labels(j)*histStep,0,
                         histSize,1) += HistVectors[d-1][j]; //add previous histogram to new one
        }
        HistVector.push_back(hist);
      }
      HistVectors.push_back(HistVector);
      histStep = histSize;
    }
    DescriptorVector = HistVectors[depth-1]; //we did not free the other depths
    len = histStep*labelSize;
}*/

// Adjacency descriptor
AdjacencyDescriptor::AdjacencyDescriptor(const int depth,
  const int labelSize, const Topology &Top, const bool normalize) : Descriptor(Top) {
    len = depth*labelSize;

    for (unsigned int i = 0; i < nodes; i++) { //d = 0, initialize each descriptor
      MatrixXf hist = MatrixXf::Zero(len, 1);
      hist(labels(i), 0) = 1;
      DescriptorVector.push_back(hist);
    }

    MatrixXi AN = MatrixXi::Identity(nodes, nodes);
    for (int d = 1; d < depth; d++) {
      AN *= neighbor; //cout << AN << endl;
      const float inc(normalize ? 1.0f/d : 1.0f);
      for (unsigned int i = 0; i < nodes; ++i)
        if (isOrphan[i] == 0)
          for (unsigned int j = 0; j < nodes; ++j)
            if (AN(i,j))
              DescriptorVector[i](d*labelSize+labels(j), 0) += inc;
    }
}

AdjacencyDescriptor::AdjacencyDescriptor(const int depth, const int strat,
  const int labelSize, const Topology &Top, const bool normalize) : Descriptor(Top) {
    len = depth*labelSize;

    for (unsigned int i = 0; i < nodes; i++) { //d = 0, initialize each descriptor
      MatrixXf hist = MatrixXf::Zero(len, 1);
      hist(labels(i), 0) = 1;
      DescriptorVector.push_back(hist);
    }
    if (depth == 1) return;

    //d = 1, neighbors
    for (unsigned int i = 0; i < nodes; i++)
      if (isOrphan[i] == 0)
        for (unsigned int j = i+1; j < nodes; j++)
          if (neighbor(i,j)) {
            DescriptorVector[i](labelSize+labels(j), 0)++;
            DescriptorVector[j](labelSize+labels(i), 0)++;
          }
    if (depth == 2) return;

    //d = 2, initialize the A and secondary B matrices
    const MatrixXi A = neighbor; //cout << "A(1)\n" << A << endl;
    MatrixXi AN = A*A;           //cout << "A(2)\n" << AN << endl;
    MatrixXi BS = AN;
    BS.diagonal().array() = 0;   //cout << "B(2)\n" << BS << endl;
    for (unsigned int i = 0; i < nodes; i++)
      if (isOrphan[i] == 0)
        for (unsigned int j = i+1; j < nodes; j++)
          if (BS(i,j)) {
            DescriptorVector[i](2*labelSize+labels(j), 0)+=(normalize ? 0.5f : 1.0f);
            DescriptorVector[j](2*labelSize+labels(i), 0)+=(normalize ? 0.5f : 1.0f);
          }
    if (depth == 3) return;

    //d = 3, initialize the main B matrix
    AN *= A;                     //cout << "A(3)\n" << AN << endl;
    MatrixXi B = BS*A+A*BS-AN+A; //cout << "B(3)\n" << B << endl;, we assume that A is binary, otherwise use A.array()*A.array()*A.array()
    for (unsigned int i = 0; i < nodes; i++)
      if (isOrphan[i] == 0)
        for (unsigned int j = i+1; j < nodes; j++)
          if (B(i,j)) {
            DescriptorVector[i](3*labelSize+labels(j), 0)+=(normalize ? 1.0f/3 : 1.0f);
            DescriptorVector[j](3*labelSize+labels(i), 0)+=(normalize ? 1.0f/3 : 1.0f);
          }
    if (depth == 4) return;

    //d > 3, main loop, initialize matrix buffer for B
    MatrixXi BB(nodes, nodes);
    for (int d = 4; d < depth; d++) {
      BB = B;
      B  = B*A+A*B-A*BS*A-(A.array()*AN.array()).matrix(); //we assume that A is binary, otherwise use A.array()*A.array()*AN.array()
      AN *= A; //cout << "A("<<d<<")\n" << AN << endl;
      BS = BB; //cout << "B("<<d<<")\n" << B << endl;
      const float inc(normalize ? 1.0f/d : 1.0f);
      for (unsigned int i = 0; i < nodes; i++)
        if (isOrphan[i] == 0)
          for (unsigned int j = i+1; j < nodes; j++)
            if (B(i,j)) {
              DescriptorVector[i](d*labelSize+labels(j), 0)+= inc;
              DescriptorVector[j](d*labelSize+labels(i), 0)+= inc;
            }
    }
}

typedef Matrix<bool, Dynamic, Dynamic> MatrixXb;

//Multiply boolean matrices
//Optimized for symetric matrices (symetric in and out!)
//Write in top-right (i < j), read in bottom-left (i > j)
void symAND(const vector<vector<unsigned int> >& A, MatrixXb& B, const Index* c) {
  bool* dB = (bool*)B.data();
  const Index size{B.rows()};
  //write top-right
  for (Index i = 0; i < size; ++i) {
    bool* rBi = &(dB[c[i]]);
    const long unsigned int s{A[i].size()};
    for (Index j = i+1; j < size; ++j) { // i < j
      const bool* rBj = &(dB[c[j]]);
      const bool* cBj = &(dB[j]);
      long unsigned int k;
      for (k = 0; k < s; ++k)
        if (A[i][k] < j && rBj[A[i][k]] || A[i][k] >= j && cBj[c[A[i][k]]])
          break;
      rBi[j] = (k < s);
    }
  }
  //diagonal
  for (Index i = 0; i < size; ++i) {
          bool* rBi = &(dB[c[i]]);
    const bool* cBi = &(dB[i]);
    const long unsigned int s{A[i].size()};
    long unsigned int k;
    for (k = 0; k < s; ++k)
      if (A[i][k] < i && rBi[A[i][k]] || A[i][k] >= i && cBi[c[A[i][k]]])
        break;
    rBi[i] = (k < s);
  }
  //copy top-right to bottom-left
  for (Index i = 0; i < size; ++i) {
    const bool* rBi = &(dB[c[i]]);
          bool* cBi = &(dB[i]);
    for (Index j = i+1; j < size; ++j) // i < j
      cBi[c[j]] = rBi[j];
  }
}

// return an symmetric matrix out of an asymmetric product
void asyAND(const vector<vector<unsigned int> >& A, MatrixXb& B, const Index* c) {
  bool* dB = (bool*)B.data();
  const Index size{B.rows()};
  bool* buffer = new bool[size]; //row buffer

  for (Index i = 0; i < size; ++i) {
    bool* rBi = &(dB[c[i]]);
    for (Index j = 0; j < size; ++j) {
      const long unsigned int s{A[j].size()};
      long unsigned int k;
      for (k = 0; k < s; ++k)
        if (rBi[A[j][k]]) break;
      buffer[j] = (k < s);
    }
    for (Index j = 0; j < size; ++j)
      rBi[j] = buffer[j];
  }
  delete[] buffer;
  //make symmetric
  for (Index i = 0; i < size; ++i) {
    bool* rBi = &(dB[c[i]]);
    bool* cBi = &(dB[i]);
    for (Index j = i+1; j < size; ++j) {
      if (cBi[c[j]] == false)
        rBi[j] = false;
      else if (rBi[j] == false)
        cBi[c[j]] = false;
    }
  }
}

// Use C and D as filters
void asyAND(const vector<vector<unsigned int> >& A, MatrixXb& B, const MatrixXb& C, const MatrixXb& D, const Index* c) {
  bool* dB = (bool*)B.data();
  const Index size{B.rows()};
  bool* buffer = new bool[size]; //row buffer

  for (Index i = 0; i < size; ++i) {
    bool* rBi = &(dB[c[i]]);
    for (Index j = 0; j < size; ++j) {
      if (C(i,j) && D(i,j))
        buffer[j] = false;
      else {
        const long unsigned int s{A[j].size()};
        long unsigned int k;
        for (k = 0; k < s; ++k)
          if (rBi[A[j][k]]) break;
        buffer[j] = (k < s);
      }
    }
    for (Index j = 0; j < size; ++j)
      rBi[j] = buffer[j];
  }
  delete[] buffer;
  //make symmetric
  for (Index i = 0; i < size; ++i) {
    bool* rBi = &(dB[c[i]]);
    bool* cBi = &(dB[i]);
    for (Index j = i+1; j < size; ++j) {
      //if (C(i,j) && D(i,j))
      //  continue;
      if (cBi[c[j]] == false)
        rBi[j] = false;
      else if (rBi[j] == false)
        cBi[c[j]] = false;
    }
  }
}

BooleanDescriptor::BooleanDescriptor(const int depth,
  const int labelSize, const Topology &Top, const bool normalize) : Descriptor(Top) {
    len = depth*labelSize;

    for (Index i = 0; i < nodes; ++i) { //d = 0, initialize each descriptor
      MatrixXf hist = MatrixXf::Zero(len, 1);
      hist(labels(i), 0) = 1;
      DescriptorVector.push_back(hist);
    }

    Index* c = new Index[nodes];
    for (Index i = 0; i < nodes; ++i) c[i] = i*nodes;

    vector<vector<unsigned int> > A = Top.neighborhood();
    MatrixXb AN = MatrixXb::Identity(nodes, nodes);
    //const MatrixXb B = neighbor.cast<bool>();
    //MatrixXb BN = MatrixXb::Identity(nodes, nodes);
    for (int d = 1; d < depth; d++) {
      symAND(A, AN, c);
      //BN *= B; //cout << (BN==AN) << ' ';
      const float inc{normalize ? 1.0f/d : 1.0f};
      for (int i = 0; i < nodes; i++)
        if (isOrphan[i] == 0)
          for (int j = 0; j < nodes; j++)
            if (AN(i,j))
            //if (BN(i,j))
              DescriptorVector[i](d*labelSize+labels(j), 0) += inc;
    }
    delete[] c;
}

BooleanDescriptor::BooleanDescriptor(const int depth, const int strat,
  const int labelSize, const Topology &Top, const bool normalize) : Descriptor(Top) {
    len = depth*labelSize;

    for (unsigned int i = 0; i < nodes; i++) { //d = 0, initialize each descriptor
      MatrixXf hist = MatrixXf::Zero(len, 1);
      hist(labels(i), 0) = 1;
      DescriptorVector.push_back(hist);
    }

    if (depth == 1) return;

    //d = 1, neighbors
    for (unsigned int i = 0; i < nodes; i++)
      if (isOrphan[i] == 0)
        for (unsigned int j = i+1; j < nodes; j++)
          if (neighbor(i,j)) {
            DescriptorVector[i](labelSize+labels(j), 0)++;
            DescriptorVector[j](labelSize+labels(i), 0)++;
          }
    if (depth == 2) return;

    Index* c = new Index[nodes];
    for (Index i = 0; i < nodes; ++i) c[i] = i*nodes;

    //d = 2, initialize the N and C matrices
    const MatrixXb N = neighbor.cast<bool>();
    // MatrixXb NN(nodes,nodes);
    // zymXbSQR(N, NN, c);
    vector<vector<unsigned int> > A = Top.neighborhood();
    MatrixXb NN = N;
    symAND(A, NN, c);
    //MatrixXb NN = N*N;
    MatrixXb C = NN;
    C.diagonal().array() = false; //cout << "C(2)\n" << CS << endl;
    for (unsigned int i = 0; i < nodes; i++)
      if (isOrphan[i] == 0)
        for (unsigned int j = i+1; j < nodes; j++)
          if (C(i,j)) {
            DescriptorVector[i](2*labelSize+labels(j), 0) += (normalize ? 0.5f : 1.0f);
            DescriptorVector[j](2*labelSize+labels(i), 0) += (normalize ? 0.5f : 1.0f);
          }
    if (depth == 3) {
      delete[] c;
      return;
    }

    //d = 3,
    asyAND(A, C, c);//cout << "C("<<d<<")\n" << C << endl;
    //asyAND(N, C, c);//cout << "C("<<d<<")\n" << C << endl;
    // C = ((C*N).array()*(N*C).array()).matrix();
    for (unsigned int i = 0; i < nodes; i++)
      if (isOrphan[i] == 0)
        for (unsigned int j = i+1; j < nodes; j++)
          if (C(i,j)) {
            DescriptorVector[i](3*labelSize+labels(j), 0)+=(normalize ? 1.0f/3 : 1.0f);
            DescriptorVector[j](3*labelSize+labels(i), 0)+=(normalize ? 1.0f/3 : 1.0f);
          }
    if (depth == 4) {
      delete[] c;
      return;
    }

    //MatrixXb D = C;
    //d > 3, main loop
    //NN == N*N
    for (int d = 3; d < depth; d++) {
      // NN *= N;
      symAND(A, NN, c); //NN == N^(d-1)
      asyAND(A, C, N, NN, c);
      //C = (C.array()*not(N.array()*NN.array())).matrix();//cout << "C("<<d<<")\n" << C << endl;
      //D = ((D*N).array()*(N*D).array()*not(N.array()*NN.array())).matrix(); cout << (D==C);
      // C = ((C*N).array()*(N*C).array()*not(N.array()*NN.array())).matrix();
      const float inc(normalize ? 1.0f/d : 1.0f);
      for (unsigned int i = 0; i < nodes; i++)
        if (isOrphan[i] == 0)
          for (unsigned int j = i+1; j < nodes; j++)
            if (C(i,j)) {
              DescriptorVector[i](d*labelSize+labels(j), 0)+= inc;
              DescriptorVector[j](d*labelSize+labels(i), 0)+= inc;
            }
    }
    delete[] c;

    /*//d > 2, main loop
    for (int d = 3; d < depth; d++) {
      C = ((C*N).array()*(N*C).array()).matrix();//cout << "C("<<d<<")\n" << C << endl;
    }*/
}

// Elementary descriptor
ElementaryDescriptor::ElementaryDescriptor(const int depth, const int labelSize,
  const Topology &Top, const bool normalize) : Descriptor(Top) {
    len = depth*labelSize;

    for (unsigned int i = 0; i < nodes; i++) { //d = 0, initialize each descriptor
      MatrixXf hist = MatrixXf::Zero(len, 1);
      hist(labels(i), 0) = 1;
      DescriptorVector.push_back(hist);
    }

    vector<vector<unsigned int> > neighborhood = Top.neighborhood();
    vector<unsigned int> Bb(depth*nodes*nodes);
    for (unsigned int b = 0; b < nodes; b++) Bb[b+b*nodes] = 1;

    vector<unsigned int> path(depth);
    vector<unsigned int> nidx(depth);
    for (unsigned int i = 0; i < nodes; i++) {
      if (neighborhood[i].empty())
        continue;
      path[0] = i;
      if (depth > 1) path[1] = neighborhood[i][0]; //start search from nidx[1]==0
      nidx[0] = 0;
      nidx[1] = 0;
      int d{1};
      while (d > 0) { //sequentially contruct paths from "i"
        while (d < depth && d > 0) { //create a path of length "depth", skipped if depth==1
          if (nidx[d] == neighborhood[path[d-1]].size()) {  //we have explored all the neighbors of "path[d-1]"
            nidx[--d]++;        //go back to previous step and change node
          } else {
            path[d] = neighborhood[path[d-1]][nidx[d]];
            Bb[nodes*(nodes*d+path[0])+path[d]] += 1;
            if (++d < depth) //d can be equal to depth!
              nidx[d] = 0;      //otherwise go on to next step (start search from nidx[d]==0)
          }
        }
        if (d > 0) { //record in histogram, otherwise we explored every path from "i" and should stop the search
          d = depth-1;    //if depth==1 then d==0 and the search is stopped
          ++nidx[d];      //otherwise go on to next node
        }
      }
    }
    for (int d = 1; d < depth; d++) {
      const float inc{normalize ? 1.0f/d : 1.0f};
      for (unsigned int i = 0; i < nodes; i++)
        for (unsigned int j = 0; j < nodes; j++)
          if (Bb[nodes*(nodes*d+i)+j])
            DescriptorVector[i](d*labelSize+labels(j), 0) += inc;
    }
}

ElementaryDescriptor::ElementaryDescriptor(const int depth, const int strat,
  const int labelSize, const Topology &Top, const bool normalize) : Descriptor(Top) {
    len = depth*labelSize;

    for (unsigned int i = 0; i < nodes; i++) { //d = 0, initialize each descriptor
      MatrixXf hist = MatrixXf::Zero(len, 1);
      hist(labels(i), 0) = 1;
      DescriptorVector.push_back(hist);
    }

    MatrixXb N = neighbor.cast<bool>(); //temporary adjacency
    vector<vector<unsigned int> > neighborhood = Top.neighborhood();
    vector<unsigned int> Bb(depth*nodes*nodes);
    for (unsigned int b = 0; b < nodes; b++) Bb[b+b*nodes] = 1;

    vector<unsigned int> path(depth);
    vector<unsigned int> nidx(depth);
    for (unsigned int i = 0; i < nodes; i++) {
      if (neighborhood[i].empty())
        continue;
      path[0] = i;
      if (depth > 1) path[1] = neighborhood[i][0]; //start search from nidx[1]==0
      nidx[0] = 0; //debug: should change once when node is done
      nidx[1] = 0;
      int d{1};
      while (d > 0) { //sequentially contruct paths from "i"
        while (d < depth && d > 0) { //create a path of length "depth", skipped if depth==1
          while (nidx[d] < neighborhood[path[d-1]].size() && //we have not exhausted the neighbors
                 N(path[d-1], neighborhood[path[d-1]][nidx[d]]) == false) //there is not path to the node
            ++nidx[d];          //look for the next step
          if (nidx[d] == neighborhood[path[d-1]].size()) { //if (path[d] == size)  //we have explored all the neighbors of "path[d-1]"
            if (d > 1) N(path[d-2], path[d-1]) = N(path[d-1], path[d-2]) = true; //"if" matters for next i
            nidx[--d]++;        //go back to previous step and change node
          } else {
            path[d] = neighborhood[path[d-1]][nidx[d]];
            Bb[nodes*(nodes*d+path[0])+path[d]] += 1;
            N(path[d-1], path[d]) = N(path[d], path[d-1]) = false; //d > 0
            if (++d < depth) //d can be equal to depth!
              nidx[d] = 0;      //otherwise go on to next step (start search from nidx[d]==0)
          }
        }
        if (d > 0) { //record in histogram, otherwise we explored every path from "i" and should stop the search
          if (d > 1) N(path[d-2], path[d-1]) = N(path[d-1], path[d-2]) = true;
          d = depth-1;    //if depth==1 then d==0 and the search is stopped
          ++nidx[d];           //look for the next step
        }
      }
    }
    for (int d = 1; d < depth; d++) {
      const float inc{normalize ? 1.0f/d : 1.0f};
      for (unsigned int i = 0; i < nodes; i++)
        for (unsigned int j = 0; j < nodes; j++)
          if (Bb[nodes*(nodes*d+i)+j])
            DescriptorVector[i](d*labelSize+labels(j), 0) += inc;
    }
}

//Don't pass twice by the same node
SnakeDescriptor::SnakeDescriptor(const int depth,
  const int labelSize, const Topology &Top, const bool normalize) : Descriptor(Top) {
    len = depth*labelSize;

    for (unsigned int i = 0; i < nodes; i++) { //d = 0, initialize each descriptor
      MatrixXf hist = MatrixXf::Zero(len, 1);
      hist(labels(i), 0) = 1;
      DescriptorVector.push_back(hist);
    }

    vector<bool> visited(nodes);
    vector<vector<unsigned int> > neighborhood = Top.neighborhood();
    vector<unsigned int> Bb(depth*nodes*nodes);
    for (int b = 0; b < nodes; b++) Bb[b+b*nodes] = 1;

    vector<unsigned int> path(depth);
    vector<unsigned int> nidx(depth);
    for (unsigned int i = 0; i < nodes; i++) {
      if (neighborhood[i].empty())
        continue;
      path[0] = i;
      if (depth > 1) path[1] = neighborhood[i][0]; //start search from nidx[1]==0
      nidx[0] = 0; //debug: should change once when node is done
      nidx[1] = 0;
      int d{1};
      while (d > 0) { //sequentially contruct paths from "i"
        while (d < depth && d > 0) { //create a path of length "depth", skipped if depth==1
          while (nidx[d] < neighborhood[path[d-1]].size() && visited[neighborhood[path[d-1]][nidx[d]]])
            ++nidx[d]; //look for the next step
          if (nidx[d] == neighborhood[path[d-1]].size()) { //if (path[d] == size)  //we have explored all the neighbors of "path[d-1]"
            if (d > 1) visited[path[d-1]] = false; //"if" matters for next i
            nidx[--d]++;                           //go back to previous step and change node
            // cout<<'X'<<endl; for (int p=1; p<d; p++) cout<<nidx[p]+1<<'/'<<neighborhood[path[p-1]].size()<<'\t';
          } else {
            // cout<<nidx[d]+1<<'/'<<neighborhood[path[d-1]].size()<<'\t';
            path[d] = neighborhood[path[d-1]][nidx[d]];
            Bb[nodes*(nodes*d+path[0])+path[d]] += 1;
            visited[path[d]] = true; //d > 0
            if (++d < depth) //d can be equal to depth!
              nidx[d] = 0;//path[d] = 0;      //otherwise go on to next step (start search from nidx[d]==0)
          }
        }
        if (d > 0) { //record in histogram, otherwise we explored every path from "i" and should stop the search
          if (d > 1) visited[path[d-1]] = false;
          d = depth-1;    //if depth==1 then d==0 and the search is stopped
          ++nidx[d];//++path[d];           //look for the next step
          // cout <<'D'<< endl; for (int p=1; p<d; p++) cout<<nidx[p]+1<<'/'<<neighborhood[path[p-1]].size()<<'\t';
        }
      }
    }
    // MatrixXi AN = neighbor;
    for (int d = 1; d < depth; d++) {
      // cout << d << endl << AN << endl << endl;
      // AN = AN*neighbor;
      // cout << d << endl << Bs[d] << endl << endl;
      const float inc(normalize ? 1.0f/d : 1.0f);
      for (unsigned int i = 0; i < nodes; i++)
        for (unsigned int j = 0; j < nodes; j++)
          if (Bb[nodes*(nodes*d+i)+j])
            DescriptorVector[i](d*labelSize+labels(j), 0) += inc;
    }
}

//Don't pass twice by the same edge but search by node (not depth)
/*SnakeDescriptor::SnakeDescriptor(const int depth, const int strat,
  const int labelSize, const Topology &Top, const bool normalize) : Descriptor(Top) {
    len = depth*labelSize;

    for (unsigned int i = 0; i < nodes; i++) { //d = 0, initialize each descriptor
      MatrixXf hist = MatrixXf::Zero(len, 1);
      hist(labels(i), 0) = 1;
      DescriptorVector.push_back(hist);
    }

    MatrixXb N = neighbor.cast<bool>(); //temporary adjacency
    vector<vector<unsigned int> > neighborhood = Top.neighborhood();
    vector<unsigned int> Bb(depth*nodes*nodes);
    for (int b = 0; b < nodes; b++) Bb[b+b*nodes] = 1; //B(0)

    vector<unsigned int> path(depth);
    vector<unsigned int> nidx(depth);

    for (unsigned int k = 2; k < depth; ++k) { //B(k-1) (B(1) should be N)
      for (unsigned int i = 0; i < nodes-1; ++i) {
        if (neighborhood[i].empty())
          continue;
        path[0] = i;
        for (unsigned int j = i+1; j < nodes; ++j) { //B(k-1)_i,j and B(k-1)_j,i
          if (neighborhood[j].empty())
            continue;
          //if we have path[k-1] == j, then (k-1)_i,j = B(k-1)_j,i = true
          path[1] = neighborhood[i][0]; //start search from nidx[1]==0
          nidx[0] = 0; //debug: should change once when node is done
          nidx[1] = 0;
          int d{1};
          while (d > 0) { //sequentially contruct paths from "i" to "j"
            while (d < k && d > 0) { //create a path of length "k", skipped if k==1
              while (nidx[d] < neighborhood[path[d-1]].size()) {//we have not exhausted the neighbors
                const unsigned int l{neighborhood[path[d-1]][nidx[d]]};
                if (N(path[d-1], l) == true && //there is a path to the node
                    !(l < j && Bb[nodes*(nodes*(k-d)+l)+j] == 0)) //there is a path of length (k-d) from the node to "j"
                  break;
                ++nidx[d];          //look for the next step
              }
              if (nidx[d] == neighborhood[path[d-1]].size()) { //if (path[d] == size)  //we have explored all the neighbors of "path[d-1]"
                if (d > 1) N(path[d-2], path[d-1]) = N(path[d-1], path[d-2]) = true; //"if" matters for next i
                nidx[--d]++;        //go back to previous step and change node
              } else {
                path[d] = neighborhood[path[d-1]][nidx[d]];
                Bb[nodes*(nodes*d+path[0])+path[d]] += 1;
                N(path[d-1], path[d]) = N(path[d], path[d-1]) = false; //d > 0
                if (++d < k) //d can be equal to k!
                  nidx[d] = 0;      //otherwise go on to next step (start search from nidx[d]==0)
              }
            }
            if (d > 0) { //record in histogram, otherwise we explored every path from "i" and should stop the search
              if (d > 1) N(path[d-2], path[d-1]) = N(path[d-1], path[d-2]) = true;
              d = k-1;    //if k==1 then d==0 and the search is stopped
              ++nidx[d];           //look for the next step
            }
          }

        }
      }
    }
}*/

#define MERGES 7
const unsigned char merges[MERGES] = {20, 10, 8, 5, 4, 2, 1};
const unsigned char ratios[MERGES] = {2, 4, 5, 8, 10, 20, 40};

void initDescriptor(Descriptor* &des,
  const char type, const int depth, const int labelSize, const Topology &Top) {
  switch (type) {
    case WALK:
    //des = new RandomWalkDescriptor(1+(depth+MERGES-2)/MERGES+2, ratios[(depth+MERGES-2)%MERGES]*labelSize, Top);
    des = new RandomWalkDescriptor(depth, labelSize*labelSize, Top);
    return;
    case FAST:
    des = new FastDescriptor(depth, labelSize, Top); //New Fast descriptor
    return;
    case FAS2:
    //des = new FastDescriptor(1+(depth+2)/4, 8 >> ((depth+2)%4), labelSize, Top); //New Fast descriptor
    des = new FastDescriptor(1+(depth+MERGES-2)/MERGES, merges[(depth+MERGES-2)%MERGES], labelSize, Top); //New Fast descriptor
    return;
    // case NEWD:
    // des = new NewDescriptor(depth, labelSize, Top); //New descriptor
    // return;
    case ADJ_:
    //des = new AdjacencyDescriptor(depth, labelSize, Top, true); //integer version
    des = new BooleanDescriptor(depth, labelSize, Top, false);
    return;
    case ADJN:
    //Normalized
    des = new BooleanDescriptor(depth, labelSize, Top, true);
    return;
    case BDJ_:
    //No-return matrix approximation
    des = new BooleanDescriptor(depth, 1, labelSize, Top, false);
    return;
    case BDJN:
    default:
    //Normalized no-return matrix approximation
    des = new BooleanDescriptor(depth, 1, labelSize, Top, true);
    return;
    case CDJ_:
    //No-return matrix
    des = new ElementaryDescriptor(depth, 1, labelSize, Top, false);
    //des = new SnakeDescriptor(depth, labelSize, Top, false); //not the same node twice
    return;
    case CDJN:
    //Normalized no-return matrix
    des = new ElementaryDescriptor(depth, 1, labelSize, Top, true);
    //des = new SnakeDescriptor(depth, labelSize, Top, true); //not the same node twice
    return;
  }
}
