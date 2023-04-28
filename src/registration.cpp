#include "registration.hpp"

#include <iostream> //cout
#include <ctime>    //clock

using namespace std;
using namespace Eigen;

//the points are sorted by matches
Registration::Registration(Graph nodes1, Graph nodes2, MatrixX2i matcherID) :
  totalID(matcherID) {

  const int size(matcherID.rows());
  sourceNodes.resize(size+1, 3);
  targetNodes.resize(size+1, 3);
  labelVector.resize(size);
  sourceNodes(0) = 0;
  targetNodes(0) = 0;
  for (int i = 0; i < size; ++i) {
    sourceNodes(i+1) = sourceNodes(i) + nodes1[matcherID(i, 0)].size();
    targetNodes(i+1) = targetNodes(i) + nodes2[matcherID(i, 1)].size();
  }

  //check the graphs are truly "super"
  super = size != sourceNodes(size) || size != targetNodes(size);
  sourcePoints.setZero(sourceNodes(size), 3);
  targetPoints.setZero(targetNodes(size), 3);
  unsigned int row1{0}, row2{0};
  for (unsigned int i = 0; i < size; ++i) {
    const int id1(matcherID(i, 0));
    const int id2(matcherID(i, 1));
    labelVector(i) = nodes1[id1][0][3];
    for (const auto& node : nodes1[id1]) {
      sourcePoints(row1, 0) = node[0];
      sourcePoints(row1, 1) = node[1];
      sourcePoints(row1, 2) = node[2];
      ++row1;
    }
    for (const auto& node : nodes2[id2]) {
      targetPoints(row2, 0) = node[0];
      targetPoints(row2, 1) = node[1];
      targetPoints(row2, 2) = node[2];
      ++row2;
    }
  }
}

//change detection beta
Registration::Registration(vector<GNode> nodes1,
  vector<GNode> nodes2, MatrixX2i changesID, int count) {
    const int size(changesID.rows());
    sourcePoints.resize(count, 3);
    targetPoints.resize(count, 3);
    labelVector.resize(count);
    totalID = changesID;
    //generate the source cloud and target cloud
    int j(0);
    for (int i=0; i<size; i++) {
        const int id1(changesID(i, 0));
        const int id2(changesID(i, 1));
        if (id2 < 0) continue;

        sourcePoints(j, 0) = nodes1[id1][0];
        sourcePoints(j, 1) = nodes1[id1][1];
        sourcePoints(j, 2) = nodes1[id1][2];

        targetPoints(j, 0) = nodes2[id2][0];
        targetPoints(j, 1) = nodes2[id2][1];
        targetPoints(j, 2) = nodes2[id2][2];

        labelVector(j++)   = nodes1[id1][3];
    }
}

Registration::~Registration() {
}

void getTransform(MatrixX3f source, MatrixX3f target, Matrix3f& R, Vector3f& T,
  const unsigned int iter) {
    const int size(source.rows());
    Matrix3Xf sourceRT;
    Vector3f average_sourceRT;
    Matrix3Xf converage_sourceRT;
    Matrix3f A;
    Matrix3f R0;
    Vector3f T0;

    R.setIdentity();
    T.setZero();

    const Vector3f average_target = target.colwise().mean();
    const MatrixX3f converage_target = target.rowwise() - average_target.transpose();

    for (unsigned int i = 0; i < iter; i++) {
        sourceRT = (R*source.transpose()).colwise() + T;

        //calculate the average value
        average_sourceRT = sourceRT.rowwise().mean();

        //obtain the convarience value of the points
        converage_sourceRT = sourceRT.colwise() - average_sourceRT;

        A = converage_sourceRT * converage_target;
        JacobiSVD<MatrixXf> svd(A, ComputeThinU | ComputeThinV);
        Matrix3f V(svd.matrixV()), U(svd.matrixU());
        R0 = V * U.transpose();
        T0 = average_target - R0 * average_sourceRT;

        R = R0 * R;
        T = T0 + T;
    }
}

void Registration::matcherRANSAC(const float thresh, char type) {
  //generate the random points for Ransac processing
  Matrix<float,4,3> sourceRandom;
  Matrix<float,4,3> targetRandom;
  //get the time value for obtaining the random points
  clock_t startTime, endTime;
  startTime = clock();

  const unsigned int size(totalID.rows());
  const unsigned int rows(sourcePoints.rows());
  if (!super) type = GENERICR;
  unsigned int maxCount{0};
  unsigned int inliersThresh;
  switch (type) {
    case GENERICR:
    case ONEPERMN: //one point per snode
    inliersThresh = size;
    break;
    case ONEPERSN: //one point per source gnode
    inliersThresh = rows;
    break;
    case ONEPERTN: //one point per target gnode (SLOW?)
    inliersThresh = targetPoints.rows();
    break;
    case ALLNODES:  //no point filter (all gnodes)
    default:
    inliersThresh = max(rows,(unsigned int)targetPoints.rows()); //rows*targetPoints.rows();
    break;
  }

  //initialize the inlier ID (?)
  //inlierID.setZero(1,2);
  for (unsigned int iter=0; iter<1000; iter++) {
    endTime = clock();
    int timedifference((int)((endTime - startTime)*10000));
    srand((int)time(0)+timedifference);

    //obtain the random points...
    for (unsigned char i = 0; i < 4; ++i) {
      //...by picking random snode
      // unsigned int id(rand()%size);
      // unsigned int s(rand()%(sourceNodes(id+1)-sourceNodes(id))+sourceNodes(id));
      //...by picking random gnode
      unsigned int s(rand()%rows);
      unsigned int id;
      for (id = 0; id < size; ++id) if (sourceNodes(id+1) > s) break;
      unsigned int t(rand()%(targetNodes(id+1)-targetNodes(id))+targetNodes(id));
      sourceRandom.row(i) = sourcePoints.row(s);
      targetRandom.row(i) = targetPoints.row(t);
    }

    //calculate the R and T
    Matrix3f R;
    Vector3f T;
    getTransform(sourceRandom, targetRandom, R, T, 1);

    //find the inliers
    vector<unsigned int> inliersM, inliersS, inliersT;
    inliersM.reserve(inliersThresh); //match index
    if (super) {
      inliersS.reserve(inliersThresh); //source gnode index
      inliersT.reserve(inliersThresh); //target gnode index
    }
    Vector3f newSource, newDifference; //only used for distance computation
    switch (type) {
      case ONEPERMN: //one point per snode match
      for (unsigned int i = 0; i < size; ++i) {
        float minDistance{thresh}; //"not smaller" than thresh
        unsigned int bestS, bestT;
        for (unsigned int s = sourceNodes(i); s < sourceNodes(i+1); ++s) {
          newSource = R*sourcePoints.row(s).transpose() + T;
          for (unsigned int t = targetNodes(i); t < targetNodes(i+1); ++t) {
            newDifference = targetPoints.row(t) - newSource.transpose();
            float distance = newDifference.norm();
            if (distance < minDistance) {
              minDistance = distance;
              bestS = s;
              bestT = t;
            }
          }
        }
        if (minDistance < thresh) { //best for an snode
          inliersM.push_back(i);
          inliersS.push_back(bestS);
          inliersT.push_back(bestT);
        }
      }
      break;
      case ONEPERSN: //one point per source gnode
      for (unsigned int i = 0; i < size; ++i)
        for (unsigned int s = sourceNodes(i); s < sourceNodes(i+1); ++s) {
          float minDistance{thresh}; //"not smaller" than thresh
          unsigned int bestS, bestT;
          newSource = R*sourcePoints.row(s).transpose() + T;
          for (unsigned int t = targetNodes(i); t < targetNodes(i+1); ++t) {
            newDifference = targetPoints.row(t) - newSource.transpose();
            float distance = newDifference.norm();
            if (distance < minDistance) {
              minDistance = distance;
              bestS = s;
              bestT = t;
            }
          }
          if (minDistance < thresh) { //best for a source gnode
            inliersM.push_back(i);
            inliersS.push_back(bestS);
            inliersT.push_back(bestT);
          }
        }
      break;
      case ONEPERTN: //one point per target gnode (SLOW?)
      for (unsigned int i = 0; i < size; ++i)
        for (unsigned int t = targetNodes(i); t < targetNodes(i+1); ++t) {
          float minDistance{thresh}; //"not smaller" than thresh
          unsigned int bestS, bestT;
          for (unsigned int s = sourceNodes(i); s < sourceNodes(i+1); ++s) {
            newSource = R*sourcePoints.row(s).transpose() + T;
            newDifference = targetPoints.row(t) - newSource.transpose();
            float distance = newDifference.norm();
            if (distance < minDistance) {
              minDistance = distance;
              bestS = s;
              bestT = t;
            }
          }
          if (minDistance < thresh) { //best for a source gnode
            inliersM.push_back(i);
            inliersS.push_back(bestS);
            inliersT.push_back(bestT);
          }
        }
      break;
      case ALLNODES:  //no point filter (all gnodes)
      default:
      for (unsigned int i = 0; i < size; ++i) {
        for (unsigned int s = sourceNodes(i); s < sourceNodes(i+1); ++s) {
          newSource = R*sourcePoints.row(s).transpose() + T;
          for (unsigned int t = targetNodes(i); t < targetNodes(i+1); ++t) {
            newDifference = targetPoints.row(t) - newSource.transpose();
            if (newDifference.norm() < thresh) {
              inliersM.push_back(i);
              inliersS.push_back(s);
              inliersT.push_back(t);
            }
          }
        }
      }
      break;
      case GENERICR:  //gnodes are snodes, simplify loop (all gnodes)
        for (unsigned int i = 0; i < size; ++i) {
          newSource = R*sourcePoints.row(i).transpose() + T;
          newDifference = targetPoints.row(i) - newSource.transpose();
          if (newDifference.norm() < thresh)
            inliersM.push_back(i);
        }
      break;
    }
    //get the new best solution
    if (inliersM.size() <= maxCount) continue;
    maxCount = inliersM.size();

    //if the inliers are not enough, continue search
    if (100*maxCount < inliersThresh || maxCount < 5) continue;

    inliersMID = inliersM;
    if (super) {
      inliersSID = inliersS;
      inliersTID = inliersT;
    }
    inlierID.setZero(maxCount, 2); //store the matched snode IDs
    for (int i = 0; i < maxCount; ++i) {
      const unsigned int oldID{inliersMID[i]};
      inlierID(i, 0) = totalID(oldID, 0);
      inlierID(i, 1) = totalID(oldID, 1);
    }
  }
}

void Registration::Alignment() {
  const int size(inliersMID.size());
  sourceInliers.resize(size, 3);
  targetInliers.resize(size, 3);
  if (super) {
    for (int i = 0; i < size; ++i) {
      sourceInliers.row(i) = sourcePoints.row(inliersSID[i]);
      targetInliers.row(i) = targetPoints.row(inliersTID[i]);
    }
  } else {
    for (int i = 0; i < size; ++i) {
      sourceInliers.row(i) = sourcePoints.row(inliersMID[i]);
      targetInliers.row(i) = targetPoints.row(inliersMID[i]);
    }
  }
  getTransform(sourceInliers, targetInliers, Rotation, Translation, 1000);
}

void Registration::Alignment(const int iter) {
  //skip duplication (snodes are of size 1)
  if (!super) {
    getTransform(sourcePoints, targetPoints, Rotation, Translation, iter);
    return;
  }
  //using point duplication
  MatrixX3f sourceDupe, targetDupe;
  const unsigned int size(totalID.rows());
  int dsize{0};
  for (unsigned int i = 0; i < size; ++i)
    dsize+=(sourceNodes(i+1)-sourceNodes(i))*(targetNodes(i+1)-targetNodes(i));
  sourceDupe.resize(dsize, 3);
  targetDupe.resize(dsize, 3);
  unsigned int drow{0};
  for (unsigned int i = 0; i < size; ++i)
    for (unsigned int s = sourceNodes(i); s < sourceNodes(i+1); ++s)
      for (unsigned int t = targetNodes(i); t < targetNodes(i+1); ++t) {
        sourceDupe.row(drow) = sourcePoints.row(s);
        targetDupe.row(drow) = targetPoints.row(t);
        ++drow;
      }
  getTransform(sourceDupe, targetDupe, Rotation, Translation, iter);
}

MatrixXf Registration::evaluate(Matrix3f R_gt, Vector3f T_gt) const {
  MatrixX3f sourceRT(((Rotation*sourcePoints.transpose()).colwise()
                      +Translation).transpose());
  MatrixX3f sourceRT_gt(((R_gt*sourcePoints.transpose()).colwise()
                         +T_gt).transpose());
  MatrixXf results(sourcePoints.rows(), 3);
  cout << "GTvsTSou GTvsTarg    label" << endl;
  results.col(0) = (sourceRT_gt-sourceRT).rowwise().norm(); //similar to RMS
  results.col(1) = (sourceRT_gt-targetPoints).rowwise().norm(); //matching evaluation
  results.col(2) = labelVector.cast<float>();
  return results;
}
