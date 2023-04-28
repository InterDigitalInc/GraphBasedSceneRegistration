#include "matcher.hpp"

#include <iostream>
#include <ctime>

using namespace std;
using namespace Eigen;

//Walk decriptor matching
Matcher::Matcher(const Descriptor& Ds1, const Descriptor& Ds2) {
    noNei = Ds1.isOrphan;
    noNej = Ds2.isOrphan; //NEW: do not try to match orphan nodes
    //the sum number of the descriptors
    size1 = Ds1.size();
    size2 = Ds2.size();
    //obtain the decriptors' matrix size
    MatrixXf descriptor0 = Ds1.getNode(0);
    int matrixRow(descriptor0.rows());
    scoreMatrix.setZero(size1, size2);

    for (int i = 0; i < size1; i++) {
        if (noNei(i) == 1) continue;
        MatrixXf descriptor1 = Ds1.getNode(i);
        for (int j = 0; j < size2; j++) {
            if (noNej(j) == 1) continue;
            MatrixXf descriptor2 = Ds2.getNode(j);
            if (descriptor1(0, 0) != descriptor2(0, 0)) //check node label
                continue;
            int scoreCount(0);
            //SPEED: DO NOT CREATE INTERMEDIARY EIGEN VECTORS
            for (int row1 = 0; row1 < matrixRow; row1++) {
                bool duplicate{false}; //check for duplicate walks //OLIVIER
                for (int row0 = 0; row0 < row1; row0++)
                    if (descriptor1.row(row1) == descriptor1.row(row0)) {
                      duplicate = true;
                      break;
                    }
                if (duplicate) continue;
                for (int row2 = 0; row2 < matrixRow; row2++)
                    if (descriptor1.row(row1) == descriptor2.row(row2)) {
                       scoreCount++;
                       break;
                    }
            }
            scoreMatrix(i, j) = (float)scoreCount/matrixRow;
        }
    }
}

//Vector descriptor matching
Matcher::Matcher(const Descriptor& Ds1, const Descriptor& Ds2, const int type) {
  type;
  size1 = Ds1.size();
  size2 = Ds2.size();
  scoreMatrix.setZero(size1, size2);
  VectorXi lablevector1 = Ds1.labels; //speeds up the process
  VectorXi lablevector2 = Ds2.labels;

  noNei = Ds1.isOrphan;
  VectorXf SumsBottomLeft = VectorXf::Zero(size1);
  for (int i = 0; i < size1; i++)
    if (noNei(i) == false)
      SumsBottomLeft(i) = Ds1.getNode(i).squaredNorm();

  noNej = Ds2.isOrphan; //NEW: do not try to match orphan nodes
  VectorXf SumsBottomRight = VectorXf::Zero(size2);
  for (int j = 0; j < size2; ++j)
    if (noNej(j) == false)
      SumsBottomRight(j) = Ds2.getNode(j).squaredNorm();

  for (int i = 0; i < size1; ++i) {
    if (noNei(i) == true) continue;
    MatrixXf descriptor1 = Ds1.getNode(i);
    const int lab1{lablevector1(i)};
    for (int j = 0; j < size2; ++j) {
      if (noNej(j) == true || lab1 != lablevector2(j)) continue; //most of the histogram is useless if we check the class anyway
      MatrixXf descriptor2 = Ds2.getNode(j);
      const float SumTop{(descriptor1.array()*descriptor2.array()).sum()}; //it's faster to convert to array on the fly
      const float SumBottom{sqrt(SumsBottomLeft(i)*SumsBottomRight(j))};
      scoreMatrix(i, j) = abs(SumTop/SumBottom);
    }
  }
}

Matcher::~Matcher() {
}

MatrixX2i Matcher::bestMatches() {
    MatrixX2i matcherID(size1, 2);
    unsigned int count(0);
    for (int i=0; i < size1; i++) {
        float maxScore(0);
        int ID(-1);
        for (int j=0; j<size2; j++)
            if (scoreMatrix(i, j) > maxScore) {
                maxScore = scoreMatrix(i, j);
                ID = j;
            }
        if (ID == -1) //no matches at all //OLIVIER
          continue;
        matcherID(count, 0) = i;
        matcherID(count++, 1) = ID;
    }
    return matcherID.topRows(count);
}

MatrixX2i Matcher::bestMatches(const float thresh) {
    MatrixX2i matcherID(size1, 2); //3
    matcherID.setZero();
    unsigned int count(0);
    for (int i = 0; i < size1; i++) {
        int ID(-1);
        float maxScore(0), secondMaxScore(0);
        if (noNei(i) == 1) continue;

        for (int j = 0; j < size2; j++) {
            if (noNej(j) == 1) continue; //NEW: do not try to match orphan nodes
            if (scoreMatrix(i, j) > maxScore) {
                secondMaxScore = maxScore;
                maxScore = scoreMatrix(i, j);
                ID = j;
            }
        }
        if (maxScore < thresh) // || ID==-1
            continue;
        matcherID(count,   0) = i;
        matcherID(count++, 1) = ID;
    }

    return matcherID.topRows(count);
}

MatrixX2i Matcher::getChangesSorted(unsigned int& count) const {
    MatrixX2i matcherID(size1, 2);
    matcherID.setZero();
    count = 0;
    unsigned int bad(size1-1);
    for (int i = 0; i < size1; i++) {
        int ID(-1);
        float maxScore(0);
        if (noNei(i) == false)
          for (int j = 0; j < size2; j++)
            if (scoreMatrix(i, j) > maxScore) {
              maxScore = scoreMatrix(i, j);
              ID = j;
            }
        if (maxScore < 0.5) {//Change detector
          matcherID(bad, 0) = i;
          matcherID(bad--, 1) = -1;
        } else {
          matcherID(count, 0) = i;
          matcherID(count++, 1) = ID;
        }
    }
    return matcherID;
}

MatrixX2i Matcher::getChanges(unsigned int& count) const {
    MatrixX2i matcherID(size1, 2);
    matcherID.setZero();
    count = 0;
    for (int i = 0; i < size1; i++) {
      matcherID(i, 0) = i;
      matcherID(i, 1) = -1;
      float maxScore(0.5); // not exactly "< 0.5"
      if (noNei(i) == false)
        for (int j = 0; j < size2; j++)
          if (scoreMatrix(i, j) > maxScore) {
            maxScore = scoreMatrix(i, j);
            matcherID(i, 1) = j;
          }
      if (matcherID(i, 1) >= 0) count++;
    }
    return matcherID;
}

MatrixX2i match(const char type,
  const Descriptor& Ds1, const Descriptor& Ds2, const int ver) {
    switch (type) {
      case WALK: {
        Matcher matcher(Ds1, Ds2); //Random walk descriptor
        return matcher.bestMatches(); //avoids the issue of low matching scores
      }
      case FAST:
      case NEWD:
      case ADJ_:
      default: {
        Matcher matcher(Ds1, Ds2, ver); //Histogram descriptor
        return matcher.bestMatches(0.75f); //0.5f
      }
    }
}

MatrixX2i changeD(const char type, const Descriptor& Ds1, const Descriptor& Ds2,
  unsigned int& count, const int ver) {
    switch (type) {
      case WALK: {
        Matcher matcher(Ds1, Ds2); //Random walk descriptor
        count = -1;
        return matcher.bestMatches(); //avoids the issue of low matching scores
      }
      case FAST:
      case NEWD:
      case ADJ_:
      default: {
        Matcher matcher(Ds1, Ds2, ver); //Histogram descriptor
        return matcher.getChanges(count);
      }
    }
}
