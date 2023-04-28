#include "fileIO.hpp"

#include <fstream>  //ifstream, ofstream
#include <iostream> //cerr

using namespace std;
using namespace Eigen;

const char* type2string(int type) {
  switch (type) {
    case SYNTHIA:
    default:
      return "synthia";
    case AIRSIM:
      return "airsim";
    case CHANGESIM:
      return "trajectory";
  }
}

char folder_type(const char* name) {
  char path[128];
  /*sprintf(path,"../Dataset/%s/pose/0.txt", name);
  ifstream file;
  file.open(path);
  if (file.good()) return FOLDER;*/
  sprintf(path,"../Dataset/%s/synthia.txt", name);
  ifstream file;
  file.open(path);
  if (file.good()) return SYNTHIA;
  if (file.is_open()) file.close();
  sprintf(path,"../Dataset/%s/airsim.txt", name);
  file.open(path);
  if (file.good()) return AIRSIM;
  if (file.is_open()) file.close();
  sprintf(path,"../Dataset/%s/trajectory.txt", name);
  file.open(path);
  if (file.good()) return CHANGESIM;
  if (file.is_open()) file.close();
  cerr << "Unrecognized dataset type for " << name << endl;
  return -1;
}

int folder_num(const char* path) {
  ifstream file(path);
  if (not(file.is_open()))
    return 0;
  int value;
  file >> value;
  file.close();
  return value;
}

int folder_num(const char* name, int type) {
  char path[128];
  //sprintf(path, "../Dataset/%s/%s.txt", name, type==AIRSIM ?"airsim":"synthia");
  sprintf(path, "../Dataset/%s/%s.txt", name, type2string(type));
  return folder_num(path);
}

int folder_rot(const char* name, const float thres) {
  char path[128];
  sprintf(path, "../Dataset/%s/orientation.txt", name);
  ifstream file(path);
  if (not(file.is_open()))
    return 0;
  int value;
  file >> value;
  float err;
  file >> err;
  file.close();
  if (err > thres)
    return -1;
  return value;
}

Matrix4f alignTransform(const char* name) {
  char path[1024];
  sprintf(path, "../Dataset/%s/%s.txt", name, name);
  ifstream file(path);
  if (not(file.is_open()))
    return Matrix4f::Identity();
    string buffer;
  do file >> buffer; while (buffer != "axisAlignment");
  file >> buffer; // "="
  Matrix4f transform;
  for (int i = 0; i < 16; i++) file >> transform(i);
  file.close();
  return transform.transpose();
}

Matrix4f alignCorrected(const char* name, const float thres) {
  Matrix4f R_90;  R_90  <<  0,-1,0,0,  1, 0,0,0, 0,0,1,0, 0,0,0,1;
  Matrix4f R_180; R_180 << -1, 0,0,0,  0,-1,0,0, 0,0,1,0, 0,0,0,1;
  Matrix4f R_270; R_270 <<  0, 1,0,0, -1, 0,0,0, 0,0,1,0, 0,0,0,1;
  const Matrix4f A = alignTransform(name);
  const int rot = folder_rot(name, thres);
  cout << name << "_rot(°)\t" << rot << endl;
  switch (rot) {
    case 90:
    return R_90*A;
    case 180:
    return R_180*A;
    case 270:
    return R_270*A;
    case 0:
    default:
    return A;
    case -1:
    return Matrix4f::Zero();
  }
}

void  save_vec3D(const char* path, const vec3D& vec) {
  ofstream file(path);
  if (not(file.is_open())) {
    cerr << "Could not save supernodes to " << path << endl;
    return;
  }
  file << vec.size() << '\n'; //number of vec2D
  for (const vec2D &ve : vec) {
    file << '\n' << ve.size() << '\n'; //number of vec1D
    for (const vec1D &v : ve) {
      file << v.size() << ' '; //size of vec1D
      for (const float &x : v) file << ' ' << x;
      file << '\n';
    }
  }
  file.close();
}

vec3D load_vec3D(const char* path) {
  vec3D vec;
  ifstream file(path);
  if (not(file.is_open()))
    return vec;
  size_t n;
  file >> n; //number of vec2D
  vec.reserve(n);
  for (size_t k = 0; k < n; k++) {
    size_t m;
    file >> m; //number of vec1D
    vec2D ve;
    ve.reserve(m);
    for (size_t i = 0; i < m; i++) {
      size_t l;
      file >> l; //size of vec1D
      ve.emplace_back(l); //create a vec1D of size l at the back of the vec2D
      for (size_t j = 0; j < l; j++) file >> ve.back()[j];
    }
    vec.push_back(ve);
  }
  file.close();
  return vec;
}

void  save_vec2D(const char* path, const vec2D& vec) {
  ofstream file(path);
  if (not(file.is_open())) {
    cerr << "Could not save blobs to " << path << endl;
    return;
  }
  file << vec.size() << '\n'; //number of vec1D
  for (const vec1D &ve : vec) {
    file << ve.size() << ' '; //size of vec1D
    for (const float &x : ve) file << ' ' << x;
    file << '\n';
  }
  file.close();
}

vec2D load_vec2D(const char* path) {
  vec2D vec;
  ifstream file(path);
  if (file.is_open()) {
    size_t n;
    file >> n; //number of vec1D
    vec.reserve(n);
    for (size_t k = 0; k < n; k++) {
      size_t l;
      file >> l; //size of vec1D
      vec.emplace_back(l); //create a vec1D of size l at the back of the vec2D
      for (size_t j = 0; j < l; j++) file >> vec.back()[j];
    }
    file.close();
  }
  return vec;
}

void save_edges(const char* path, const MatrixXi edges, const VectorXi labels) {
  ofstream file(path);
  if (not(file.is_open())) {
    cerr << "Could not save edges to " << path << endl;
    return;
  }
  file << edges << '\n' << labels << flush;
  file.close();
}

MatrixXi load_edges(const char* path, const int size) {
  MatrixXi raw;
  ifstream file(path);
  if (not(file.is_open()))
    return raw;
  raw = MatrixXi::Zero(size, size+1);
  for (int i = 0; i < (size+1)*size; i++) file >> raw(i);
  file.close();
  return raw.transpose();
}

void save_matches(const char* path, const MatrixXi matches) {
  ofstream file(path);
  if (not(file.is_open())) {
    cerr << "Could not save matches to " << path << endl;
    return;
  }
  file << matches;
  file.close();
}

void save_matches(const char* path, const MatrixXi matches,
  const char* name1, const char* name2) {
  ofstream file(path, ios_base::app);
  if (not(file.is_open())) {
    cerr << "Could not save matches to " << path << endl;
    return;
  }
  file << name1 << '-' << name2 << endl;
  file << matches.transpose() << endl;
  file.close();
}

void save_transform(const char* path, const Matrix4f RT) {
  ofstream file(path);
  if (not(file.is_open())) {
    cerr << "Could not save transform to " << path << endl;
    return;
  }
  file << RT;
  file.close();
}

void save_transform(const char* path, const Matrix3f R, const Vector3f T) {
  Matrix4f RT = Matrix4f::Identity();
  RT.block<3,3>(0,0) = R;
  RT.block<3,1>(0,3) = T;
  save_transform(path, RT);
}

void save_transform(const char* path, const Matrix4f RT,
  const char* name1, const char* name2) {
  ofstream file(path, ios_base::app);
  if (not(file.is_open())) {
    cerr << "Could not save transform to " << path << endl;
    return;
  }
  file << name1 << '-' << name2 << endl;
  file << RT << endl;
  file.close();
}

void save_transform(const char* path, const Matrix3f R, const Vector3f T,
  const char* name1, const char* name2) {
  Matrix4f RT = Matrix4f::Identity();
  RT.block<3,3>(0,0) = R;
  RT.block<3,1>(0,3) = T;
  save_transform(path, RT, name1, name2);
}

void save_error(const char* path, const float t, const float r, const float e) {
  ofstream file(path);
  if (not(file.is_open())) {
    cerr << "Could not save error metrics to " << path << endl;
    return;
  }
  file << "translation:  \t" << t << endl;
  file << "rotation:     \t" << r << endl;
  file << "RMS deviation:\t" << e;
  file.close();
}

void save_error(const char* path, const float t, const float r, const float e,
  const char* name1, const char* name2) {
  ofstream file(path, ios_base::app);
  if (not(file.is_open())) {
    cerr << "Could not save error metrics to " << path << endl;
    return;
  }
  file << name1 << '-' << name2
       << "\ttranslation: "   << t
       << "\trotation: "      << r
       << "\tRMSD: " << e << endl;
  file.close();
}
