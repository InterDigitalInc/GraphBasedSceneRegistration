/*
Copyright © 2023 InterDigital
All Rights Reserved
This program contains proprietary information which is a trade secret/business secret of InterDigital R&D france is protected, even if unpublished, under applicable Copyright laws (including French droit d’auteur) and/or may be subject to one or more patent(s).
*/

#ifndef fileIO_hpp
#define fileIO_hpp

#include <eigen3/Eigen/Dense>  //Eigen types
#include <vector>
typedef std::vector<float> vec1D;
typedef std::vector<vec1D> vec2D;
typedef std::vector<vec2D> vec3D;

#define SYNTHIA 0
#define AIRSIM 1
#define CHANGESIM 2
//#define FOLDER 2

const char* type2string(int type);

char folder_type(const char* name);

int folder_num(const char* path);
int folder_num(const char* name, int type);

int folder_rot(const char* name, const float thres);
Eigen::Matrix4f alignTransform(const char* name);
Eigen::Matrix4f alignCorrected(const char* name, const float thres);

void  save_vec3D(const char* path, const vec3D& vec);
vec3D load_vec3D(const char* path);
void  save_vec2D(const char* path, const vec2D& vec);
vec2D load_vec2D(const char* path);

void save_edges(const char* path, const Eigen::MatrixXi edges,
  const Eigen::VectorXi labels);
Eigen::MatrixXi load_edges(const char* path, const int size);

//void save_matches(const char* path, const Eigen::MatrixXi matches);
void save_matches(const char* path, const Eigen::MatrixXi matches,
  const char* name1, const char* name2);

void save_transform(const char* path, const Eigen::Matrix4f RT);
//void save_transform(const char* path,
//  const Eigen::Matrix3f R, const Eigen::Vector3f T);
void save_transform(const char* path,
  const Eigen::Matrix3f R, const Eigen::Vector3f T,
  const char* name1, const char* name2);

void save_error(const char* path, const float t, const float r, const float e);
void save_error(const char* path, const float t, const float r, const float e,
  const char* name1, const char* name2);

#endif /* fileIO_hpp */
