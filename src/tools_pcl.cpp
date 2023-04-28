#include "tools_pcl.hpp"

#include <iostream>             //cout
#include <pcl/common/pca.h>          //principal component analysis
#include <pcl/filters/voxel_grid.h>  //VoxelGrid
#include <pcl/filters/passthrough.h> //PassThrough
#include <pcl/common/common.h>       //getMinMax3D
  //transformPointCloud
#include "utils.hpp" //Cloud types

using namespace std;
using namespace Eigen;
using namespace pcl;

// 0 is darkest
#define DARKEN(x,i) ((i)*(x)/(i+1))
// 0 is brightest
#define BRIGHTEN(x,i) ((i+1)*(x)/(i))

PointCloud<PointXYZRGB>::Ptr snodesToCloud(
  const vector<SNode>& snodes, const unsigned char colors[]) {
  PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
  for (const SNode& snode: snodes)
    for (const GNode& node: snode) {
      const int lab{(int)node[3]-1};
      const unsigned char r(DARKEN(colors[3*lab  ],2));
      const unsigned char g(DARKEN(colors[3*lab+1],2));
      const unsigned char b(DARKEN(colors[3*lab+2],2));
      PointXYZRGB point{r,g,b};
      point.x = node[0]; point.y = node[1]; point.z = node[2];
      cloud->push_back(point);
    }
  return cloud;
}

vector<SNode> simplifyNodes(const vector<SNode>& snodes, char type) {
  if (type == FULLC) //no simplification
    return snodes;

  vector<SNode> pcaNodes;
  if (type != GNODE) {
    pcaNodes.reserve(snodes.size());
    for (const SNode& snode: snodes) {
      GNode temp{snode.front()}; //lab,frame,x,y,w,h,...
      if (snode.size() >= 3) {
        PointCloud<PointXYZ> inCloud;
        for (const GNode& node : snode)
          inCloud.emplace_back(node[NODE_X], node[NODE_Y], node[NODE_Z]);

        pcl::PCA<PointXYZ> pca;
        pca.setInputCloud(inCloud.makeShared());
        const Vector4f mean = pca.getMean();
        temp[NODE_X] = mean[0];
        temp[NODE_Y] = mean[1];
        temp[NODE_Z] = mean[2];
        pcaNodes.emplace_back(SNode({temp}));
        if (type == PCA7C) {
          GNode temp2{snode.back()};
          const Matrix3f vecs(pca.getEigenVectors().array().rowwise()
                             *pca.getEigenValues().transpose().array().sqrt());
          for (int i = 0; i < 3; ++i) {
            temp2[NODE_X] = mean[0]+vecs(0,i);
            temp2[NODE_Y] = mean[1]+vecs(1,i);
            temp2[NODE_Z] = mean[2]+vecs(2,i);
            pcaNodes.back().emplace_back(temp2);
            temp2[NODE_X] = mean[0]-vecs(0,i);
            temp2[NODE_Y] = mean[1]-vecs(1,i);
            temp2[NODE_Z] = mean[2]-vecs(2,i);
            pcaNodes.back().emplace_back(temp2);
          }
        }
      } else if (snode.size() == 2 && type != PCA7C) {
        temp[NODE_X] = 0.5f*(snode.front()[NODE_X]+snode.back()[NODE_X]);
        temp[NODE_Y] = 0.5f*(snode.front()[NODE_Y]+snode.back()[NODE_Y]);
        temp[NODE_Z] = 0.5f*(snode.front()[NODE_Z]+snode.back()[NODE_Z]);
        pcaNodes.emplace_back(SNode({temp}));
      } else if (type != PCA7C)
        pcaNodes.emplace_back(SNode({temp}));
    }
  }
  return pcaNodes;
}

PointCloud<PointXYZRGB>::Ptr filterModel(PointCloud<PointXYZRGB>::Ptr input,
  const float leafSize, const int limits) {

  PointCloud<PointXYZRGB>::Ptr output(new PointCloud<PointXYZRGB>);
  // Create the filtering object: downsample the dataset using a leaf size
  VoxelGrid<PointXYZRGB> avg;
  avg.setInputCloud(input);
  avg.setLeafSize(leafSize, leafSize, leafSize);
  avg.filter(*output);

  //Filter object
  PassThrough<PointXYZRGB> filter;
  filter.setInputCloud(output);

  filter.setFilterFieldName("x");
  filter.setFilterLimits(-limits, limits);
  filter.filter(*output);

  filter.setFilterFieldName("y");
  filter.setFilterLimits(-limits, limits);
  filter.filter(*output);

  return output;
}

void filterModelinPlace(PointCloud<PointXYZRGB>::Ptr input,
  const float leafSize, const int limits) {

  // Create the filtering object: downsample the dataset using a leaf size
  VoxelGrid<PointXYZRGB> avg;
  avg.setInputCloud(input);
  avg.setLeafSize(leafSize, leafSize, leafSize);
  avg.filter(*input);

  //Filter object
  PassThrough<PointXYZRGB> filter;
  filter.setInputCloud(input);

  filter.setFilterFieldName("x");
  filter.setFilterLimits(-limits, limits);
  filter.filter(*input);

  filter.setFilterFieldName("y");
  filter.setFilterLimits(-limits, limits);
  filter.filter(*input);

  return;
}

void modelProperties(PointCloud<PointXYZRGB>::Ptr model, const Matrix4f& align,
  Vector3f& dimensions, Vector4f& center) {
  transformPointCloud(*model, *model, align); //align to axes
  PointXYZRGB minP, maxP;
  getMinMax3D(*model, minP, maxP);
  dimensions <<0.5f*(maxP.x-minP.x), 0.5f*(maxP.y-minP.y), 0.5f*(maxP.z-minP.z);
  center <<0.5f*(maxP.x+minP.x), 0.5f*(maxP.y+minP.y), 0.5f*(maxP.z+minP.z), 1;
}
