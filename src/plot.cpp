#include "plot.hpp"
#include <pcl/visualization/pcl_visualizer.h>   //visualization::
#include <pcl/common/transforms.h>              //transformPointCloud

#include <boost/make_shared.hpp>  //boost::shared_ptr

#include <thread>

//#include <iostream> //cout
//using namespace std;//cout

using namespace cv;
using namespace pcl;
using namespace Eigen;

using namespace std::chrono_literals;
typedef Array<float,1,4> RowArray4f;

#define NODE_SIZE 25
#define POINT_SIZE 2 //3
#define DOT_SIZE 1
#define Z_OFFSET 10 //40//80
#define DEFAULT_Z 8

Matrix4f transformMatrix(Matrix3f R, Vector3f T, const float offset=0) {
  Matrix4f RT = Matrix4f::Identity();
  for (int row = 0; row<3; row++) //OLIVIER1
    for (int col = 0; col<3; col++)
      RT(row, col) = R(row, col);
   RT(0, 3) = T(0, 0);
   RT(1, 3) = T(1, 0);
   RT(2, 3) = T(2, 0)+offset;
   return RT;
}

visualization::PCLVisualizer::Ptr makeViewer(
  const PointCloud<PointXYZRGB>::Ptr& cloud1, MatrixXi edges1, const int psize1,
  const PointCloud<PointXYZRGB>::Ptr& cloud2, MatrixXi edges2, const int psize2,
  MatrixX2i matches, const int ctype=MEANC,
  const Matrix3f* R=nullptr, const Vector3f* T=nullptr, const float offset=0,
  const Matrix4f* A=nullptr, const float color=0, bool axis=true) {

  PointCloud<PointXYZRGB>::Ptr align1;
  if (R && T) { //TODO: create if only for offset!=0
    align1 = boost::make_shared<PointCloud<PointXYZRGB>>();
    Matrix4f RTo = transformMatrix(*R,*T,offset);
    transformPointCloud(*cloud1, *align1, RTo);
  } else if (A) {
    align1 = boost::make_shared<PointCloud<PointXYZRGB>>();
    *align1 = *cloud1;
  } else
    align1 = cloud1;

  PointCloud<PointXYZRGB>::Ptr align2;
  if (A) {
    align2 = boost::make_shared<PointCloud<PointXYZRGB>>();
    transformPointCloud(*align1, *align1, *A);
    transformPointCloud(*cloud2, *align2, *A);
  } else
    align2 = cloud2;

  visualization::PCLVisualizer::Ptr viewer(new visualization::PCLVisualizer("point cloud"));
  viewer->setBackgroundColor(color, color, color);

  //Plot clouds
  if (psize1 > 0) {
    visualization::PointCloudColorHandlerRGBField<PointXYZRGB> rgb(align1);
    viewer->addPointCloud<PointXYZRGB>(align1, rgb, "Point cloud 1");
    viewer->setPointCloudRenderingProperties(
      visualization::PCL_VISUALIZER_POINT_SIZE, psize1, "Point cloud 1");
  }
  if (psize2 > 0) {
    visualization::PointCloudColorHandlerRGBField<PointXYZRGB> rgb(align2);
    viewer->addPointCloud<PointXYZRGB>(align2, rgb, "Point cloud 2");
    viewer->setPointCloudRenderingProperties(
      visualization::PCL_VISUALIZER_POINT_SIZE, psize2, "Point cloud 2");
  }

  if (axis)
    viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
  viewer->setCameraPosition(0, 0, DEFAULT_Z, 0, 0, 1, 0, -1, 0);

  //Plot edges
  for (int i = 0; i < edges1.rows(); ++i)
    for (int j=i+1; j < edges1.cols(); ++j)
      if (edges1(i,j))
        switch (ctype) {
          case FULLC:
          break; //not enough information
          case MEANC:
          case GNODE:
          default:
          viewer->addLine<PointXYZRGB>(align1->points[i], align1->points[j],
            (align1->points[i].r+align1->points[j].r)/510.0f,
            (align1->points[i].g+align1->points[j].g)/510.0f,
            (align1->points[i].b+align1->points[j].b)/510.0f,
            std::__cxx11::to_string(j+align1->size()*i)+"-edges1");
          break;
          case PCA7C:
          viewer->addLine<PointXYZRGB>(align1->points[7*i], align1->points[7*j],
            (align1->points[7*i].r+align1->points[7*j].r)/510.0f,
            (align1->points[7*i].g+align1->points[7*j].g)/510.0f,
            (align1->points[7*i].b+align1->points[7*j].b)/510.0f,
            std::__cxx11::to_string(j+align1->size()*i)+"-edges1");
          break;
        }

  for (int i = 0; i < edges2.rows(); ++i)
    for (int j=i+1; j < edges2.cols(); ++j)
      if (edges2(i,j))
        switch (ctype) {
          case FULLC:
          break; //not enough information
          case MEANC:
          case GNODE:
          default:
          viewer->addLine<PointXYZRGB>(align2->points[i], align2->points[j],
            (align2->points[i].r+align2->points[j].r)/510.0f,
            (align2->points[i].g+align2->points[j].g)/510.0f,
            (align2->points[i].b+align2->points[j].b)/510.0f,
            std::__cxx11::to_string(j+align2->size()*i)+"-edges2");
          break;
          case PCA7C:
          viewer->addLine<PointXYZRGB>(align2->points[7*i], align2->points[7*j],
            (align2->points[7*i].r+align2->points[7*j].r)/510.0f,
            (align2->points[7*i].g+align2->points[7*j].g)/510.0f,
            (align2->points[7*i].b+align2->points[7*j].b)/510.0f,
            std::__cxx11::to_string(j+align2->size()*i)+"-edges2");
          break;
        }

  //Plot matches
  for (int i = 0; i < matches.rows(); ++i) {
    viewer->addLine<PointXYZRGB>(
      align1->points[matches(i, 0)], align2->points[matches(i, 1)],
      align1->points[matches(i, 0)].r/255.0f,//170.0f,
      align1->points[matches(i, 0)].g/255.0f,//170.0f,
      align1->points[matches(i, 0)].b/255.0f,//170.0f,
      std::__cxx11::to_string(i)+"-matches");
  }

  return viewer;
}

void view(visualization::PCLVisualizer::Ptr viewer,
  const Matrix3f* P=nullptr, const Matrix4f* M=nullptr,
  const Matrix4f* A=nullptr, const char* name=nullptr) {

  if (P && M) {
    Matrix4f N(M->array().rowwise()*RowArray4f(-1,-1,1,1)); //vertical flip
    if (A) N = (*A)*N;
    viewer->setCameraParameters(*P, N);
  }

  if (name) {
    char path[128];
    sprintf(path, "images/%s.png", name);
    viewer->saveScreenshot(path);
  }

  while(!viewer->wasStopped()) {
    viewer->spinOnce(17);
    std::this_thread::sleep_for(17ms);
  }
}

void view(visualization::PCLVisualizer::Ptr viewer,
  const Matrix3f* P, Mat poses,
  const Matrix4f* A=nullptr, const char* name=nullptr) {

  viewer->setShowFPS(false);
  const RowArray4f vflip(-1,-1,1,1); //vertical flip

  int i(-1);
  while(!viewer->wasStopped() && i < poses.rows-1) {
    Matrix4f N(Map<Matrix<double,4,4,RowMajor>>(
      poses.ptr<double>(++i)).cast<float>().array().rowwise()*vflip);
    if (A) N = (*A)*N;
    viewer->setCameraParameters(*P, N);
    viewer->spinOnce(17);
    std::this_thread::sleep_for(17ms);
    if (name) {
      char path[128];
      sprintf(path, "images/%s-%04d.png", name, i);
      viewer->saveScreenshot(path);
    }
  }
}

void visualizeGG(
  const PointCloud<PointXYZRGB>::Ptr& cloud1, MatrixXi edges1,
  const PointCloud<PointXYZRGB>::Ptr& cloud2, MatrixXi edges2, const int ctype,
  MatrixX2i matches, Matrix4f A, Matrix3f R, Vector3f T, const char* name) {

  visualization::PCLVisualizer::Ptr viewer(makeViewer(
    cloud1, edges1, NODE_SIZE, cloud2, edges2, NODE_SIZE, matches,
    ctype, &R, &T, Z_OFFSET, &A, 1.0f));
  view(viewer, nullptr, nullptr, &A, name);
}

void visualizeGM(
  const PointCloud<PointXYZRGB>::Ptr& cloud1, MatrixXi edges, const int ctype,
  const PointCloud<PointXYZRGB>::Ptr& cloud2, Matrix3f P, Matrix4f M,
  Matrix4f A, Matrix3f R, Vector3f T, const char* name) {

  visualization::PCLVisualizer::Ptr viewer(makeViewer(
    cloud1, edges, NODE_SIZE, cloud2, MatrixXi(), POINT_SIZE, MatrixX2i(),
    ctype, &R, &T, 0, &A, 1.0f));
  view(viewer, &P, &M, &A, name);
}

void visualizeMM(
  const PointCloud<PointXYZRGB>::Ptr& cloud1,
  const PointCloud<PointXYZRGB>::Ptr& cloud2,
  Matrix4f A, Matrix3f R, Vector3f T, const char* name) {

  visualization::PCLVisualizer::Ptr viewer(makeViewer(
    cloud1, MatrixXi(), DOT_SIZE, cloud2, MatrixXi(), DOT_SIZE, MatrixX2i(),
    MEANC, &R, &T, 0, &A, 0.0f));
  view(viewer, nullptr, nullptr, &A, name);
}

void visualizeAM(const PointCloud<PointXYZRGB>::Ptr& cloud,
  Matrix3f P, Mat poses, const char* name) {

  visualization::PCLVisualizer::Ptr viewer(makeViewer(
    cloud, MatrixXi(), 2, cloud, MatrixXi(), 0, MatrixX2i(),
    MEANC, nullptr, nullptr, 0, nullptr, 0.5));
  view(viewer, &P, poses, nullptr, name);
}

void visualize2D(const PointCloud<PointXYZRGB>::Ptr& cloud,
  MatrixXi edges, const int ctype, Matrix4f A, const char* name) {

  //Matrix4f trans = transformMatrix(Matrix3f::Identity(), A.block<3,1>(0,3));
  visualization::PCLVisualizer::Ptr viewer(makeViewer(
    cloud, edges, NODE_SIZE, (cloud), MatrixXi(), 0, MatrixX2i(),
    ctype, nullptr, nullptr, 0, &A, 1.0f, false)); //&trans
  viewer->setSize(800,800);
  view(viewer, nullptr, nullptr, &A, name); //TODO: add z to name?
}
