#include "tools.hpp"

//#include "pointCloudExtraction.hpp"  //pointCloudExtraction

#include <opencv2/highgui.hpp>  //imread, destroyAllWindows
#include <iostream>             //cout
#include <fstream>              //ifstream
#include <string>

#include "fileIO.hpp" //dataset types

using namespace cv;
using namespace std;
using namespace Eigen;

void airsim2Matrix(const double* in, double* out) {
  const double w{in[3]}, x{in[4]}, y{in[5]}, z{in[6]};
  const double xx{x*x}, yy{y*y}, zz{z*z},
               xy{x*y}, wz{w*z}, wy{w*y},
               xz{x*z}, yz{y*z}, wx{w*x};

  out[0] = 1.0f-2*(yy+zz);
  out[1] = 2*(xy-wz);
  out[2] = 2*(wy+xz);
  out[3] = in[0];

  out[4] = 2*(xy+wz);
  out[5] = 1.0f-2*(xx+zz);
  out[6] = 2*(yz-wx);
  out[7] = in[1];

  out[8] = 2*(xz-wy);
  out[9] = 2*(yz+wx);
  out[10] = 1.0f-2*(xx+yy);
  out[11] = in[2];

  out[12] = 0.0f;
  out[13] = 0.0f;
  out[14] = 0.0f;
  out[15] = 1.0f;
}

void changesim2Pose(const double* in, double* out) {
  const double x{in[3]}, y{in[4]}, z{in[5]}, w{in[6]};
  const double xx{x*x}, yy{y*y}, zz{z*z},
               xy{x*y}, wz{w*z}, wy{w*y},
               xz{x*z}, yz{y*z}, wx{w*x};

  out[0] = 1.0f-2*(yy+zz);
  out[1] = 2*(xy-wz);
  out[2] = 2*(wy+xz);
  out[3] = in[0];

  out[4] = 2*(xy+wz);
  out[5] = 1.0f-2*(xx+zz);
  out[6] = 2*(yz-wx);
  out[7] = in[1];

  out[8] = 2*(xz-wy);
  out[9] = 2*(yz+wx);
  out[10] = 1.0f-2*(xx+yy);
  out[11] = in[2];

  out[12] = 0.0f;
  out[13] = 0.0f;
  out[14] = 0.0f;
  out[15] = 1.0f;
}

void pose2Meshlab(Matrix4f pose, vector<float> camera, int w, int h) {

}

void readPoses(const char* name, const int type, Mat& pose, const int skip) {
  char path[128];
  sprintf(path, "../Dataset/%s/%s.txt", name, type2string(type));
  ifstream file(path);
  int N;
  file >> N;
  double buffer;
  if (type == AIRSIM) {
    for (int i = 0; i < skip*7; i++) file >> buffer;
    for (int i = 0; i < pose.rows; i++) {
      double* row(pose.ptr<double>(i));
      double line[7];
      for (int j = 0; j < 7; j++) file >> line[j];
      airsim2Matrix(line, row);
    }
  } else if (type == CHANGESIM) {
    for (int i = 0; i < skip*8; i++) file >> buffer;
    for (int i = 0; i < pose.rows; i++) {
      double* row(pose.ptr<double>(i));
      double line[7];
      file >> buffer;
      for (int j = 0; j < 7; j++) file >> line[j];
      changesim2Pose(line, row);
    }
  } else {
    for (int i = 0; i < skip*16; i++) file >> buffer;
    for (int i = 0; i < pose.rows; i++) {
      double* row(pose.ptr<double>(i));
      for (int j = 0; j < 16; j++) file >> row[j];
    }
  }
  file.close();
}

Matrix4f getTransformMatrix(Mat pose, int i) {
    Matrix4f frame_pose;
    unsigned int count{0};
    const double* line{pose.ptr<double>(i)};
    for (int row=0; row<4; ++row)
      for (int col=0; col<4; ++col)
        frame_pose(row, col) = line[count++];
    return frame_pose;
}

void getTransformMatrix(Mat in, int i, Matrix4f& out) {
    unsigned int count{0};
    const double* line{in.ptr<double>(i)};
    for (int row=0; row<4; ++row)
      //for (int col=0; col<4; ++col, ++count)
      for (int col=0; col<4; ++col)
        out(row, col) = line[count++];
}

string cvtype2str(int type) {
  string r;
  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);
  switch (depth) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }
  r += "C";
  r += (chans+'0');
  return r;
}

#define PREVIEW false
#define REAL_MODE true
void extractBlobs(vector<RBlob> &blobs, Mat poses, Mat Label,
  const uchar* fuse, vector<float> camera, const uchar rotated,
  const char* name, const bool esanet) {

  pointCloudExtraction Extractor(camera, rotated);
  Matrix4f frame_pose;

  const int count{poses.rows};
  char path[256];
  sprintf(path, "../Dataset/%s/depth/", name);
  const string depth_path{path};
  sprintf(path, "../Dataset/%s/%s/", name, esanet ? "esalab":"label");
  const string label_path{path};
  sprintf(path, "../Dataset/%s/changes/", name);
  const string chang_path{path};
  sprintf(path, "../Dataset/%s/instance/", name);
  const string insta_path{path};
  char base_name[32];
  unsigned short bar{0};
  bool channels; //check if labels maps have 1 or 3 channels
  for (int i = 0; i < count; i++) {
    //show progression
    if ((10*i)/count%10 == bar) cout << 10*(bar++) << "%-" << flush;

    //load the pose of the current image
    getTransformMatrix(poses, i, frame_pose);
    if (frame_pose(3,3) == 0) continue; //the matrix is incorrect

    //load the depth image and the label image
    sprintf(base_name, "%d.png", i);
    Mat depth = imread(depth_path+base_name, IMREAD_UNCHANGED);
    Mat label = imread(label_path+base_name, IMREAD_UNCHANGED);
    Mat chang = imread(chang_path+base_name, IMREAD_UNCHANGED);
    Mat insta = imread(insta_path+base_name, IMREAD_UNCHANGED);

    if (i == 0)
      channels = label.channels()==3;
    else if (channels != (label.channels()==3)) {
      cerr << "inconsistent number of channels" << endl;
      continue;
    }

    //extract 3D points
    if (!insta.empty())
      Extractor.insertValue(depth, label, insta, fuse);
    else
      Extractor.insertValue(depth, label, chang, fuse);

    Extractor.RawBlobExtraction(blobs, channels, depth.depth(),
      frame_pose, i, REAL_MODE, PREVIEW);
  }
  if (PREVIEW)
    destroyAllWindows();

  checkColors(blobs, Label.data, Label.rows, channels); //converts to indices if necessary
  cout << "100%" << endl;
}

#define COUNT_THRESH 5000
#define COVARIA false
void convertBlobs(vector<RBlob> &blobs, bool legacy, vector<SNode>& snodes,
  const int min_frame, const int max_frame, float* ari,
  const int thresh, const bool real) {

  blobsToNodes(blobs, legacy, snodes, min_frame, max_frame, COUNT_THRESH, real);
  if (!legacy) {
    //remove snodes with too few points
    const int thr{COVARIA ? 7*thresh : thresh};
    auto end = remove_if(snodes.begin(), snodes.end(),
      [thr](SNode const &snode) { return snode.size() < thr; });
    snodes.erase(end, snodes.end());
  }
  if (ari)
    *ari = adjustedRandIndex(snodes);
  if (legacy)
    stripSNodes(snodes);
}

void extractPointCloud(vector<SNode>& snodes,
  Mat poses, Mat Label, const uchar* fuse, vector<float> camera, uchar rotated,
  const char* name, bool legacy, int offset, float* ari, const int thresh) {

  pointCloudExtraction Extractor(camera, rotated);
  Matrix4f frame_pose;
  //Matrix4f old_pose;

  const int count(poses.rows);
  char path[256];
  sprintf(path, "../Dataset/%s/depth/", name);
  const string depth_path(path);
  //sprintf(path, "../Dataset/%s/esalab/", name);
  sprintf(path, "../Dataset/%s/label/", name);
  const string label_path(path);
  sprintf(path, "../Dataset/%s/changes/", name);
  const string chang_path(path);
  sprintf(path, "../Dataset/%s/instance/", name);
  const string insta_path(path);
  char base_name[32];
  unsigned short bar(0);
  bool channels; //check if labels maps have 1 or 3 channels
  for (int i = 0; i < count; i++) {
    //show progression
    if ((10*i)/count%10 == bar) cout << 10*(bar++) << "%-" << flush;

    //load the pose of the current image
    getTransformMatrix(poses, i, frame_pose);
    if (frame_pose(3,3) == 0) continue; //the matrix is incorrect

    //T speed, R speed and RMSD speed?

    /*if (i > 0) { //get speed
      //camera speed
      Vector4f cam(0,0,0,1);
      cout <<i<<'\t'<< ((frame_pose-old_pose)*Vector4f(0,0,0,1)).norm()
              <<'\t'<< ((frame_pose-old_pose)*Vector4f(0,0,1,1)).norm()
              <<'\t'<< ((frame_pose-old_pose)*Vector4f(0,0,2,1)).norm() << endl;
      //center point speed
    } else
      cout << "i\tcam_move\tclo_move\tfar_move" << endl;
    old_pose = frame_pose;*/

    //load the depth image and the label image
    sprintf(base_name, "%d.png", i+offset);
    Mat depth = imread(depth_path+base_name, IMREAD_UNCHANGED);
    Mat label = imread(label_path+base_name, IMREAD_UNCHANGED);
    Mat chang = imread(chang_path+base_name, IMREAD_UNCHANGED);
    Mat insta = imread(insta_path+base_name, IMREAD_UNCHANGED);

    if (i == 0)
      channels = label.channels()==3;
    else if (channels != (label.channels()==3)) {
      cerr << "inconsistent number of channels" << endl;
      continue;
    }

    //extract 3D points
    if (!insta.empty())
      Extractor.insertValue(depth, label, insta, fuse);
    else
      Extractor.insertValue(depth, label, chang, fuse);

    Extractor.RegionExtractionB(snodes, legacy, channels, depth.depth(),
      frame_pose, i+offset, COUNT_THRESH, COVARIA, REAL_MODE, PREVIEW);
  }
  if (PREVIEW)
    destroyAllWindows();

  if (ari)
    *ari = adjustedRandIndex(snodes);
  if (!legacy) {
    //remove snodes with too few points
    const int thr(COVARIA ? 7*thresh : thresh);
    auto end = remove_if(snodes.begin(), snodes.end(),
      [thr](SNode const &snode) { return snode.size() < thr; });
    snodes.erase(end, snodes.end());
  }

  checkColors(snodes, Label.data, Label.rows, channels); //converts to indices if necessary, before cloud creation
  cout << "100%" << endl;
}
