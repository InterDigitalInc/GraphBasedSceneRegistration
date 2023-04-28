#include "tools.hpp"
#include "tools_pcl.hpp"
#include <string>
#include "matcher.hpp"
  //#include "descriptor.hpp"
#include "registration.hpp"

#include <pcl/io/pcd_io.h> //save point cloud
#include <pcl/io/ply_io.h> //save point cloud

#include <iostream>
#include <chrono>

//#include <opencv2/highgui.hpp> //imshow
//#include <opencv2/imgproc.hpp> //resize, circle

#include "fileIO.hpp"
#include "plot.hpp"
#include "utils.hpp"

#include "airsim.h"
#include "changesim.h"
#include "scannet.h"
//#include "scene0005.h"

#define NOW() chrono::high_resolution_clock::now()

using namespace cv;
using namespace std;
using namespace Eigen;
using namespace pcl;

#define USE_ESANET false
Blobs computeBlobs(const char* name, Mat poses,
  Mat Label, const uchar* fuse, vector<float> camera, uchar rotated,
  bool saving=true, bool force=false) { //we should not have to redo the blobs

  char path[128];
  sprintf(path, USE_ESANET ? "blobs-ESANet/%s.txt" : "blobs/%s.txt", name);
  Blobs blobs;
  if (!force)
   blobs = load_vec2D(path);
  if (blobs.empty()) {
    extractBlobs(blobs, poses, Label, fuse, camera, rotated, name, USE_ESANET);
    if (saving)
      save_vec2D(path, blobs);
  }
  return blobs;
}

#define NODE_THRESH 10
#define USE_INSTANCE_GT false
vector<SNode> computeNodes(const char* name,
  PointCloud<PointXYZRGB>::Ptr& cloud, Mat poses, Mat Label, const uchar* fuse,
  vector<float> camera, uchar rotated, bool saving, bool force,
  int min_frame=0, int max_frame=-1, char type=FULLC, bool ply=false) {

  char path[128];
  sprintf(path, "snodes/%s_%d_S_%c.pcd", name, min_frame, ctype2char(type)); //nodes
  string file{path};
  sprintf(path, "snodes/%s_%d_S_%c.txt", name, min_frame, ctype2char(type)); //nodes
  vector<SNode> snodes;
  //do not load the nodes if we have to remake them or if the cloud is not there
  if (!force && io::loadPCDFile<PointXYZRGB>(file, *cloud) != -1)
    snodes = load_vec3D(path);

  bool convert{false}; //faster but slightly less accurate than extraction
  if ((type == MEANC || type == PCA7C) && snodes.empty()) {
    sprintf(path, "snodes/%s_%d_S_%c.pcd", name, min_frame, ctype2char(FULLC));
    file.assign(path);
    //do not load the nodes if the cloud is not there
    if (io::loadPCDFile<PointXYZRGB>(file, *cloud) != -1) {
      sprintf(path, "snodes/%s_%d_S_%c.txt", name, min_frame, ctype2char(FULLC));
      snodes = load_vec3D(path);
      convert = !snodes.empty();
    }
  }

  if (convert || snodes.empty()) {
      auto start = NOW();
      if (convert) { //this is faster than starting from the blobs (file load)
        snodes = simplifyNodes(snodes, type);
        cout << "adjusted_Rand_Idx\t" << "ukn" << endl;
      } else {
        snodes.clear(); //ensure coherency
        //create and use blobs if necessary
        Blobs blobs{computeBlobs(name,poses,Label,fuse,camera,rotated)};
        float ari;
        convertBlobs(blobs, type==GNODE, snodes, min_frame, max_frame, &ari,
          NODE_THRESH, !USE_INSTANCE_GT);
        if (ari > -2)
          cout << "adjusted_Rand_Idx\t" << ari << endl;
        else
          cout << "adjusted_Rand_Idx\t" << "ukn" << endl;
        if (type == MEANC || type == PCA7C)
          snodes = simplifyNodes(snodes, type);
      }
      cloud = snodesToCloud(snodes, Label.data);
      chrono::duration<double, milli> time(NOW()-start);
      cout << name << "_snodes(ms)\t" << time.count() << endl;
      if (saving) { //save to the correct paths
        sprintf(path, "snodes/%s_%d_S_%c.pcd", name, min_frame, ctype2char(type));
        file.assign(path);
        sprintf(path, "snodes/%s_%d_S_%c.txt", name, min_frame, ctype2char(type));
        save_vec3D(path, snodes);
        io::savePCDFileBinary(file, *cloud);
      }
  } else
      cout << "LOADED_" << name << "_SNODES" << endl;

  if (ply) { //for meshlab
    sprintf(path,"snodes/%s_%d_S_%c.ply", name, min_frame, ctype2char(type));
    file.assign(path);
    io::savePLYFileASCII(file, *cloud);
  }
  return snodes;
}

/*Topology computeTopology(const char* name,
  const vector<SNode> &snodes, float thresh, bool saving, bool force) {
  char path[128];
  sprintf(path, "edges/%s-%.1f.txt", name, thresh);

  const MatrixXi edges = load_edges(path, snodes.size());
  if (force || edges.size() == 0) {
    if (thresh > 0) {
      auto start = NOW();
      Topology Top(snodes, thresh);
      chrono::duration<double, milli> time(NOW()-start);
      cout << name << "_edges(ms)\t" << time.count() << endl;
      if (saving)
        save_edges(path, Top.getEdges(), Top.getLabels());
      return Top;
    } else {
      auto start = NOW();
      Topology Top(snodes);
      chrono::duration<double, milli> time(NOW()-start);
      cout << name << "_edges(ms)\t" << time.count() << endl;
      //if(saving)
      //  save_edges(path, Top.getEdges(), Top.getLabels()); //no saving for auto
      return Top;
    }
  } else
      cout << "LOADED_" << name << "_EDGES" << endl;

  Topology Top(edges, thresh);
  return Top;
}*/

/*Topology computeTopology(const char* name,
  const vector<SNode> &snodes, float thresh, bool saving, bool force) {
  auto start = NOW();
  Topology Top(snodes);
  chrono::duration<double, milli> time(NOW()-start);
  cout << name << "_edges(ms)\t" << time.count() << endl;
  return Top;
}*/

Matrix3f cameraToIntrinsics(const vector<float> &camera) {
  Matrix3f intrinsics;
  intrinsics << camera[0],         0, camera[2],
                        0, camera[1], camera[3],
                        0,         0,         1;
  return intrinsics;
}

// void printNumpyMat(Matrix4f mat) {
//   cout << "np.matrix(\'\\\n"
//     << mat.row(0) << ";\\\n" << mat.row(1) << ";\\\n"
//     << mat.row(2) << ";\\\n" << mat.row(3) << "\')" << endl;
// }

void printNumpyMat(Matrix4f mat, const char prefix[] = "") {
  cout << prefix << "np.matrix(\'\\\n"
    << mat.row(0) << ";\\\n" << mat.row(1) << ";\\\n"
    << mat.row(2) << ";\\\n" << mat.row(3) << "\')" << endl;
}


//A change is an unmatched node, we can only check the unmatched nodes in the query graph (name2)
float changeIoU(const char* name, GNode node, const uchar* fuse,
  const Mat& colors, const int width=640) {
  const int label((int)node[3]-1);
  const int frame((int)node[4]);
  //cout << name << ',' << frame << ','
  const uchar r(colors.ptr(label)[0]),
              g(colors.ptr(label)[1]),
              b(colors.ptr(label)[2]);
  Mat labelMap = loadImage(name, frame, width, fuse, LABELMAP);
  Mat changMap = loadImage(name, frame, width, fuse, CHANGMAP);
  //imshow("labelMap", labelMap);
  inRange(labelMap, Scalar(b,g,r), Scalar(b,g,r), labelMap);

  cv::Mat channels[3];
  split(changMap, channels);
  bitwise_and(channels[2], labelMap!=0, changMap);
  // imshow("labelMapFiltered", labelMap);
  // imshow("changeMap", channels[2]);
  // imshow("changeMapFiltered", changMap);
  // waitKey(0);
  uchar* seg = (uchar*) changMap.data; //direct access to image
  const int cols(changMap.cols);
  const int rows(changMap.rows);
  if (countNonZero(changMap)) {
    //cout << frame << ':' << label << '=' << (int)r << ' ' << (int)g << ' ' << (int)b << '\n';
    //cout << countNonZero(labelMap) << '/';
    //cout << countNonZero(changMap) << '/' << labelMap.rows*labelMap.cols << endl;
    float maxIoU(0);
    for (int i = cols; i < cols*(rows-1); i++) { //ignore first/last rows
      const int lab(seg[i]);//seg[3*i+2]%13%8);
      if (lab) {
        stack<int> neighbors;
        neighbors.push(i); //seed
        int minU{rows}, maxU{0}, minV{cols}, maxV{0};
        do { //4-neighbor spread
          const int j(neighbors.top());
          neighbors.pop();
          seg[j] = 0;
          if (seg[j-1]    == lab) neighbors.push(j-1);
          if (seg[j+1]    == lab) neighbors.push(j+1);
          if (seg[j-cols] == lab) neighbors.push(j-cols);
          if (seg[j+cols] == lab) neighbors.push(j+cols);
          const int v(j%cols);
          const int u((j-v)/cols);
          if (v > maxV) maxV = v;
          if (v < minV) minV = v;
          if (u > maxU) maxU = u;
          if (u < minU) minU = u;
        } while (!neighbors.empty());
        if ((maxV-minV+1)*(maxU-minU+1) < 36) //too small
          continue;
        //cout << '\t' << lab%13%8 << ':' << minV << ' ' << minU << ' ' << maxV-minV+1 << ' ' << maxU-minU+1 << endl;
        float xA{std::max((float)minV, node[5])},
      	      yA{std::max((float)minU, node[6])},
      	      xB{std::min((float)maxV, node[5]+node[7]-1)},
      	      yB{std::min((float)maxU, node[6]+node[8]-1)};
      	float interArea{std::max(0.0f, xB-xA+1)*std::max(0.0f, yB-yA+1)};
      	float ioU{interArea/(node[7]*node[8]+(maxV-minV+1)*(maxU-minU+1)-interArea)};
        if (ioU > maxIoU)
          maxIoU = ioU;
        //cout << "ioU: " << ioU << endl;
      }
    }
    return maxIoU;
  }
  return 0;
}

int main(int argc, const char* argv[]) {
    int argC(3);
    if (argc < argC)
      cout << "usage: ./sgRegistration folder1 folder2 [dtype depth ctype task rtype \
saving start1 count1 start2 count2 suffix]" << endl;
    const char* name1{argv[1]};
    const char* name2{argv[2]};
    const char ptype1{folder_type(name1)};
    const char ptype2{folder_type(name2)};
    const  int  pnum1{folder_num(name1, ptype1)};
    const  int  pnum2{folder_num(name2, ptype2)};
    if (ptype1 < 0 || ptype2 < 0) return -1;
    const char  dtype{argc > argC ? char2dtype(argv[argC++][0]) : (char)ADJ_};
    const  int  depth{argc > argC ?       atoi(argv[argC++])    : 3};
    const char  ctype{argc > argC ? char2ctype(argv[argC++][0]) : (char)FULLC};
    const char   task{argc > argC ?  char2task(argv[argC++][0]) : (char)NOPLOT};
    const char  rtype{argc > argC ? char2rtype(argv[argC++][0]) : (char)ONEPERMN};
    const bool saving{argc > argC ? (bool)atoi(argv[argC++])    : false};
    const  int start1{argc > argC ?       atoi(argv[argC++])    : 0};
    const  int count1{argc > argC ?      (atoi(argv[argC++])> 0 ?
      atoi(argv[argC-1]) : pnum1-start1) : pnum1-start1};//to the end by default
    const  int start2{argc > argC ?       atoi(argv[argC++])    : 0};
    const  int count2{argc > argC ?      (atoi(argv[argC++])> 0 ?
      atoi(argv[argC-1]) : pnum2-start2) : pnum2-start2};
    if (count1 <= 0 || count2 <= 0) return -1;
    const char* suffix{argc > argC ? argv[argC++] : ""};

    //Prepare the chronometer
    chrono::duration<double, milli> exec_time;
    auto mid_time = NOW();
    const auto start_time = mid_time;

    //Prepare the viewer
    char filepath[1024];
    console::setVerbosityLevel(console::L_ALWAYS); //make pcl quiet

    char filename[32];
    sprintf(filename, "%c-%c-%c-%d_%s.txt", ctype2char(ctype), rtype2char(rtype), dtype2char(dtype), depth, suffix);

    const Matrix4f A1 = alignCorrected(name1, 0.2);
    const Matrix4f A2 = alignCorrected(name2, 0.2);
    if (A1.isZero(0) || A2.isZero(0)) return -1;
    const Matrix4f A1_inv = A1.inverse();
    const Matrix4f A2_inv = A2.inverse();

    if (task == TESTGT) return 0;

    //2 is reference/target
    const Matrix4f GT = A2_inv*A1;
    if (saving) {
      sprintf(filepath, "ground_truth/%s-%s.txt", name1, name2);
      save_transform(filepath, GT);
    }
    const Matrix3f R_gt = GT.block<3,3>(0,0);
    const Vector3f T_gt = GT.block<3,1>(0,3);

    if (ptype1 == CHANGESIM)
      sprintf(filepath, "../Dataset/%s/cloud_map01.ply", name1);
    else
      sprintf(filepath, "../Dataset/%s/%s_vh_clean_2.labels.ply", name1, name1);
    const string ply1(filepath);
    if (ptype2 == CHANGESIM)
      sprintf(filepath, "../Dataset/%s/cloud_map01.ply", name2);
    else
      sprintf(filepath, "../Dataset/%s/%s_vh_clean_2.labels.ply", name2, name2);
    const string ply2(filepath);

    if (task == GTRUTH) {
      PointCloud<PointXYZRGB>::Ptr model1(new PointCloud<PointXYZRGB>);
      io::loadPLYFile(ply1, *model1);
      PointCloud<PointXYZRGB>::Ptr model2(new PointCloud<PointXYZRGB>);
      io::loadPLYFile(ply2, *model2);
      visualizeMM(model1, model2, A2, R_gt, T_gt);
      return 0;
    }

    //Camera parameters and poses
    uchar rotated;
    vector<float> camera(5);
    if (ptype1 == CHANGESIM) {
      camera[0] = 320; //fx
      camera[1] = 320; //fy
      camera[2] = 320; //cx
      camera[3] = 240; //cy
      camera[4] = 1;   //s
      rotated = ROTATE_NONE;
    } else if (ptype1 == AIRSIM) {
      camera[0] = 512; //fx
      camera[1] = 512; //fy
      camera[2] = 512; //cx
      camera[3] = 288; //cy
      camera[4] = 1;   //s
      rotated = ROTATE_NONE;
    } else {
      if (name1[0] == 's') {
        //ScanNet camera
        camera[0] = 577.870605; //fx
        camera[1] = 577.870605; //fy
        camera[2] = 319.500000; //cx
        camera[3] = 239.500000; //cy
        camera[4] = 1;   //s
        rotated = ROTATE_NONE;
      } else {
        //3RScan camera
        camera[0] = 760.052; //fx
        camera[1] = 758.792; //fy
        camera[2] = 485.694; //cx
        camera[3] = 266.208; //cy
        camera[4] = 1;   //s
        rotated = ROTATE_90_CLOCKWISE;
      }
    }

    Mat pose1 = Mat::zeros(count1, 16, CV_64FC1);
    Mat pose2 = Mat::zeros(count2, 16, CV_64FC1);
    readPoses(name1, ptype1, pose1, start1);
    readPoses(name2, ptype2, pose2, start2);

    if (task == ANIMAT) {
      PointCloud<PointXYZRGB>::Ptr model1(new PointCloud<PointXYZRGB>);
      io::loadPLYFile(ply1, *model1);

      const Matrix3f intrinsics = cameraToIntrinsics(camera);
      mid_time = NOW();
      visualizeAM(model1, intrinsics, pose1, "animation/Seq_0");
      exec_time = NOW()-mid_time;
      cout << "animation_time(ms)\t" << exec_time.count() << endl;
      cout << "number_of_frames1\t" << count1 << endl;
      return 0;
    }

    //Prepare dataset labels
    Mat Label;
    const uchar* fuse = nullptr;
    if (ptype1 == CHANGESIM)
      Label = Label_changesim;
    else if (ptype1 == AIRSIM)
      Label = Label_airsim;
    else {
      Label = Label_scannet;
      if (name1[0] == 's')
        fuse = USE_ESANET ? nyu41_to_nyu40 : id_to_nyu40;
    }

    const int labels(Label.rows);
    cout << "number_of_labels\t" << labels << endl;

    //Only extract the blobs
    if (task == RBLOBS) {
      cout << "number_of_frames1\t" << count1 << endl;
      mid_time = NOW();
      Blobs blobs1 = computeBlobs(name1, pose1, Label, fuse,
        camera, rotated);
      exec_time = NOW()-mid_time;
      cout << "rblobs1__time(ms)\t" << exec_time.count() << endl;
      cout << "number_of_rblobs1\t" << blobs1.size() << endl;

      cout << "number_of_frames2\t" << count2 << endl;
      mid_time = NOW();
      Blobs blobs2 = computeBlobs(name2, pose2, Label, fuse,
        camera, rotated);
      exec_time = NOW()-mid_time;
      cout << "rblobs2__time(ms)\t" << exec_time.count() << endl;
      cout << "number_of_rblobs2\t" << blobs2.size() << endl;

      return 0;
    }

    //Extract point clouds
    cout << "number_of_frames1\t" << count1 << endl;
    PointCloud<PointXYZRGB>::Ptr cloud1(new PointCloud<PointXYZRGB>);
    mid_time = NOW();
    vector<SNode> snodes1 = computeNodes(name1, cloud1, pose1, Label, fuse,
      camera, rotated, saving, task==FORCED, start1, -1, ctype, false);
    exec_time = NOW()-mid_time;
    cout << "snodes1__time(ms)\t" << exec_time.count() << endl;
    cout << "number_of_snodes1\t" << snodes1.size() << endl;//gnodes1
    cout << "number_of_gnodes1\t" << totalSize(snodes1) << endl;
    //cout << "adjusted_RandIdx1\t" << ari << endl;

    if (task == ALIGN1) {
      PointCloud<PointXYZRGB>::Ptr model1(new PointCloud<PointXYZRGB>);
      io::loadPLYFile(ply1, *model1);
      if (ptype1 == CHANGESIM) //ChangeSim models are too dense
        filterModelinPlace(model1);
      const Matrix4f ext = getTransformMatrix(pose1, 0);
      const Matrix3f intrinsics = cameraToIntrinsics(camera);
      visualizeGM(cloud1, MatrixXi(), ctype, model1, intrinsics, ext, A1);
      return 0;
    }

    if (task == ANIMA1) {
      /*sprintf(filepath, "images/%s+teapot2.ply", name1);
      const string ply1tea(filepath);
      PointCloud<PointXYZRGB>::Ptr tea1(new PointCloud<PointXYZRGB>);
      io::loadPLYFile(ply1tea, *tea1);
      int32_t rgb = (static_cast<uint32_t>(255) << 16 |
        static_cast<uint32_t>(255) << 8 | static_cast<uint32_t>(255));
      for (auto &p: tea1->points) p.rgb=rgb;*/

      PointCloud<PointXYZRGB>::Ptr model1(new PointCloud<PointXYZRGB>);
      io::loadPLYFile(ply1, *model1);
      if (ptype1 == CHANGESIM) //ChangeSim models are too dense
        filterModelinPlace(model1);
      const Matrix3f intrinsics = cameraToIntrinsics(camera);
      mid_time = NOW();
      //visualizeAM(tea1, intrinsics, pose1, name1);//cloud1
      visualizeAM(model1, intrinsics, pose1, name1);
      exec_time = NOW()-mid_time;
      cout << "animation_time(ms)\t" << exec_time.count() << endl;
      cout << "number_of_frames1\t" << count1 << endl;

      return 0;
    }

    cout << "number_of_frames2\t" << count2 << endl;
    PointCloud<PointXYZRGB>::Ptr cloud2(new PointCloud<PointXYZRGB>);
    mid_time = NOW();
    vector<SNode> snodes2 = computeNodes(name2, cloud2, pose2, Label, fuse,
      camera, rotated, saving, task==FORCED, start2, -1, ctype, false);
    exec_time = NOW()-mid_time;
    cout << "snodes2__time(ms)\t" << exec_time.count() << endl;
    cout << "number_of_snodes2\t" << snodes2.size() << endl;//gnodes2.size()
    cout << "number_of_gnodes2\t" << totalSize(snodes2) << endl;
    //cout << "adjusted_RandIdx2\t" << ari << endl;

    /*int test(0);
    for (const SNode& snode: snodes2) {
      float ioU{changeIoU(name2, snode[0], fuse, Label)};
      if (ioU > 0 || (snode[0].size() > 10 && snode[0][9]))
        cout << test << '\t' << snode[0][9] << '\t' << ioU << endl;
      ++test;
    }*/

    if (task == ALIGN2) {
      PointCloud<PointXYZRGB>::Ptr model2(new PointCloud<PointXYZRGB>);
      io::loadPLYFile(ply2, *model2);
      if (ptype2 == CHANGESIM) //ChangeSim models are too dense
        filterModelinPlace(model2);
      const Matrix4f ext = getTransformMatrix(pose2, 0);
      const Matrix3f intrinsics = cameraToIntrinsics(camera);
      visualizeGM(cloud2, MatrixXi(), ctype, model2, intrinsics, ext, A2);
      return 0;
    }

    if (task == ANIMA2) {
      /*sprintf(filepath, "images/%s+teapot2.ply", name2);
      const string ply2tea(filepath);
      PointCloud<PointXYZRGB>::Ptr tea2(new PointCloud<PointXYZRGB>);
      io::loadPLYFile(ply2tea, *tea2);
      int32_t rgb = (static_cast<uint32_t>(255) << 16 |
        static_cast<uint32_t>(255) << 8 | static_cast<uint32_t>(255));
      for (auto &p: tea2->points) p.rgb=rgb;*/

      PointCloud<PointXYZRGB>::Ptr model2(new PointCloud<PointXYZRGB>);
      io::loadPLYFile(ply2, *model2);
      if (ptype2 == CHANGESIM) //ChangeSim models are too dense
        filterModelinPlace(model2);
      const Matrix3f intrinsics = cameraToIntrinsics(camera);
      mid_time = NOW();
      //visualizeAM(tea2, intrinsics, pose2, name2); //cloud2
      visualizeAM(model2, intrinsics, pose2, name2); //cloud2
      exec_time = NOW()-mid_time;
      cout << "animation_time(ms)\t" << exec_time.count() << endl;
      cout << "number_of_frames2\t" << count2 << endl;
      return 0;
    }

    if (task == POINTS || task == FORCED) return 0;

    //Create the graphs' topologies
    const float thresh(0);
    mid_time = NOW();
    Topology Top1(snodes1);
    cout << name1 << "_edges(ms)\t" << -1 << endl;
    //Topology Top1(computeTopology(name1, snodes1, thresh, saving, false));//gnodes1
    exec_time = NOW()-mid_time;
    cout << "edges1__time(ms)\t" << exec_time.count() << endl;
    cout << "number_of_edges1\t" << Top1.getCount() << endl;
    cout << "connectivity1(%)\t" << (100*Top1.getCount())/(
      snodes1.size()*(snodes1.size()-1)) << endl;//gnodes1.size()*(gnodes1.size()-1))
    cout << "connectivity_threshS1\t" << Top1.getThreshold() << endl;

    if (task == EDGES1) {
      PointCloud<PointXYZRGB>::Ptr model1(new PointCloud<PointXYZRGB>);
      io::loadPLYFile(ply1, *model1);
      if (ptype1 == CHANGESIM) //ChangeSim models are too dense
        filterModelinPlace(model1);
      const Matrix4f ext = getTransformMatrix(pose1, 0);
      const Matrix3f intrinsics = cameraToIntrinsics(camera);
      visualizeGM(cloud1, Top1.getEdges(), ctype, model1, intrinsics, ext, A1);
      return 0;
    }

    mid_time = NOW();
    Topology Top2(snodes2);
    cout << name2 << "_edges(ms)\t" << -1 << endl;
    //Topology Top2(computeTopology(name2, snodes2, thresh, saving, false));//gnodes2
    exec_time = NOW()-mid_time;
    cout << "edgesS2__time(ms)\t" << exec_time.count() << endl;
    cout << "number_of_edgesS2\t" << Top2.getCount() << endl;
    cout << "connectivityS2(%)\t" << (100*Top2.getCount())/(
      snodes2.size()*(snodes2.size()-1)) << endl;//gnodes2.size()*(gnodes2.size()-1))
    cout << "connectivity_threshS2\t" << Top2.getThreshold() << endl;

    if (task == EDGES2) {
      PointCloud<PointXYZRGB>::Ptr model2(new PointCloud<PointXYZRGB>);
      io::loadPLYFile(ply2, *model2);
      if (ptype2 == CHANGESIM) //ChangeSim models are too dense
        filterModelinPlace(model2);
      const Matrix4f ext = getTransformMatrix(pose2, 0);
      const Matrix3f intrinsics = cameraToIntrinsics(camera);
      visualizeGM(cloud2, Top2.getEdges(), ctype, model2, intrinsics, ext, A2);
      //visualizeGM(cloud1, Top1.getEdges(), ctype, model2, intrinsics, ext, A2, R_gt, T_gt);
      return 0;
    }

    if (task == TOPOLO) return 0;

    if (task == TEASER) {
      //visualizeGG(cloud1, Top1.getEdges(), cloud1, Top1.getEdges(), MatrixX2i(),
      //  A1, Matrix3f::Identity(), Vector3f::Identity(), name1);
      visualize2D(cloud1, Top1.getEdges(), ctype, A1, name1);
      visualize2D(cloud2, Top2.getEdges(), ctype, A1, name2);
      return 0;
    }

    //Generate the descriptors
    Descriptor* Des1;
    mid_time = NOW();
    initDescriptor(Des1, dtype, depth, labels, Top1);
    exec_time = NOW()-mid_time;
    cout << "descriptor1_time(ms)\t" << exec_time.count() << endl;
    cout << "descriptor_size1\t" << Des1->getLen() << endl;

    Descriptor* Des2;
    mid_time = NOW();
    initDescriptor(Des2, dtype, depth, labels, Top2);
    exec_time = NOW()-mid_time;
    cout << "descriptor2_time(ms)\t" << exec_time.count() << endl;
    cout << "descriptor_size2\t" << Des2->getLen() << endl;

    //Graph matching
    mid_time = NOW();
    MatrixX2i matcherID(match(dtype, *Des1, *Des2, 3));
    exec_time = NOW()-mid_time;
    cout << "matcher_time(ms)\t" << exec_time.count() << endl;
    cout << "number_of_matches\t" << matcherID.rows() << endl;

    //#define CHANGES
    #ifdef CHANGES
    unsigned int good;
    MatrixX2i changesID(changeD(dtype, *Des1, *Des2, good, 3));
    #endif //CHANGES
    delete Des1;
    delete Des2;

    /*sprintf(filepath, "matches/%c-%d_%s.txt", dtype2char(dtype), depth, suffix);
    if (saving)
      save_matches(filepath, matcherID, name1, name2);
    else {
      #ifdef CHANGES
      for (const auto & point : gnodes1)
        cout << setfill(' ') << setw(2) << point[3] << ' ';
      cout << endl;
      cout << changesID.transpose() << endl;
      #endif //CHANGES
      for (int i = 0; i < gnodes1.size(); i++)
        if (gnodes1[i][3] == 11) {
          cout << 1 << ' ' << setfill(' ') << setw(2) << i << ' ';
          for (int j = 0; j < 3; j++) cout << gnodes1[i][j] << ' ';
          cout << endl;
        }
      for (int i = 0; i < gnodes2.size(); i++)
        if (gnodes2[i][3] == 11) {
          cout << 2 << ' ' << setfill(' ') << setw(2) << i << ' ';
          for (int j = 0; j < 3; j++) cout << gnodes2[i][j] << ' ';
          cout << endl;
        }
    }*/

    if (saving) {
      sprintf(filepath, "matches/%s", filename);
      save_matches(filepath, matcherID, name1, name2);
    } else {
      cout << "matches\n" << matcherID.transpose() << endl;
      /*namedWindow(string(name1)+" first");
      namedWindow(string(name2)+" first");
      namedWindow(string(name1)+" last");
      namedWindow(string(name2)+" last");
      const int ratio = ptype1 != CHANGESIM ? 7 : 1;
      moveWindow(string(name1)+" first", 100, 10);
      moveWindow(string(name2)+" first", 100+640, 10);
      moveWindow(string(name1)+" last", 100, 10+640);
      moveWindow(string(name2)+" last", 100+640, 10+640);
      for (int i = 0; i < matcherID.rows(); ++i) {
        GNode f1(snodes1[matcherID(i,0)][0]), f2(snodes2[matcherID(i,1)][0]),
              l1(snodes1[matcherID(i,0)].back()), l2(snodes2[matcherID(i,1)].back());
        Mat fmat1 = ratio*loadImage(name1, f1[4], 640, fuse, LABELMAP);
        Mat fmat2 = ratio*loadImage(name2, f2[4], 640, fuse, CHANGMAP);
        rectangle(fmat1, Point(f1[5],f1[6]), Point(f1[5]+f1[7], f1[6]+f1[8]),
          Scalar(255,255,255), 8);
        rectangle(fmat1, Point(f1[5],f1[6]), Point(f1[5]+f1[7], f1[6]+f1[8]),
          Scalar(0,0,0), 4);
        rectangle(fmat2, Point(f2[5],f2[6]), Point(f2[5]+f2[7], f2[6]+f2[8]),
          Scalar(255,255,255), 8);
        rectangle(fmat2, Point(f2[5],f2[6]), Point(f2[5]+f2[7], f2[6]+f2[8]),
          Scalar(0,0,0), 4);
        Mat lmat1 = ratio*loadImage(name1, l1[4], 640, fuse, LABELMAP);
        Mat lmat2 = ratio*loadImage(name2, l2[4], 640, fuse, CHANGMAP);
        rectangle(lmat1, Point(l1[5],l1[6]), Point(l1[5]+l1[7], l1[6]+l1[8]),
          Scalar(255,255,255), 8);
        rectangle(lmat1, Point(l1[5],l1[6]), Point(l1[5]+l1[7], l1[6]+l1[8]),
          Scalar(0,0,0), 4);
        rectangle(lmat2, Point(l2[5],l2[6]), Point(l2[5]+l2[7], l2[6]+l2[8]),
          Scalar(255,255,255), 8);
        rectangle(lmat2, Point(l2[5],l2[6]), Point(l2[5]+l2[7], l2[6]+l2[8]),
          Scalar(0,0,0), 4);
        imshow(string(name1)+" first", fmat1);
        imshow(string(name2)+" first", fmat2);
        imshow(string(name1)+" last", lmat1);
        imshow(string(name2)+" last", lmat2);
        waitKey(0);
      }
      destroyAllWindows();*/
      #ifdef CHANGES
      cout << "snodes1 lab/frame/x/y/w/h" << endl;
      for (int i = 3; i < snodes1[0][0].size(); i++) {
        for (const SNode& snode: snodes1)
          cout << setfill(' ') << setw(3) << (int)snode[0][i] << ' ';
        cout << endl;
      }
      cout << "matches(CD)\n" << changesID.transpose() << endl;
      cout << "snodes2 lab/frame/x/y/w/h" << endl;
      for (int i = 3; i < snodes2[0][0].size(); i++) {
        for (const SNode& snode: snodes2)
          cout << setfill(' ') << setw(3) << (int)snode[0][i] << ' ';
        cout << endl;
      }
      #endif //CHANGES
    }

    //Scene registration
    mid_time = NOW();
    Registration registration(snodes1, snodes2, matcherID);//gnodes1, gnodes2
    // #ifdef CHANGES
    // Registration registration2(gnodes1, gnodes2, changesID, good);
    // #endif //CHANGES

    if (rtype != NORANSAC) { //reject the outliers with ICP-RANSAC method
      registration.matcherRANSAC(0.25*Top1.getThreshold(), rtype);//2 //10
      cout << "number_of_inliers\t" << registration.inlierID.rows() << endl;
      #ifdef CHANGES
      cout << "inliers\n" << registration.inlierID.transpose() << endl;
      #endif //CHANGES
      registration.Alignment(); //final pose estimation based on inliers
    } else
      registration.Alignment(1000);
    // #ifdef CHANGES
    // registration2.Alignment(1000);
    // #endif //CHANGES

    exec_time = NOW()-mid_time;
    cout << "registration_time(ms)\t" << exec_time.count() << endl;

    exec_time = NOW()-start_time;
    cout << "total_runtime(ms)\t" << exec_time.count() << endl;

    if (!saving)
      cout << registration.evaluate(R_gt, T_gt) << endl;

    // const Matrix3f R = registration.Rotation;
    const Matrix3f R = registration.Rotation.rowwise().normalized();
    const Vector3f T = registration.Translation;
    // #ifdef CHANGES
    // const Matrix3f R2 = registration2.Rotation;
    // const Vector3f T2 = registration2.Translation;
    // #endif //CHANGES

    if (saving) {
      sprintf(filepath, "result/%s", filename);
      save_transform(filepath, R, T, name1, name2);
    } else
      cout<<"R\n"<< R<<"\nT\n"<< T<<"\nR_gt\n"<< R_gt<<"\nT_gt\n"<< T_gt<< endl;
    // #ifdef CHANGES
    // cout<<"R2\n"<< R2<<"\nT2\n"<< T2<< endl;
    // #endif //CHANGES

    //Compute RMS error
    Matrix4f RT = Matrix4f::Identity();
    RT.block<3,3>(0,0) = R;
    RT.block<3,1>(0,3) = T;

    //const Matrix4f M_rms = GT*RT.inverse() - Matrix4f::Identity();
    const Matrix4f M_rms = RT*GT.inverse() - Matrix4f::Identity();
    const Matrix3f A_rms = M_rms.block<3,3>(0,0);
    const Vector3f T_rms = M_rms.block<3,1>(0,3);

    //Load bounding box
    PointCloud<PointXYZRGB>::Ptr model2(new PointCloud<PointXYZRGB>);
    io::loadPLYFile(ply2, *model2); //2 is reference ?
    //Get center and dimensions from the aligned model
    Vector3f X_rms;
    Vector4f C_tmp;
    modelProperties(model2, A2, X_rms, C_tmp);

    //cout << "C_tmp\t" << C_tmp.transpose() << endl;
    Matrix4f A0 = Matrix4f::Identity();
    A0.block<4,1>(0,3) = -C_tmp;
    Matrix4f A0_inv = Matrix4f::Identity();
    A0_inv.block<4,1>(0,3) = C_tmp;
    C_tmp = A2.inverse()*C_tmp;
    //Transform the center to the target space
    const Vector3f C_rms(C_tmp(0),C_tmp(1),C_tmp(2));
    //cout << "C_rms\t" << C_rms.transpose() << endl;
    //cout << "X_rms\t" << X_rms.transpose() << endl;
    // DiagonalMatrix<float,3> D_rms(A_rms.col(0).squaredNorm(),
    //                               A_rms.col(1).squaredNorm(),
    //                               A_rms.col(2).squaredNorm());
    const DiagonalMatrix<float,3> D_rms(-2*A_rms.diagonal());
    const float e_tmp = X_rms.transpose()*D_rms*X_rms;
    const float e_rms = sqrt((T_rms+A_rms*C_rms).squaredNorm()+e_tmp/3.0f);

    const Matrix4f GTe = A0*A1*GT*A1_inv*A0_inv;
    const Matrix3f Re_gt = GTe.block<3,3>(0,0);
    const Vector3f Te_gt = GTe.block<3,1>(0,3);
    const Matrix4f RTe = A0*A1*RT*A1_inv*A0_inv;
    const Matrix3f Re = RTe.block<3,3>(0,0);
    const Vector3f Te = RTe.block<3,1>(0,3);
    const float r_err = (Re*Re_gt.inverse()-Matrix3f::Identity()).norm();
    const float t_err = (Te-Te_gt).norm();

    // printNumpyMat(RT, "RT = ");
    // printNumpyMat(GT, "RT_gt = ");

    if (saving) {
      sprintf(filepath, "error/%s", filename);
      save_error(filepath, t_err, r_err, e_rms, name1, name2);
    } else
      cout <<"T_err\t"<< t_err<<"\nR_err\t"<< r_err<<"\nRMS_d\t"<< e_rms<< endl;

    //plot the semantic point and matching with PCL library
    if (task == MODELS) {
      PointCloud<PointXYZRGB>::Ptr model1(new PointCloud<PointXYZRGB>);
      io::loadPLYFile(ply1, *model1);
      if (ptype1 == CHANGESIM) //ChangeSim models are too dense
        filterModelinPlace(model1);
      PointCloud<PointXYZRGB>::Ptr model2(new PointCloud<PointXYZRGB>);
      io::loadPLYFile(ply2, *model2);
      if (ptype2 == CHANGESIM) //ChangeSim models are too dense
        filterModelinPlace(model2);
      visualizeMM(model1, model2, A2, R, T);
    } else if (task == CLOUDS)
      visualizeGG(cloud1, MatrixXi(), cloud2, MatrixXi(), ctype, matcherID, A2, R, T); //inlierID, Identity

    return 0;
}
