#include "pointCloudExtraction.hpp"

#include <opencv2/highgui.hpp> //imshow
#include <opencv2/imgproc.hpp> //resize, circle

#include <iostream>  //cout
#include <stack>

using namespace cv;
using namespace std;
using namespace Eigen;

#include "contours.hpp"

pointCloudExtraction::pointCloudExtraction(vector<float>&camera, uchar rotated) {
  s = camera[4];
  fx = camera[0]*s;
  fy = camera[1]*s;
  cx = camera[2]*s;
  cy = camera[3]*s;
  rot = rotated;
}

void LUT16u2(const ushort* src, const uchar* lut, uchar* dst, const int len) {
  for (int i = 0; i < len; i++)
    dst[i] = lut[src[i]];
}

void LUT8u2(const uchar* src, const uchar* lut, uchar* dst, const int len) {
  for (int i = 0; i < len; i++)
    dst[i] = lut[src[i]];
}

Mat loadImage(const char* name, int idx, int width,
  const uchar* fuse, char type) {
  char path[256];
  sprintf(path, "../Dataset/%s/%s/%d.png", name, type==DEPTHMAP ? "depth"   :
                                                 type==CHANGMAP ? "changes" :
                                                 "label", idx);
  Mat image = imread(path, IMREAD_UNCHANGED);
  if (image.cols != width)
    resize(image, image, Size(), (float)width/image.cols,
                                 (float)width/image.cols, INTER_NEAREST);
  if (type==DEPTHMAP) {
    image.convertTo(image, CV_32FC1, image.depth()==CV_8U ? 0.2 : 0.001);
    return image;
  }
  if (fuse) {
    Mat image8u = Mat::zeros(image.size(), CV_8UC1);
    LUT16u2((const ushort*)image.data, fuse, (uchar*)image8u.data, image.total());
    return image8u;
  }
  return image;
}

void pointCloudExtraction::insertValue(Mat &depthImg, Mat &labelImg,
  const Mat &extraImg, const uchar* fuse) {
    Mat labelTmp = labelImg;
    Mat depthTmp = depthImg;
    if (s != 1) //depth is the focus
      resize(depthTmp, depthTmp, Size(), s, s, INTER_NEAREST);
    if (labelTmp.size() != depthTmp.size())
      resize(labelTmp, labelTmp, depthTmp.size(), 1, 1, INTER_NEAREST);
    const int cols(depthTmp.cols), rows(depthTmp.rows);

    depthMap = Mat::zeros(rows+2, cols+2, CV_32FC1);
    depthTmp.convertTo(depthMap(Rect(1, 1, cols, rows)),
      depthMap.type(), depthTmp.depth()==CV_8U ? 0.2 : 0.001); //200*0.001

    Mat deltaX = Mat::zeros(rows+2, cols+2, CV_32FC1);
    deltaX(Rect(1, 1, cols, rows)) =
      depthMap(Rect(0, 1, cols, rows)) - depthMap(Rect(1, 1, cols, rows));
    inRange(deltaX, Scalar(-1), Scalar(1), threshX);

    Mat deltaY = Mat::zeros(rows+2, cols+2, CV_32FC1);
    deltaY(Rect(1, 1, cols, rows)) =
      depthMap(Rect(1, 0, cols, rows)) - depthMap(Rect(1, 1, cols, rows));
    inRange(deltaY, Scalar(-1), Scalar(1), threshY);

    if (fuse) {
      Mat image8u = Mat::zeros(rows, cols, CV_8UC1);
      if (labelTmp.depth() == CV_16U)
        LUT16u2((const ushort*)labelTmp.data, fuse, (uchar*)image8u.data, rows*cols);
      else
        LUT8u2(  (const uchar*)labelTmp.data, fuse, (uchar*)image8u.data, rows*cols);
      labelMap = Mat::zeros(rows+2, cols+2, image8u.type());
      image8u.copyTo(labelMap(Rect(1, 1, cols, rows)));
    } else {
      labelMap = Mat::zeros(rows+2, cols+2, labelTmp.type());
      labelTmp.copyTo(labelMap(Rect(1, 1, cols, rows)));
    }

    if (!extraImg.empty()) {
      extraMap = Mat::zeros(rows+2, cols+2, extraImg.type());
      if (extraImg.size() != depthTmp.size())
        resize(extraImg, extraMap(Rect(1, 1, cols, rows)), depthTmp.size(), 1, 1, INTER_NEAREST);
      else
        extraImg.copyTo(extraMap(Rect(1, 1, cols, rows)));
    }

    //remove pixels with no depth
    if (labelMap.channels() > 1)  {
      cv::Mat channels[3];
      split(labelMap, channels);
      bitwise_and(channels[0], depthMap!=0, channels[0]);
      bitwise_and(channels[1], depthMap!=0, channels[1]);
      bitwise_and(channels[2], depthMap!=0, channels[2]);
      merge(channels, 3, labelMap);
      //multilabel bool?
      //convert to int-based mono channel?
    } else
      bitwise_and(labelMap, depthMap!=0, labelMap);
}

#define DIST2(x,y,z,q) pow((q)[0]-(x),2)+pow((q)[1]-(y),2)+pow((q)[2]-(z),2)
#define DISTS(p,q) pow((q)[0]-(p)[0],2)+pow((q)[1]-(p)[1],2)+pow((q)[2]-(p)[2],2)

//Legacy object extraction algorithm: the node the first occurence of the object
//keep: adds other occurences to the tail of the super node for ARI computation
int addLNode(vector<SNode> &snodes, const Vector4f &p, const int lab,
  const float thr, const int frame=-1,
  const int x=-1, const int y=-1, const int w=-1, const int h=-1,
  const int change=0, const int instance=-1, const bool real=true, const bool keep=false) {
  GNode new_node({ p[0], p[1], p[2], (float)lab, (float)frame,
    (float)x, (float)y, (float)w, (float)h, (float)change, (float)instance });
  int id(0);
  if (instance < 0 || real) { //search object by distance threshold
    for (SNode& snode : snodes) {
      if (lab == (int)snode.front()[NODE_L])
        if (DISTS(p,snode.front()) < thr) {
          if (keep)
            snode.push_back(new_node);
          return id; //merge (i.e. no new node if keep is false)
        }
      id++;
    }
  } else
    for (SNode& snode : snodes) { //search object by instance matching
      if (lab == (int)snode.front()[NODE_L] && instance == (int)snode.front()[NODE_I]) {
        if (keep)
          snode.push_back(new_node);
        return id; //merge (i.e. no new node if keep is false)
      }
      id++;
    }
  snodes.emplace_back(SNode({new_node})); //no match: create a new super node
  return snodes.size()-1;
}

//Super node object extraction algorithm: stores all occurences of the objects
//merge: check if super nodes are close enough to be merged
unsigned int addSNode(vector<SNode> &snodes, const Vector4f &p, const int lab,
  const float thr, const int frame=-1,
  const int x=-1, const int y=-1, const int w=-1, const int h=-1,
  const int change=0, const int instance=-1, const bool real=true, const bool merge=true) {
  GNode new_node({ p[0], p[1], p[2], (float)lab, (float)frame,
    (float)x, (float)y, (float)w, (float)h, (float)change, (float)instance });
  unsigned int id{0};
  if (instance < 0 || real) { //search object by distance threshold
    vector<unsigned int> ids;
    if (merge)
      ids.reserve(snodes.size());
    for (SNode& snode : snodes) {
      if (lab == (int)snode.front()[NODE_L])
        for (const GNode& node : snode)
          if (DISTS(p,node) < thr)
            if (merge) {
              ids.push_back(id);
              break; //merge is enabled: search for another matching SNode
            } else {
              snode.push_back(new_node);
              return id; //merge is disabled: return after one matching SNode
            }
      ++id;
    }
    if (ids.size() > 1) //matches: need to merge the snodes (merge is true)
      for (const unsigned int& from: ids)
        mergeSNodes(snodes, ids[0], from); //the first SNode is the recipient
    if (ids.size() > 0) { //match: push to first super node (merge is true)
      snodes[ids[0]].emplace_back(new_node);
      return ids[0];
    }
  } else //search object by instance matching
    for (SNode& snode : snodes) {
      if (     lab == (int)snode.front()[NODE_L] &&
          instance == (int)snode.front()[NODE_I]) {
        snode.push_back(new_node);
        return id;
      }
      ++id;
    }
  snodes.emplace_back(SNode({new_node})); //no SNode was found, create new one
  return snodes.size()-1;
}

#define MAX_DEP 30//5 //15//30
#define REAL_MODE false
#define DEBUG_PAUSE false
// new(RGB)	 81  38   0 == 3
// missing	 41  36 132 == 2
// replaced	131 192  13 == 1
// rotated	 25  48  16 == 4
#define CHANGE_IDX(x) ((x)%13%8) //%4
#define BRG2CHANGE(ext,i) CHANGE_IDX((ext)[3*(i)+2])
const char* depth_window = "Depth";
const char* label_window = "Label";

//if "real" is false use the extraMap to help differentiate instances
void pointCloudExtraction::RawBlobExtraction(Blobs &blobs,
  const bool channels, const int bits, Matrix4f& pose, int frame_num,
  const bool real, const bool debug) {
        uchar* seg = (uchar*) labelMap.data; //direct access to image
  const float* dep = (float*) depthMap.data;
  const uchar* ext = (uchar*) extraMap.data; //BGR values if 3 channels
  const uchar* thX = (uchar*)  threshX.data;
  const uchar* thY = (uchar*)  threshY.data;
  const int cols{labelMap.cols};
  const int rows{labelMap.rows};
  const bool has_ext{!extraMap.empty()};
  Mat label, depth;
  if (debug) {
    namedWindow(depth_window);
    namedWindow(label_window);
    moveWindow(depth_window, 100, 100);
    moveWindow(label_window, 100+(cols-2)/2, 100);
    if (channels)
      label =   labelMap.clone()(Rect(cols/2+1, 1, (cols-2)/2, rows-2));
    else
      label = 7*labelMap.clone()(Rect(cols/2+1, 1, (cols-2)/2, rows-2));
    //in theory: labelMap.rows==depthMap.rows && labelMap.cols==depthMap.cols
    depth = Mat::zeros(rows-2, (cols-2)/2, CV_8U);
    depthMap(Rect(1, 1, (cols-2)/2, rows-2)).convertTo(depth,
      CV_8U, bits==CV_8U ? 5 : 1000.0/16.0);
  }
  for (int i = cols; i < cols*(rows-1); i++) { //ignore first/last rows
    const int lab{channels ? BGR2INT(seg,i) : seg[i]};
    if (lab) {
      int    change{ channels && has_ext ? BRG2CHANGE(ext,i) :      0};
      int    instan{!channels && has_ext ?            ext[i] :     -1};
      stack<int> neighbors;
      neighbors.push(i); //seed
      int count{0}, minU{rows}, maxU{0}, minV{cols}, maxV{0};
      //float as to avoid large number issues
      float sumX{0},sumY{0},sumZ{0}, su2X{0},su2Y{0},su2Z{0};
      do { //4-neighbor spread
        const int j{neighbors.top()};
        neighbors.pop();
        if (real || !has_ext) {
          if (channels) {
            if (BGR2INT(seg,j-1)    == lab && thX[j])      neighbors.push(j-1);
            if (BGR2INT(seg,j+1)    == lab && thX[j+1])    neighbors.push(j+1);
            if (BGR2INT(seg,j-cols) == lab && thY[j])      neighbors.push(j-cols);
            if (BGR2INT(seg,j+cols) == lab && thY[j+cols]) neighbors.push(j+cols);
            if (has_ext && change==0 && BRG2CHANGE(ext,j)!=0)
              change = BRG2CHANGE(ext,j);
            seg[3*j] = 0; seg[3*j+1] = 0; seg[3*j+2] = 0;
          } else {
            if (seg[j-1]    == lab && thX[j])      neighbors.push(j-1);
            if (seg[j+1]    == lab && thX[j+1])    neighbors.push(j+1);
            if (seg[j-cols] == lab && thY[j])      neighbors.push(j-cols);
            if (seg[j+cols] == lab && thY[j+cols]) neighbors.push(j+cols);
            if (has_ext && instan==-1 && ext[j]!=-1)
              instan = ext[j]; //cout<<"ext warning "<<instan<<' '<<(int)ext[j]<<endl;
            seg[j] = 0;
          }
        } else {
          if (channels) {
            if (BGR2INT(seg,j-1)   ==lab && change==BRG2CHANGE(ext,j-1)    &&
              thX[j])      neighbors.push(j-1);
            if (BGR2INT(seg,j+1)   ==lab && change==BRG2CHANGE(ext,j+1)    &&
              thX[j+1])    neighbors.push(j+1);
            if (BGR2INT(seg,j-cols)==lab && change==BRG2CHANGE(ext,j-cols) &&
              thY[j])      neighbors.push(j-cols);
            if (BGR2INT(seg,j+cols)==lab && change==BRG2CHANGE(ext,j+cols) &&
              thY[j+cols]) neighbors.push(j+cols);
            seg[3*j] = 0; seg[3*j+1] = 0; seg[3*j+2] = 0;
          } else { //thX and thY are useless if we have the instance GT
            if (seg[j-1]   ==lab && instan==ext[j])      neighbors.push(j-1);
            if (seg[j+1]   ==lab && instan==ext[j+1])    neighbors.push(j+1);
            if (seg[j-cols]==lab && instan==ext[j])      neighbors.push(j-cols);
            if (seg[j+cols]==lab && instan==ext[j+cols]) neighbors.push(j+cols);
            seg[j] = 0;
          }
        }
        const float z{dep[j]};
        if (z <= MAX_DEP) {
          ++count;
          const int v{ j     %cols-1}; //-1 for padding compensation
          const int u{(j-v+1)/cols-1}; //-1 for padding compensation
          //Bounding-box
          if (v > maxV) maxV = v;
          if (v < minV) minV = v;
          if (u > maxU) maxU = u;
          if (u < minU) minU = u;

          const int ur{(rot == ROTATE_NONE)                ? u :
                       (rot == ROTATE_90_CLOCKWISE) ? cols-2-v :
                       (rot == ROTATE_90_COUNTERCLOCKWISE) ? v :
                       (rot == ROTATE_180)          ? rows-2-u : u};
          const int vr{(rot == ROTATE_NONE)                ? v :
                       (rot == ROTATE_90_CLOCKWISE)        ? u :
                       (rot == ROTATE_90_COUNTERCLOCKWISE) ? rows-2-u :
                       (rot == ROTATE_180)          ? cols-2-v : v};
          //3D geometry
          const float x{z*(vr-cx)}; //centered but not scaled by fx
          const float y{z*(ur-cy)}; //centered but not scaled by fy
          sumX += x;
          sumY += y;
          sumZ += z;
          su2X += x*x;
          su2Y += y*y;
          su2Z += z*z;
        }
      } while (!neighbors.empty());
      if (count > 0) { //necessary if all pixels of the blob have "z > MAX_DEP"
        //Store result
        const float meaX{sumX/count}, //centered but not scaled by fx
                    meaY{sumY/count}, //centered but not scaled by fy
                    meaZ{sumZ/count};
        const float varX{su2X/count-meaX*meaX}, //centered but not scaled by fx*fx
                    varY{su2Y/count-meaY*meaY}, //centered but not scaled by fy*fy
                    varZ{su2Z/count-meaZ*meaZ}; //we could move "count" later
        Vector4f meaV = pose*Vector4f{meaX/fx, meaY/fy, meaZ, 1.0f};
        Vector4f sdeV = pose*Vector4f{varX > 0.0f ? sqrt(varX)/fx : 0.0f,
                                      varY > 0.0f ? sqrt(varY)/fy : 0.0f,
                                      varZ > 0.0f ? sqrt(varZ)    : 0.0f, 0.0f};
        if (debug && (isnan(meaV[0]) || isnan(meaV[1]) || isnan(meaV[2]) ||
                      isnan(sdeV[0]) || isnan(sdeV[1]) || isnan(sdeV[2])))
          cout << frame_num << '\n' << pose << endl;
        //float is required if we want to compute z and apply the pose here
        blobs.push_back({(float)frame_num, (float)instan, (float)change, (float)lab,
           (float)count, meaV[0], meaV[1], meaV[2], sdeV[0], sdeV[1], sdeV[2],
           (float)minV, (float)minU, (float)maxV, (float)maxU});
        if (debug) {
          rectangle(label, Point(minV-(cols-2)/2,minU), Point(maxV-(cols-2)/2,maxU),
            Scalar(255,255,255), 8, LINE_8);
          rectangle(label, Point(minV-(cols-2)/2,minU), Point(maxV-(cols-2)/2,maxU),
            Scalar(0,0,0), 3, LINE_8);
          rectangle(depth, Point(minV,minU), Point(maxV,maxU),
            Scalar(255), 8, LINE_8);
        }
      }
    }
	}
  if (debug) {
    imshow(depth_window, depth);
    imshow(label_window, label);
    waitKey(DEBUG_PAUSE?0:1);
  }
}

void pointCloudExtraction::RegionExtractionB(Graph &snodes, const bool legacy,
  const bool channels, const int bits, Matrix4f& pose, int frame_num,
  const int thr, const bool covariance,
  const bool real, const bool debug) {
        uchar* seg = (uchar*) labelMap.data; //direct access to image
  const float* dep = (float*) depthMap.data;
  const uchar* ext = (uchar*) extraMap.data;
  const uchar* thX = (uchar*)  threshX.data;
  const uchar* thY = (uchar*)  threshY.data;
  const int cols{labelMap.cols};
  const int rows{labelMap.rows};
  const bool has_ext{!extraMap.empty()};
  Mat label, depth;
  if (debug) {
    namedWindow(depth_window);
    namedWindow(label_window);
    moveWindow(depth_window, 100, 100);
    moveWindow(label_window, 100+(cols-2)/2, 100);
    if (channels)
      label =   labelMap.clone()(Rect(cols/2+1, 1, (cols-2)/2, rows-2));
    else
      label = 7*labelMap.clone()(Rect(cols/2+1, 1, (cols-2)/2, rows-2));
    //in theory: labelMap.rows==depthMap.rows && labelMap.cols==depthMap.cols
    depth = Mat::zeros(rows-2, (cols-2)/2, CV_8U);
    depthMap(Rect(1, 1, (cols-2)/2, rows-2)).convertTo(depth,
      CV_8U, bits==CV_8U ? 5 : 1000.0/16.0);
  }
  for (int i = cols; i < cols*(rows-1); i++) { //ignore first/last rows
    const int lab{channels ? BGR2INT(seg,i) : seg[i]};
    if (lab) {
      int    change{ channels && has_ext ? BRG2CHANGE(ext,i) :      0};
      int    instan{!channels && has_ext ?            ext[i] :     -1};
      stack<int> neighbors;
      neighbors.push(i); //seed
      int count{0}, minU{rows}, maxU{0}, minV{cols}, maxV{0};
      //float as to avoid large number issues
      double sumX{0},sumY{0},sumZ{0}, su1X{0},su1Y{0}, su2X{0},su2Y{0},su2Z{0};
      do { //4-neighbor spread
        const int j{neighbors.top()};
        neighbors.pop();
        if (real || !has_ext) {
          if (channels) {
            if (BGR2INT(seg,j-1)    == lab && thX[j])      neighbors.push(j-1);
            if (BGR2INT(seg,j+1)    == lab && thX[j+1])    neighbors.push(j+1);
            if (BGR2INT(seg,j-cols) == lab && thY[j])      neighbors.push(j-cols);
            if (BGR2INT(seg,j+cols) == lab && thY[j+cols]) neighbors.push(j+cols);
            if (has_ext && change==0 && BRG2CHANGE(ext,j)!=0)
              change = BRG2CHANGE(ext,j);
            seg[3*j] = 0; seg[3*j+1] = 0; seg[3*j+2] = 0;
          } else {
            if (seg[j-1]    == lab && thX[j])      neighbors.push(j-1);
            if (seg[j+1]    == lab && thX[j+1])    neighbors.push(j+1);
            if (seg[j-cols] == lab && thY[j])      neighbors.push(j-cols);
            if (seg[j+cols] == lab && thY[j+cols]) neighbors.push(j+cols);
            if (has_ext && instan==-1 && ext[j]!=-1)
              instan = ext[j]; //cout<<"ext warning "<<instan<<' '<<(int)ext[j]<<endl;
            seg[j] = 0;
          }
        } else {
          if (channels) {
            if (BGR2INT(seg,j-1)   ==lab && change==BRG2CHANGE(ext,j-1)    &&
              thX[j])      neighbors.push(j-1);
            if (BGR2INT(seg,j+1)   ==lab && change==BRG2CHANGE(ext,j+1)    &&
              thX[j+1])    neighbors.push(j+1);
            if (BGR2INT(seg,j-cols)==lab && change==BRG2CHANGE(ext,j-cols) &&
              thY[j])      neighbors.push(j-cols);
            if (BGR2INT(seg,j+cols)==lab && change==BRG2CHANGE(ext,j+cols) &&
              thY[j+cols]) neighbors.push(j+cols);
            seg[3*j] = 0; seg[3*j+1] = 0; seg[3*j+2] = 0;
          } else { //thX and thY are useless if we have the instance GT
            if (seg[j-1]   ==lab && instan==ext[j])      neighbors.push(j-1);
            if (seg[j+1]   ==lab && instan==ext[j+1])    neighbors.push(j+1);
            if (seg[j-cols]==lab && instan==ext[j])      neighbors.push(j-cols);
            if (seg[j+cols]==lab && instan==ext[j+cols]) neighbors.push(j+cols);
            seg[j] = 0;
          }
        }
        const float z{dep[j]};
        if (z <= MAX_DEP) {
          ++count;
          const int v{ j   %cols};
          const int u{(j-v)/cols};
          //Bounding-box
          if (v > maxV) maxV = v;
          if (v < minV) minV = v;
          if (u > maxU) maxU = u;
          if (u < minU) minU = u;
          //TODO: pixel-wise inaccuracies
          const int ur{(rot == ROTATE_NONE)                ? u :
                       (rot == ROTATE_90_CLOCKWISE) ? cols-1-v :
                       (rot == ROTATE_90_COUNTERCLOCKWISE) ? v :
                       (rot == ROTATE_180)          ? rows-1-u : u};
          const int vr{(rot == ROTATE_NONE)                ? v :
                       (rot == ROTATE_90_CLOCKWISE)        ? u :
                       (rot == ROTATE_90_COUNTERCLOCKWISE) ? rows-1-u :
                       (rot == ROTATE_180)          ? cols-1-v : v};
          //3D geometry
          sumX += z*v;
          sumY += z*u;
          sumZ += z;
          if (covariance) {
            const float zz{z*z};
            su1X += zz*v; //useless if we compute x and y in the loop
            su1Y += zz*u;
            su2X += zz*(v*v);
            su2Y += zz*(u*u);
            su2Z += zz;
          }
        }
      } while (!neighbors.empty());
      if (count > thr) {
        const double meaX(sumX/count),
                     meaY(sumY/count),
                     meaZ(sumZ/count);
        Vector4f point((meaX-meaZ*(cx+1))/fx,
                       (meaY-meaZ*(cy+1))/fy,
                        meaZ, 1);
        unsigned int id;
        if (legacy) {
          point = pose*point;
          id = addLNode(snodes, point, lab, 1.0f, frame_num, minV, minU,
            maxV-minV+1, maxU-minU+1, change, instan, REAL_MODE, false); //dont keep nodes
        } else {
          if (covariance) {
            const double varX(su2X/count-meaX*meaX),
              varY(su2Y/count-meaY*meaY),
              varZ(su2Z/count-meaZ*meaZ);
            Vector3f dvect(sqrt(varX+
              (1+cx)*((1+cx)*varZ-2*su1X/count+2*meaX*meaZ))/fx,
              sqrt(varY+
              (1+cy)*((1+cy)*varZ-2*su1Y/count+2*meaY*meaZ))/fy,
              sqrt(varZ));
            for (int k = 0; k < 3; ++k) {
              Vector4f pointk(point);
              pointk(k) = point(k)+dvect(k);
              pointk = pose*pointk;
              addSNode(snodes, pointk, lab, 1.0f, frame_num, minV, minU,
                maxV-minV+1, maxU-minU+1, change, instan);
              pointk = point;
              pointk(k) = point(k)-dvect(k);
              pointk = pose*pointk;
              addSNode(snodes, pointk, lab, 1.0f, frame_num, minV, minU,
                maxV-minV+1, maxU-minU+1, change, instan);
            }
          }
          point = pose*point;
          id = addSNode(snodes, point, lab, 1.0f, frame_num, minV, minU,
            maxV-minV+1, maxU-minU+1, change, instan);
        }

        if (debug) {
          if (snodes[id].size() == 7 && covariance || snodes[id].size() == 1) {
            Vector4f point0(snodes[id].back()[0],
                            snodes[id].back()[1],
                            snodes[id].back()[2], 1);
            point0 = pose.inverse()*point0;
            const float V = point0[0]*fx/point0[2]+cx;
            const float U = point0[1]*fy/point0[2]+cy;
            circle(label, Point(V-cols/2,U), 10, Scalar(255,255,255), 8, LINE_8);
            circle(label, Point(V-cols/2,U), 10, Scalar(0,0,0), 3, LINE_8);
            circle(depth, Point(V,U), 10, Scalar(255), 8, LINE_8);
          }
        }
      }
    }
	}
  if (debug) {
    imshow(depth_window, depth);
    imshow(label_window, label);
    waitKey(DEBUG_PAUSE?0:1);
  }
}

//------------------------------------------------------------------------------
/*void pointCloudExtraction::insertValueC(Mat &depthImg, Mat &labelImg,
  const uchar* fuse) {
    //cout<<"received the depth map, start generate the point cloud"<<endl;
    Mat labelTmp = labelImg;
    Mat depthTmp = depthImg;
    if (s != 1) //depth is the focus
      resize(depthTmp, depthTmp, Size(), s, s, INTER_NEAREST);
    if (labelTmp.size() != depthTmp.size())
      resize(labelTmp, labelTmp, depthTmp.size(), 1, 1, INTER_NEAREST);
    //cout<<depthTmp<<endl;
    const int cols(depthTmp.cols), rows(depthTmp.rows);

    depthMap = Mat::zeros(rows, cols, CV_32FC1);
    depthTmp.convertTo(depthMap, depthMap.type(), 0.001);

    labelMap = Mat::zeros(rows, cols, CV_8UC1);
    LUT16u2((const ushort*)labelTmp.data,fuse,(uchar*)labelMap.data,rows*cols);

    //remove pixels with no depth
    bitwise_and(labelMap, depthMap!=0, labelMap);
}

void addGNode(vector<GNode> &nodes, const Vector4f &p, const int lab,
  const float thr, const int frame=-1,
  const int x=-1, const int y=-1, const int w=-1, const int h=-1,
  const int change=0, const int instance=-1, const bool fuse=true) {
  if (fuse) //look for existing gnodes in the same region
    for (const auto& q : nodes)
      if (lab == (int)q[NODE_L])
        if (DISTS(p,q) < thr) return; //merge (i.e. no new node)
  nodes.emplace_back(GNode({ p[0], p[1], p[2], (float)lab, (float)frame,
    (float)x, (float)y, (float)w, (float)h, (float)change, (float)instance }));
}

// Mono-labeler
void pointCloudExtraction::RegionExtractionC(vector<GNode> &points,
  Matrix4f& pose) const {
    Mat mask = labelMap>0;
    double minLab, maxLab;
    minMaxIdx(labelMap, &minLab, &maxLab, 0, 0, mask);
    Mat binIm;
    for (uchar lab = (uchar)minLab; lab <= (uchar)maxLab; lab++) {
      inRange(labelMap, Scalar(lab), Scalar(lab), binIm);
      if (!countNonZero(binIm)) continue;

      Exteriors exteriors;
      Interiors interiors;
      labeling(binIm, exteriors, interiors);

      // imwrite("CC-IMG.png", 25*binIm+5);
      // Mat extImg = Mat::zeros(labelMap.size(), CV_8UC3);
      // Mat intImg = Mat::zeros(labelMap.size(), CV_8UC3);
      // for (int b = 0; b < exteriors.size(); b++)
      //   if (exteriors[b].size()) {
      //     Scalar color(rand()&255,rand()&255,rand()&255);
      //     drawContours(extImg, exteriors, b, color, 1, 8);
      //     if (interiors[b].size())
      //       drawAllContours(intImg, interiors[b], color, 1, 8)
      //   }
      // imwrite("CC-EXT.png",extImg);
      // imwrite("CC-INT.png",intImg);
      // if (lab > (uchar)minLab)
      // exit(0);

      for (int b = 0; b < exteriors.size(); b++)
        if (exteriors[b].size() > 40) { //minimum required == sqrt(area/pi)
          //Mat intMask = Mat::ones(binIm.size(), CV_8U);
          //drawAllContours(intMask, interiors[b], Scalar(0), FILLED, 8);
          Moments M = moments(exteriors[b]);
          if (M.m00 > 5000) {
            Point2i center(M.m10/M.m00, M.m01/M.m00);
            float cz = depthMap.at<float>(center.y,center.x);
            Vector4f point((center.x-cx*cz)/fx, (center.y-cy*cz)/fy, cz, 1);
            point = pose*point;
            addGNode(points, point, lab, 1.0f); //25
          }
        }
    }
}*/

// Legacy multi-labeler
/*void pointCloudExtraction::RegionExtractionCM(vector<GNode> &points,
  Matrix4f& pose) const {
  Contours exteriors, interiors;
  multiLabeling(labelMap, exteriors, interiors);
  // Mat intMask = Mat::ones(labelMap.size(), CV_8U);
  // for (int b = 0; b < interiors.size(); b++)
  //   if (interiors[b].size())
  //     drawContours(labelMap, interiors, b, Scalar(0), FILLED, 8);
  // Mat intImg = Mat::zeros(labelMap.size(), CV_8U);
  // drawAllContours(intImg, interiors);
  // imwrite("INT-B.png", intImg);
  // exit(0);

  for (int b = 0; b < exteriors.size(); b++)
    if (exteriors[b].size() > 40) { //minimum required == sqrt(area/pi)
      Moments M = moments(exteriors[b]);
      int area = M.m00; //contourArea(exteriors[b], intMask);
      if (area > 5000) {
        Point2i center(M.m10/M.m00, M.m01/M.m00); //contourCenter(exteriors[b], intMask);
        int lab = labelMap.at<uchar>(center);//labelMap.at<uchar>(exteriors[b][0]);
        if (lab != labelMap.at<uchar>(exteriors[b][0]))
          continue;//cout << '!';
        float cz = depthMap.at<float>(center);
        Vector4f point((center.x-cx)*cz/fx, (center.y-cy)*cz/fy, cz, 1);
        point = pose*point;
        addGNode(points, point, lab, 1.0f); //25
      }
    }
}*/

// Fast multi-labeler
/*void pointCloudExtraction::RegionExtractionM(vector<GNode> &points,
  Matrix4f& pose) const {
  Contours exteriors;
  multiLabeling(labelMap, exteriors);
  Mat extImg = Mat::zeros(labelMap.size(), CV_8UC3);
  for (int b = 0; b < exteriors.size(); b++)
    if (exteriors[b].size()) {
      Scalar color(rand()&255,rand()&255,rand()&255);
      drawContours(extImg, exteriors, b, color, 1, 8);
    }
  imwrite("M-EXT.png",extImg);
  exit(0);

  for (int b = 0; b < exteriors.size(); b++)
    if (exteriors[b].size() > 40) { //minimum required == sqrt(area/pi)
      Moments M = moments(exteriors[b]);
      int area = M.m00;
      if (area > 5000) {
        Point2i center(M.m10/M.m00, M.m01/M.m00);
        int lab = labelMap.at<uchar>(center);
        if (lab != labelMap.at<uchar>(exteriors[b][0]))
          continue;//cout << '!';
        float cz = depthMap.at<float>(center);
        Vector4f point((center.x-cx)*cz/fx, (center.y-cy)*cz/fy, cz, 1);
        point = pose*point;
        addGNode(points, point, lab, 1.0f); //25
      }
    }
}

// Full multi-labeler
void pointCloudExtraction::RegionExtractionF(vector<GNode> &points,
  Matrix4f& pose) const {
  Exteriors exteriors;
  Interiors interiors;
  multiLabeling(labelMap, exteriors, interiors);
  imwrite("CCM-IMG.png", 25*labelMap+5);
  Mat extImg = Mat::zeros(labelMap.size(), CV_8UC3);
  Mat intImg = Mat::zeros(labelMap.size(), CV_8UC3);
  for (int b = 0; b < exteriors.size(); b++)
    if (exteriors[b].size()) {
      Scalar color(rand()&255,rand()&255,rand()&255);
      drawContours(extImg, exteriors, b, color, 1, 8);
      if (interiors[b].size())
        for (int c = 0; c < interiors[b].size(); c++)
          if (interiors[b][c].size())
            drawContours(intImg, interiors[b], c, color, 1, 8);
    }
  imwrite("CCM-EXT.png",extImg);
  imwrite("CCM-INT.png",intImg);
  exit(0);
  // Mat intMask = Mat::ones(labelMap.size(), CV_8U);
  // for (int b = 0; b < interiors.size(); b++)
  //   if (interiors[b].size())
  //     drawContours(intMask, interiors, b, Scalar(0), FILLED, 8);

  for (int b = 0; b < exteriors.size(); b++)
    if (exteriors[b].size() > 40) { //minimum required == sqrt(area/pi)
      Moments M = moments(exteriors[b]);
      int area = M.m00; //contourArea(exteriors[b], intMask);
      if (area > 5000) {
        Point2i center(M.m10/M.m00, M.m01/M.m00); //contourCenter(exteriors[b], intMask);
        int lab = labelMap.at<uchar>(center);//labelMap.at<uchar>(exteriors[b][0]);
        if (lab != labelMap.at<uchar>(exteriors[b][0]))
          continue;//cout << '!';
        float cz = depthMap.at<float>(center);
        Vector4f point((center.x-cx)*cz/fx, (center.y-cy)*cz/fy, cz, 1);
        point = pose*point;
        addGNode(points, point, lab, 1.0f); //25
      }
    }
}

// Thin multi-labeler
void pointCloudExtraction::RegionExtractionT(vector<GNode> &points,
  Matrix4f& pose) const {
  Exteriors exteriors;
  Interiors interiors;
  thinLabeling(labelMap, exteriors, interiors);
  imwrite("CCM-IMG.png", 25*labelMap+5);
  Mat extImg = Mat::zeros(labelMap.size(), CV_8UC3);
  Mat intImg = Mat::zeros(labelMap.size(), CV_8UC3);
  for (int b = 0; b < exteriors.size(); b++)
    if (exteriors[b].size()) {
      Scalar color(rand()&255,rand()&255,rand()&255);
      drawContours(extImg, exteriors, b, color, 1, 8);
      if (interiors[b].size())
        for (int c = 0; c < interiors[b].size(); c++)
          if (interiors[b][c].size())
            drawContours(intImg, interiors[b], c, color, 1, 8);
    }
  imwrite("CCM-EXT.png",extImg);
  imwrite("CCM-INT.png",intImg);
  exit(0);
  // Mat intMask = Mat::ones(labelMap.size(), CV_8U);
  // for (int b = 0; b < interiors.size(); b++)
  //   if (interiors[b].size())
  //     drawContours(intMask, interiors, b, Scalar(0), FILLED, 8);

  for (int b = 0; b < exteriors.size(); b++)
    if (exteriors[b].size() > 40) { //minimum required == sqrt(area/pi)
      Moments M = moments(exteriors[b]);
      int area = M.m00; //contourArea(exteriors[b], intMask);
      if (area > 5000) {
        Point2i center(M.m10/M.m00, M.m01/M.m00); //contourCenter(exteriors[b], intMask);
        int lab = labelMap.at<uchar>(center);//labelMap.at<uchar>(exteriors[b][0]);
        if (lab != labelMap.at<uchar>(exteriors[b][0]))
          continue;//cout << '!';
        float cz = depthMap.at<float>(center);
        Vector4f point((center.x-cx)*cz/fx, (center.y-cy)*cz/fy, cz, 1);
        point = pose*point;
        addGNode(points, point, lab, 1.0f); //25
      }
    }
}*/
