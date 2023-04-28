/*
Copyright © 2023 InterDigital
All Rights Reserved
This program contains proprietary information which is a trade secret/business secret of InterDigital R&D france is protected, even if unpublished, under applicable Copyright laws (including French droit d’auteur) and/or may be subject to one or more patent(s).
*/

#ifndef nodes_hpp
#define nodes_hpp

#include <vector>
typedef std::vector<float> RBlob; //2D blob
typedef std::vector<RBlob> Blobs; //blob list
typedef std::vector<float> GNode; //Graph node
typedef std::vector<GNode> SNode; //Super node
typedef std::vector<SNode> Graph; //Super graph (no topology)

#define NODE_X 0  //3D X
#define NODE_Y 1  //3D Y
#define NODE_Z 2  //3D Z
#define NODE_L 3  //label
#define NODE_F 4  //frame
#define NODE_V 5  //2D column
#define NODE_U 6  //2D row
#define NODE_W 7  //2D width
#define NODE_H 8  //2D height
#define NODE_C 9  //change
#define NODE_I 10 //instance

#define BLOB_F 0
#define BLOB_I 1
#define BLOB_C 2
#define BLOB_L 3   //==NODE_L
#define BLOB_N 4   //2D pixel count
#define BLOB_X 5
#define BLOB_Y 6
#define BLOB_Z 7
#define BLOB_DX 8  //3D X deviation
#define BLOB_DY 9  //3D Y deviation
#define BLOB_DZ 10 //3D Z deviation
#define BLOB_V1 11 //2D bounding box left
#define BLOB_U1 12 //2D bounding box top
#define BLOB_V2 13 //2D bounding box right
#define BLOB_U2 14 //2D bounding box bottom

#define RGB2INT(r,g,b) ( (((r)&0xFF) << 16) | (((g)&0xFF) << 8) | ((b)&0xFF) )
#define BGR2INT(d,i) RGB2INT((d)[3*(i)+2],(d)[3*(i)+1],(d)[3*(i)])
#define INT2R(i) (unsigned char)(((i) & 0x00FF0000) >> 16)
#define INT2G(i) (unsigned char)(((i) & 0x0000FF00) >> 8)
#define INT2B(i) (unsigned char)( (i) & 0x000000FF)

void checkColors(Graph& snodes, const unsigned char colors[],
  const unsigned int rows, bool channels);
void checkColors(Blobs&  blobs, const unsigned char colors[],
  const unsigned int rows, bool channels);

unsigned int totalSize(const Graph& snodes);
float adjustedRandIndex(const Graph& snodes);

void print(const Graph& snodes);

void mergeSNodes(Graph& snodes, const int to, const int from);
void stripSNodes(Graph& snodes);
void blobsToNodes(const Blobs& blobs, bool legacy, Graph& snodes,
  const int min_frame, const int max_frame,
  const int cnt_thr, const bool real = true);

//Node order: x, y, z, label, frame, v, u, w, h, change, instance
//            0, 1, 2,     3,     4, 5, 6, 7, 8,      9, 10

#endif /* nodes_hpp */
