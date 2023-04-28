#include "nodes.hpp"

#include <iostream>             //cout

using namespace std;

/*int findColor(uint8_t r, uint8_t g, uint8_t b, const Mat& colors) {
  for (int i = 0; i < colors.rows; i++)
    if (colors.ptr(i)[0]==r && colors.ptr(i)[1]==g && colors.ptr(i)[2]==b)
      return i;
  return -1; //RGB2INT(r,g,b)
}*/

unsigned int findColor(int lab,
  const unsigned char colors[], const unsigned int rows) {
  if (lab < 0 || lab > 0xFFFFFF) //impossible color
    return 0;
  const unsigned char r{INT2R(lab)};
  const unsigned char g{INT2G(lab)};
  const unsigned char b{INT2B(lab)};
  for (unsigned int i = 0; i < rows; i++)
    if (colors[3*i]==r && colors[3*i+1]==g && colors[3*i+2]==b)
      return i+1;
  return 0; //lab+1: 0 is unknown
}

void checkColors(vector<SNode>& snodes, const unsigned char colors[],
  const unsigned int rows, bool channels) {
  if (channels) //convert the rgb values to indices in the palette if possible
    for (SNode& snode: snodes) {
      const float lab{(float)findColor((int)snode.front()[NODE_L],colors,rows)};
      for (GNode& node: snode)
        node[NODE_L] = lab; //0 is unknown
    }
  else //otherwise simply check if the index is correct
    for (SNode& snode: snodes)
      if ((int)snode.front()[NODE_L] > rows ||
          (int)snode.front()[NODE_L] < 0) //color index is out of bounds
        for (GNode& node: snode)
          node[NODE_L] = 0;
}

void checkColors(vector<RBlob>& blobs, const unsigned char colors[],
  const unsigned int rows, bool channels) {
  if (channels) //convert the rgb values to indices in the palette if possible
    for (RBlob& blob: blobs) {
      const float lab((float)findColor((int)blob[BLOB_L], colors, rows));
      blob[BLOB_L] = lab; //0 is unknown
    }
  else //otherwise simply check if the index is correct
    for (RBlob& blob: blobs)
      if ((int)blob[BLOB_L] > rows ||
          (int)blob[BLOB_L] < 0) //color index is out of bounds
        blob[BLOB_L] = 0;
}

/*vector<SNode> gnodesToSnodes(const vector<GNode>& nodes) {
  vector<SNode> snodes;//(nodes.size()); //as many snodes as nodes
  snodes.reserve(nodes.size());
  for (const GNode& node: nodes) {
    snodes.emplace_back(SNode({GNode(node.size())})); //create an snode containing 1 node with the correct number of properties
    for (int j = 0; j < node.size(); j++)
      snodes.back().back()[j] = node[j];
  }
  return snodes;
}*/

/*vector<SNode> stripSNodes(const vector<SNode>& snodes) {
  vector<SNode> stripped;
  stripped.reserve(snodes.size());
  for (const SNode& snode: snodes)
    stripped.emplace_back(SNode({snode.front()}));
  return stripped;
}*/

unsigned int totalSize(const vector<SNode>& snodes) {
  unsigned int size{0};
  for (const SNode& snode: snodes)
    size += snode.size();
  return size;
}

unsigned int lastProperty(const vector<SNode>& snodes, const unsigned char i) {
  unsigned int last{0};
  for (const SNode& snode: snodes)
    for (const GNode& gnode: snode)
      if (gnode[i] > last) last = gnode[i];
  return last;
}

unsigned int lastInstance(const vector<SNode>& snodes) {
  return lastProperty(snodes, NODE_I);
}

unsigned int lastLabel(const vector<SNode>& snodes) {
  return lastProperty(snodes, NODE_L);
}

float adjustedRandIndex(const vector<SNode>& snodes) {
  const unsigned int rows{(unsigned int)snodes.size()};
  const unsigned int cols{lastInstance(snodes)+1}; //1 column padding
  const unsigned int n{totalSize(snodes)};

  if (cols < 2)
    return -2;

  vector<unsigned int> counts(rows*cols);
  vector<unsigned int> a(rows);
  vector<unsigned int> b(cols);
  for (unsigned int i = 0; i < rows; ++i) {
    a[i] = snodes[i].size();
    for (const GNode& gnode: snodes[i]) {
      ++counts[i*cols+gnode[NODE_I]];
      ++b[gnode[NODE_I]];
    }
  }

  unsigned int sN2{0};
  for (const int& x : counts)
    sN2 += x*x;
  sN2 = (sN2-n)/2;

  unsigned int sA2{0};
  for (const unsigned int& ai : a)
    sA2 += ai*ai;
  sA2 = (sA2-n)/2;
  //if larger than sN2 : fusion

  unsigned int sB2{0};
  for (const unsigned int& bj : b)
    sB2 += bj*bj;
  sB2 = (sB2-n)/2;
  //if larger than sN2 : fission

  const float right(2.0f*(float)sA2*(float)sB2/(float)(n*(n-1)));
  return ((float)sN2-right)/((float)(sA2+sB2)/2.0f-right);
}

void stripSNodes(vector<SNode>& snodes) {
  for (SNode& snode: snodes)
    if (!snode.empty()) //be careful not to create new nodes
      snode.resize(1);  //only keep the first one
}

void print(const Graph& snodes) {
  for (const SNode& snode : snodes) {
    for (const GNode& node : snode) {
      for (const float& x : node)
        cout << ' ' << x;
      cout << endl;
    }
    cout << endl;
  }
}

void mergeSNodes(vector<SNode> &snodes, const int to, const int from) {
  if (to == from) return; //avoid issues and simplify looping
  snodes[to].insert(snodes[to].end(), snodes[from].begin(), snodes[from].end());
  snodes.erase(snodes.begin()+from);
}

#define DISTU(x,y,z,u,v,w) ((u)-(x))*((u)-(x))+((v)-(y))*((v)-(y))+((w)-(z))*((w)-(z))
#define DISTB(b,n) DISTU(b[BLOB_X],b[BLOB_Y],b[BLOB_Z],n[NODE_X],n[NODE_Y],n[NODE_Z])
#define DISTG(m,n) DISTU(m[NODE_X],m[NODE_Y],m[NODE_Z],n[NODE_X],n[NODE_Y],n[NODE_Z])

float distSq(const GNode& p, const GNode& q) {
  return DISTG(p,q);
}

float distSq(const SNode& s, const GNode& q) {
  float res{-1};
  for (const GNode& p : s) {
    const float dist{DISTG(p,q)};
    if (dist < res || res < 0) res = dist;
  }
  return res;
}

float distSq(const GNode& p, const SNode& t) {
  return distSq(t,p);
}

float distSq(const SNode& s, const SNode& t) {
  float res{-1};
  for (const GNode& q : t) {
    const float dist{distSq(s,q)};
    if (dist < res || res < 0) res = dist;
  }
  return res;
}

int addNode(vector<SNode> &snodes, const RBlob& blob,
  const bool legacy, const float thr, const bool real, const bool merge) {

  GNode new_node({ blob[BLOB_X], blob[BLOB_Y], blob[BLOB_Z], blob[BLOB_L],
    blob[BLOB_F], blob[BLOB_V1], blob[BLOB_U1],
    blob[BLOB_V2]-blob[BLOB_V1]+1, blob[BLOB_U2]-blob[BLOB_U1]+1,
    blob[BLOB_C], blob[BLOB_I]});
  int id(0);
  if (blob[BLOB_I] < 0 || real) { //search object by distance threshold
    vector<int> ids;
    if (merge)
      ids.reserve(snodes.size());
    for (SNode& snode : snodes) {
      if (blob[BLOB_L] == (int)snode.front()[NODE_L])
        if (legacy) {
          if (DISTB(blob,snode.front()) < thr) {
            if (merge)
              snode.push_back(new_node);
            return id; //merge (i.e. no new node)
          }
        } else {
          for (const GNode& node : snode)
            if (DISTB(blob,node) < thr)
              if (merge) {
                ids.push_back(id);
                break; //merge is enabled: search for another matching SNode
              } else {
                snode.push_back(new_node);
                return id; //merge is disabled: return after one matching SNode
              }
        }
      id++;
    }
    if (merge) { //test made for clarity (ids is empty if merge is false)
      if (ids.size() > 1) //need to merge the snodes
        for (const int& from: ids)
          mergeSNodes(snodes, ids[0], from); //the first SNode is the recipient
      if (ids.size() > 0) {
        snodes[ids[0]].emplace_back(new_node);
        return ids[0];
      }
    }
  } else //look if object instance was previously found
    for (SNode& snode : snodes) {
      if (blob[BLOB_L] == (int)snode.front()[NODE_L] && blob[BLOB_I] == (int)snode.front()[NODE_I]) {
        if (!legacy || (legacy && merge))
          snode.push_back(new_node);
        return id; //merge is disabled: return after one matching SNode
      }
      id++;
    }
  snodes.emplace_back(SNode({new_node})); //no SNode was found, create new one
  return snodes.size()-1;
}

int addBlob(vector<SNode> &snodes, const RBlob& blob,
  const bool legacy, const float thr, const bool real, const bool merge) {

  GNode new_node({ blob[BLOB_X], blob[BLOB_Y], blob[BLOB_Z], blob[BLOB_L],
    blob[BLOB_F], blob[BLOB_V1], blob[BLOB_U1],
    blob[BLOB_V2]-blob[BLOB_V1]+1, blob[BLOB_U2]-blob[BLOB_U1]+1,
    blob[BLOB_C], blob[BLOB_I]});
  int id{0};
  if (blob[BLOB_I] < 0 || real) { //search object by distance threshold
    vector<int> ids;
    if (merge)
      ids.reserve(snodes.size());
    for (SNode& snode : snodes) {
      if (blob[BLOB_L] == (int)snode.front()[NODE_L]) //check label
        for (const GNode& node : snode)
          if (legacy) { //in legacy mode we only check the first node
            if (DISTB(blob,snode.front()) < thr) {
              if (merge)
                snode.push_back(new_node);
              return id; //merge (i.e. no new node)
            }
          } else {
            if (DISTB(blob,node) < thr) //check distance
              if (merge) {
                ids.push_back(id);
                break; //merge is enabled: search for another matching SNode
              } else {
                snode.push_back(new_node);
                return id; //merge is disabled: return after one matching SNode
              }
          }
      id++; //snode index
    }
    //size is 0 if merge is false
    if (ids.size() > 0) { //there is a match: add to the relevant snode
      if (ids.size() > 1) //many matches: merge the snodes
        for (const int& from: ids)
          mergeSNodes(snodes, ids.front(), from); //first SNode is recipient
      snodes[ids[0]].emplace_back(new_node);
      return ids[0];
    }
  } else //using ground truth, look if object instance was previously found
    for (SNode& snode : snodes) {
      if (blob[BLOB_L] == (int)snode.front()[NODE_L] && blob[BLOB_I] == (int)snode.front()[NODE_I]) {
        if (!legacy || (legacy && merge))
          snode.push_back(new_node);
        return id;
      }
      id++;
    }
  snodes.emplace_back(SNode({new_node})); //no SNode was found, create new one
  return snodes.size()-1;
}

void blobsToNodes(const vector<RBlob>& blobs,
  bool legacy, vector<SNode>& snodes, const int min_frame, const int max_frame,
  const int cnt_thr, const bool real) {

  for (const RBlob& blob : blobs) {
    if (blob[BLOB_F] < min_frame || (max_frame>=0 && blob[BLOB_F] > max_frame))
      continue;
    if (blob[BLOB_N] > cnt_thr)
      addNode(snodes, blob, legacy, 1.0f, real, true);//!legacy);
  }
}
