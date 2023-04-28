/*
Copyright © 2023 InterDigital
All Rights Reserved
This program contains proprietary information which is a trade secret/business secret of InterDigital R&D france is protected, even if unpublished, under applicable Copyright laws (including French droit d’auteur) and/or may be subject to one or more patent(s).
*/

#ifndef utils_hpp
#define utils_hpp

//descriptor types
#define WALK 0 //random walk
#define FAST 1 //walk histogram
#define FAS2 2 //merged walk histogram
#define NEWD 3 //histogram beta
#define ADJ_ 4 //adjacency
#define ADJN 6 //adjacency, weighted
#define BDJ_ 5 //adjacency, straight path (approximated)
#define BDJN 7 //adjacency, straight path (approximated), weighted
#define CDJ_ 8 //adjacency, straight path
#define CDJN 9 //adjacency, straight path, weighted

char char2dtype(const char c);
char dtype2char(const char type);

//Cloud types
#define FULLC 0
#define MEANC 1
#define PCA7C 2
#define GNODE 3

char char2ctype(const char c);
char ctype2char(const char type);

//Registration types
#define NORANSAC 0 //use all gnodes in matched snode pairs
#define ALLNODES 1 //one or less inlier per each gnode pairings in snode pair
#define ONEPERSN 2 //one or less inlier per source gnode in snode pair
#define ONEPERTN 3 //one or less inlier per target gnode in snode pair
#define ONEPERMN 4 //one or less inlier per snode pair
#define GENERICR 5 //gnodes are snodes

char char2rtype(const char c);
char rtype2char(const char type);

#define NOPLOT -1 //full pipeline, no display
#define CLOUDS 0  //full pipeline, show the two scene graphs with their matches
#define MODELS 1  //full pipeline, show the "registered" models
#define ALIGN1 2  //process sequence 1, show graph/cloud 1 over model 1
#define ALIGN2 3  //process sequences, show graph/cloud 2 over model 2
#define GTRUTH 4  //check transform ground truth, display coregistered models
#define ANIMAT 5  //show model 1 with poses 1
#define POINTS 6  //process sequences
#define TOPOLO 7  //process sequences, construct edges
#define TESTGT 8  //check transform ground truth
#define TEASER 9  //process both sequences, construct edges, show both graphs
#define ANIMA1 10 //process sequence 1, show cloud 1 over model 1 with poses 1
#define ANIMA2 11 //process sequences, show cloud 2 over model 2 with poses 2
#define FORCED 12 //process sequences, do not load existing cloud files
#define EDGES1 13 //process sequences, construct edges 1, show graph 1 over model 1
#define EDGES2 14 //process sequences, construct edges, show graph 2 over model 2
#define RBLOBS 15 //convert the sequences into blob files

char char2task(const char c);
char char2task(const char c, const char d);

#endif /* utils_hpp */
