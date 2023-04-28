//generate the correspondent rgb value of all labels in the segmentation image
cv::Mat Label_scannet((cv::Mat_<uchar>(40, 3) <<  //0,   0,   0,
                                174, 199, 232,		// wall           //0
                                152, 223, 138,		// floor
                                 31, 119, 180, 		// cabinet
                                255, 187, 120,		// bed
                                188, 189,  34, 		// chair
                                140,  86,  75,  	// sofa
                                255, 152, 150,		// table          //6 (7 in file)
                                214,  39,  40,  	// door
                                197, 176, 213,		// window
                                148, 103, 189,		// bookshelf
                                196, 156, 148,		// picture        //10
                                 23, 190, 207, 		// counter
                                178,  76,  76,
                                247, 182, 210,		// desk
                                 66, 188, 102,
                                219, 219, 141,		// curtain
                                140,  57, 197,
                                202, 185,  52,
                                 51, 176, 203,
                                200,  54, 131,
                                 92, 193,  61,                      //20
                                 78,  71, 183,
                                172, 114,  82,
                                255, 127,  14, 		// refrigerator
                                 91, 163, 138,
                                153,  98, 156,
                                140, 153, 101,
                                158, 218, 229,		// shower curtain
                                100, 125, 154,
                                178, 127, 135,
                                120, 185, 128,                      //30
                                146, 111, 194,
                                 44, 160,  44,  	// toilet
                                112, 128, 144,		// sink
                                 96, 207, 209,
                                227, 119, 194,		// bathtub
                                213,  92, 176,
                                 94, 106, 211,
                                 82,  84, 163,  	// otherfurn
                                100,  85, 144));                    //39

const uchar nyu41_to_nyu40[41] =
{1,2,3,4,5,6,7,8,9,10,
11,12,13,14,15,16,17,18,19,20,
21,22,23,24,25,26,27,28,29,30,
31,32,33,34,35,36,37,38,39,40,
0};

const uchar id_to_nyu40[1358] = {0,1,5,2,7,8,6,3,15,14,5,4,0,18,34,11,9,33,10,40,0,16,23,5,7,39,29,24,35,3,0,27,21,25,32,12,17,0,40,18,40,22,36,39,7,7,40,37,40,40,40,39,30,40,40,28,39,39,38,40,0,40,38,40,40,40,40,39,39,38,39,19,40,40,5,3,40,40,38,26,31,39,40,40,38,39,13,39,40,38,39,39,40,40,0,38,39,39,40,39,40,40,40,40,39,38,40,38,7,0,39,40,40,0,0,40,39,38,40,40,38,40,39,40,0,38,39,0,1,39,40,40,40,40,40,40,38,0,40,40,20,38,40,39,40,38,38,0,40,0,0,0,38,39,38,39,38,40,0,12,38,8,0,40,0,24,40,40,40,40,40,0,0,0,40,0,0,39,0,35,40,0,40,0,0,40,0,0,38,40,0,38,0,38,40,40,0,0,0,0,0,0,40,0,40,0,0,0,40,0,0,0,40,39,40,0,40,0,0,0,40,40,7,0,0,40,40,40,40,40,40,40,38,39,39,38,0,0,40,0,0,0,40,0,0,40,0,40,0,0,40,0,0,0,0,0,0,40,0,0,0,40,0,0,40,40,0,0,0,40,0,0,0,0,0,0,8,0,0,0,40,40,39,40,40,0,40,0,0,39,0,40,0,0,0,0,0,40,40,0,40,38,0,0,40,38,0,39,0,0,0,0,40,0,0,0,40,0,0,40,0,40,0,40,0,40,39,0,0,0,0,40,40,0,0,0,0,0,0,40,0,0,38,0,0,40,40,0,0,0,0,0,0,0,39,0,40,27,0,0,0,40,0,39,0,40,40,0,0,0,40,0,40,0,0,0,0,0,40,38,0,0,0,0,0,3,40,40,0,39,0,0,40,0,0,38,0,40,0,40,0,40,0,0,0,0,0,0,40,0,39,39,0,0,0,40,0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,40,0,40,40,40,0,0,0,37,0,0,0,0,0,0,0,38,0,40,0,40,0,0,0,0,0,0,40,0,40,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,37,0,0,0,39,0,0,0,40,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,37,0,0,0,0,0,0,38,0,0,0,0,40,0,0,0,0,40,0,39,0,0,0,40,0,0,0,0,0,0,0,0,0,0,29,0,0,0,0,0,40,0,0,0,0,40,0,0,0,0,40,0,0,0,0,40,40,40,0,0,39,0,0,8,37,40,40,0,0,0,0,0,0,0,0,39,0,0,0,0,0,0,0,0,0,40,40,40,0,0,0,0,0,40,0,0,0,0,0,0,0,40,0,38,0,0,40,0,0,0,0,0,0,0,0,40,0,40,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,40,0,0,0,0,0,8,0,0,0,0,0,0,0,29,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,29,0,0,0,0,0,0,0,0,40,0,0,0,0,0,0,40,0,0,0,40,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,40,0,0,0,0,0,0,40,0,0,0,0,0,0,40,0,0,0,38,0,0,40,0,0,0,0,0,0,0,0,0,0,0,0,40,0,40,35,40,0,0,0,0,0,0,0,0,0,0,0,0,0,0,39,0,0,0,0,0,0,0,0,0,0,40,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,40,0,40,0,0,0,0,40,0,40,0,0,0,0,0,0,0,0,0,40,40,3,40,40,0,40,0,0,39,0,0,0,0,0,40,40,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,39,0,0,0,0,0,40,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,5,0,0,0,0,0,0,0,40,0,0,0,0,0,0,0,0,0,0,0,0,0,40,0,0,0,0,0,0,0,0,0,0,0,40,0,0,40,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,40,0,0,0,0,0,0,0,1,40,0,0,0,0,0,0,40,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,40,0,0,0,38,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,40,0,0,0,0,0,0,0,40,0,0,40,40,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,40,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,40,0,0,0,0,0,0,0,0,0,0,24,38,0,0,0,0,0,0,0,0,37,0,0,0,0,0,0,0,0,0,0,40,0,0,0,0,0,0,0,0,0,0,0,0,0,0,40,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,40,39,0,0,0,0,38,0,0,40,40,0,0,0,0,0,0,0,0,40,0,0,0,0,0,0,0,0,0,35,0,0,0,0,0,0,0,0,0,0,12,0,0,0,0,0,0,40,3,1,40,8,38,39,40,38,40,3,40,40,40,39,38,38,39,40,40,40,5,40,40,40,11,40,40,4,38,7,40,40,40,21,40,40,40,38,39,40,1,40,40,40,8,39,40,40,40,23,40,38,38,40,11,38,38,40,40,40,40,22,40,39,40,40,37,40,37,40,40,40,40,40,37,40,40,40,39,1,39,29,40,40,40,40,40,40,37,29,31,40,39,33,40,40,40,40,40,40,37,40,40,20,40,40,39,29,40,40,40,40,40,40,40,40,40,40,40,15,40,40,40,40,37,40,39,5,40,40,40,38,40,37,40,29,40,21,40,21,40,40,40,40,40,40,38,40,40,6,40,40,15,40,40,40,40,40,3,40,40,40,40,40,40,40,40,40,40,40,40,15,40,40,5,40,38,39,21,40,40,8,40,2,40,4,40,39,40,40,40,7,37,40};