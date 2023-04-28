cv::Mat Label_changesim((cv::Mat_<uchar>(32, 3) <<  0,   0,   0,    // background
                                 81,  38,   0,    // column
                                 41,  36, 132,    // pipe
                                 25,  48,  16,    // wall (4)
                                131, 192,  13,    // beam
                                157, 131,  11,    // floor
                                 21,   1,   4,    // frame
                                 43, 110, 139,    // fence
                                 90, 104,   2,    // wire
                                 61, 101, 116,    // cable
                                  3,   2, 145,    // window
                                 33,   1, 220,    // railing
                                226,  37,  73,    // rail
                                 84, 144,   2,    // ceiling
                                  7,  25,  89,    // stair
                                 14, 178,  44,    // duct
                                  0, 214,   0,    // gril
                                140,   3,   0,    // lamp
                                 48,  14,   2,    // trash
                                  0,   9,  13,    // shelf
                                 73,   8,  15,    // door
                                194,  76,  70,    // barrel
                                 78,  53, 104,    // sign
                                137,   5,   0,    // box
                                155,  47,  90,    // bag
                                168,   8,  11,    // electric_box
                                166,  20, 160,    // vehicle
                                129,  61, 128,    // ladder
                                  9,  16, 155,    // canister
                                 34, 249,  86,    // extinguisher
                                 51,   1,  50,    // pallet
                                  1,  58,  15));    // hand_truck

const uchar* fuse = NULL;
