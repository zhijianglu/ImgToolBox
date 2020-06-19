#pragma once
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
using namespace std;
using namespace cv;
extern int ROW;
extern int COL;
extern int FOCAL_LENGTH;
const int NUM_OF_CAM = 1;


extern std::string IMAGE_TOPIC;
extern std::string IMU_TOPIC;
extern std::string FISHEYE_MASK;
extern std::vector<std::string> CAM_NAMES;
extern int MAX_CNT;
extern int MIN_DIST;
extern int WINDOW_SIZE;
extern int FREQ;
extern double F_THRESHOLD;
extern int SHOW_TRACK;
extern int USE_EDGE_MASK;
extern double SCALE_FACTOR;
extern int SHOW_DEBUG_IMG;
extern int MASK_TYPE;

extern int RECORD_IMG;
extern std::string OUT_PUT_PATH;

extern int STEREO_TRACK;
extern int EQUALIZE;
extern int FISHEYE;
extern bool PUB_THIS_FRAME;
extern int appMode;

struct calib_param {
    std::string Output_PATH;
    std::string Input_PATH;
    int board_width;
    int board_height;
    int gride_width;
    int gride_height;
};


struct down_sample_param {
    std::string Output_PATH;
    std::string Input_PATH;
    int InPutDataType;
    int OutPutDataType;
    int OutWidth;
    int Outheight;
};

struct undistort_param {
    std::string Output_PATH;
    std::string Input_PATH;

    int InPutDataType;
    int OutPutDataType;

    double cam_fx;
    double cam_fy;
    double cam_cx;
    double cam_cy;

    double k1;
    double k2;
    double k3;

    double r1;
    double r2;

    double downsample_factor;
    double cam_width;
    double cam_height;

};





extern calib_param CalibCfg;
extern down_sample_param DownSampleCfg;
extern undistort_param UndistortCfg;


void readParameters(std::string config_file);
