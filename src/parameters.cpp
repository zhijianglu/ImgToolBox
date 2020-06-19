#include "parameters.h"

std::string IMAGE_TOPIC;
std::string IMU_TOPIC;
std::vector<std::string> CAM_NAMES;
std::string FISHEYE_MASK;
int MAX_CNT;
int MIN_DIST;
int WINDOW_SIZE;
int FREQ;
double F_THRESHOLD;
int SHOW_TRACK;
int STEREO_TRACK;
int EQUALIZE;
int ROW;
int COL;
int FOCAL_LENGTH;
int FISHEYE;
bool PUB_THIS_FRAME;
int USE_EDGE_MASK;
double SCALE_FACTOR;
int appMode;
int SHOW_DEBUG_IMG;
int MASK_TYPE;

calib_param CalibCfg;
down_sample_param DownSampleCfg;
undistort_param UndistortCfg;
void readParameters(std::string config_file)
{

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    fsSettings["configMode"] >> appMode;

    if(appMode==0){
        cout<<"start loading calib parameters......"<<endl;
        fsSettings["calib_input_data_path"] >> CalibCfg.Input_PATH;
        fsSettings["calib_result_path"] >> CalibCfg.Output_PATH;
        CalibCfg.board_width=fsSettings["board_width"];
        CalibCfg.board_height=fsSettings["board_height"];

        CalibCfg.gride_width=fsSettings["gride_width"];
        CalibCfg.gride_height=fsSettings["gride_height"];

        cout<<"board_width: "<<CalibCfg.board_width<<endl;
        cout<<"board_height: "<<CalibCfg.board_height<<endl;
        cout<<"gride_width: "<<CalibCfg.gride_width<<endl;
        cout<<"gride_height: "<<CalibCfg.gride_height<<endl;

        cout<<"Input_PATH: "<<CalibCfg.Input_PATH<<endl;
        cout<<"Output_PATH: "<<CalibCfg.Output_PATH<<endl;
    }


    if(appMode==1){
        cout<<"start loading down sample parameters......"<<endl;

        fsSettings["down_sample_data_path"] >> DownSampleCfg.Input_PATH;
        fsSettings["down_sample_output_path"] >> DownSampleCfg.Output_PATH;
        DownSampleCfg.Outheight=fsSettings["down_sample_out_height"];
        DownSampleCfg.OutWidth=fsSettings["down_sample_out_width"];

        DownSampleCfg.InPutDataType=fsSettings["down_sample_input_data_type"];
        DownSampleCfg.OutPutDataType=fsSettings["down_sample_output_data_type"];

        cout<<"Outheight: "<<DownSampleCfg.Outheight<<endl;
        cout<<"OutWidth: "<<DownSampleCfg.OutWidth<<endl;

        std::string datatype = DownSampleCfg.InPutDataType==0 ? "images":"video";

        cout<<"input file: "<< datatype <<endl;

        datatype = DownSampleCfg.OutPutDataType==0 ? "images":"video";
        cout<<"out file: "<<datatype<<endl;

        cout<<"Input_PATH: "<<DownSampleCfg.Input_PATH<<endl;
        cout<<"Output_PATH: "<<DownSampleCfg.Output_PATH<<endl;
    }

    if(appMode==2){

        cout<<"start loading undistort parameters......"<<endl;

        fsSettings["undistort_data_path"] >> UndistortCfg.Input_PATH;
        fsSettings["down_sample_output_path"] >> UndistortCfg.Output_PATH;

        UndistortCfg.OutPutDataType=fsSettings["undistort_output_data_type"];
        UndistortCfg.InPutDataType=fsSettings["undistort_input_data_type"];

        UndistortCfg.cam_fx=fsSettings["undistort_cam_fx"];
        UndistortCfg.cam_fy=fsSettings["undistort_cam_fy"];
        UndistortCfg.cam_cx=fsSettings["undistort_cam_cx"];
        UndistortCfg.cam_cy=fsSettings["undistort_cam_cy"];

        UndistortCfg.k1=fsSettings["undistort_k1"];
        UndistortCfg.k2=fsSettings["undistort_k2"];
        UndistortCfg.k3=fsSettings["undistort_k3"];

        UndistortCfg.r1=fsSettings["undistort_r1"];
        UndistortCfg.r2=fsSettings["undistort_r2"];

        UndistortCfg.downsample_factor = fsSettings["undistort_downsample_factor"];
        UndistortCfg.cam_width = fsSettings["undistort_cam_width"];
        UndistortCfg.cam_height = fsSettings["undistort_cam_height"];

    }


    USE_EDGE_MASK = fsSettings["use_edge_mask"];
    SCALE_FACTOR = fsSettings["scale_factor"];
    SHOW_DEBUG_IMG = fsSettings["show_debug_img"];
    MASK_TYPE = fsSettings["mask_type"];

    cout<<"USE_EDGE_MASK"<<USE_EDGE_MASK<<endl;

    fsSettings["image_topic"] >> IMAGE_TOPIC;
    fsSettings["imu_topic"] >> IMU_TOPIC;
    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    FREQ = fsSettings["freq"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK = fsSettings["show_track"];
    EQUALIZE = fsSettings["equalize"];
    FISHEYE = fsSettings["fisheye"];

    CAM_NAMES.push_back(config_file);

    WINDOW_SIZE = 20;
    STEREO_TRACK = false;
    FOCAL_LENGTH = 460;
    PUB_THIS_FRAME = false;

    if (FREQ == 0)
        FREQ = 100;


    fsSettings.release();

}

