//
// Created by lab on 2020/6/5.
//

#include <iostream>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <vector>
#include "/home/lab/cpp_lib/getfile.h"
#include "parameters.h"
using namespace std;
using namespace cv;
using namespace Eigen;
string ConfigPath = "../config/config.yaml";



int main(){

    readParameters(ConfigPath);

    if (appMode!=2) {
        cout<<"wrong mode in config file"<<endl;
        return -1;
    }

    //TODO read parameter

    double cam_fx=UndistortCfg.cam_fx;
    double cam_fy=UndistortCfg.cam_fy;
    double cam_cx=UndistortCfg.cam_cx;
    double cam_cy=UndistortCfg.cam_cy;

    double k1= UndistortCfg.k1;
    double k2= UndistortCfg.k2;
    double k3= UndistortCfg.k3;
    double r1= UndistortCfg.r1;
    double r2= UndistortCfg.r2;

    double downsample_factor = UndistortCfg.downsample_factor;
    double cam_width = UndistortCfg.cam_width;
    double cam_height = UndistortCfg.cam_height;

    cv::Mat input_K = (cv::Mat_<float>(3, 3) << cam_fx, 0.0f, cam_cx, 0.0f, cam_fy, cam_cy, 0.0f, 0.0f, 1.0f);
    cv::Mat input_D = (cv::Mat_<float>(1, 5) << k1, k2, k3,r1, r2);
    cout<<"Undistort parameter is:\n K: \n"<<input_K<<endl;
    cout<<"distort coeff: \n"<<input_D<<endl;

    float resize_fx, resize_fy, resize_cx, resize_cy;
    resize_fx = cam_fx * downsample_factor;
    resize_fy = cam_fy * downsample_factor;
    resize_cx = cam_cx * downsample_factor;
    resize_cy = cam_cy * downsample_factor;

    cv::Mat resize_K = (cv::Mat_<float>(3, 3) << resize_fx, 0.0f, resize_cx, 0.0f, resize_fy, resize_cy, 0.0f, 0.0f, 1.0f);
    resize_K.at<float>(2, 2) = 1.0f;
    int resize_width = cam_width * downsample_factor;
    int resize_height = cam_height * downsample_factor;

    cv::Mat undist_map1, undist_map2;
    cv::initUndistortRectifyMap(
        input_K,
        input_D,
        cv::Mat_<double>::eye(3, 3),
        resize_K,
        cv::Size(resize_width, resize_height),
        CV_32FC1,
        undist_map1, undist_map2);

    //TODO start undistorte

    std::string source = UndistortCfg.Input_PATH;
    std::vector<std::string> files;
    if (getdir(source, files) >= 0) {
        printf("found %d image files in folder %s!\n", (int) files.size(), source.c_str());
    } else if (getFile(source, files) >= 0) {
        printf("found %d image files in file %s!\n", (int) files.size(), source.c_str());
    } else {
        printf("could not load file list! wrong path / file?\n");
    }

    int image_count = files.size();
    int frameCnt = 0;
    int key = 0;
    for (int index = 0; index < image_count; ++index)
    {
        Mat imageInput = imread(files[index]);
        Mat undistorted_image;
        imshow("raw_input",imageInput);
        cv::remap(imageInput, undistorted_image, undist_map1, undist_map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
        imshow("undist_output",undistorted_image);

        key = waitKey(0);

        if (key == 's')
        {
            std::ostringstream frameName;
            frameName << std::setw(6) << std::setfill('0') << frameCnt;
            std::string outImg = DownSampleCfg.Output_PATH + "/frame-" + frameName.str() + ".jpg";
            imwrite(outImg, undistorted_image);
        }
        frameCnt++;
    }

    return 0;
}
