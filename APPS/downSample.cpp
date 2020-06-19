//
// Created by lab on 20-5-25.
//


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
#include "io_tools.h"
#include "parameters.h"
using namespace cv;
using namespace std;
string ConfigPath = "../config/config.yaml";

void ProcessVideo(){
    VideoCapture capture;
    Mat frame;

    frame= capture.open(DownSampleCfg.Input_PATH);

    if(!capture.isOpened())
    {
        printf("can not open ...\n");
        return;
    }
    namedWindow("video", CV_WINDOW_AUTOSIZE);

    int key;
    int savedImgCnt = 0;
    int frameCnt = 0;
    cout << "press 's' to save frame,'e' to quit program, other key to continue process" << endl;

    while (capture.read(frame) && key!= 'e')
    {

//        pyrDown(frame,frame,Size(DownSampleCfg.OutWidth,DownSampleCfg.Outheight));
//        pyrDown(frame,frame,Size(DownSampleCfg.OutWidth,DownSampleCfg.Outheight));

        cv::resize(frame, frame, Size(DownSampleCfg.OutWidth,DownSampleCfg.Outheight), cv::INTER_AREA);

        imshow("video", frame);
        key = waitKey(0);

        if(key == 's'){
            std::ostringstream frameName;
            frameName << std::setw(6) << std::setfill('0') << frameCnt;
            std::string outImg = DownSampleCfg.Output_PATH + "/frame-" + frameName.str() + ".jpg";
            savedImgCnt++;
            imwrite(outImg, frame);
        }
        frameCnt++;
    }
    capture.release();
}

void ProcessImages(){

    //---------------------------------
    std::string source = DownSampleCfg.Input_PATH;

    std::vector<std::string> files;
    if (getdir(source, files) >= 0) {
        printf("found %d image files in folder %s!\n", (int) files.size(), source.c_str());
    } else if (getFile(source, files) >= 0) {
        printf("found %d image files in file %s!\n", (int) files.size(), source.c_str());
    } else {
        printf("could not load file list! wrong path / file?\n");
        return;
    }

    int image_count = files.size();


    namedWindow("Images", CV_WINDOW_AUTOSIZE);

    int key;
    int savedImgCnt = 0;
    int frameCnt = 0;
    cout << "press 's' to save frame,'e' to quit program, other key to continue process" << endl;

    Mat frame;

    for (int img_idx = 0; img_idx < files.size(); ++img_idx)
    {
        frame = imread(files[img_idx]);


//        pyrDown(frame,frame,Size(DownSampleCfg.OutWidth,DownSampleCfg.Outheight));
//        pyrDown(frame,frame,Size(DownSampleCfg.OutWidth,DownSampleCfg.Outheight));

        cv::resize(frame, frame, Size(DownSampleCfg.OutWidth, DownSampleCfg.Outheight),cv::INTER_AREA);

        imshow("Images", frame);
        key = waitKey(0);

        if (key == 's')
        {
            std::ostringstream frameName;
            frameName << std::setw(6) << std::setfill('0') << frameCnt;
            std::string outImg = DownSampleCfg.Output_PATH + "/frame-" + frameName.str() + ".jpg";
            savedImgCnt++;
            imwrite(outImg, frame);
        }
        frameCnt++;
    }

}


int main(){

    readParameters(ConfigPath);

    if (appMode!=1) {
        cout<<"wrong mode in config file"<<endl;
        return -1;
    }
    if(DownSampleCfg.InPutDataType == 0) ProcessImages();

    if(DownSampleCfg.InPutDataType == 1) ProcessVideo();


}