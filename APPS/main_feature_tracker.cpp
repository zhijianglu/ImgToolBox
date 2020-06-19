//
// Created by lab on 20-6-2.
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
#include <feature_tracker/feature_tracker.h>
#include "/home/lab/cpp_lib/getfile.h"
#include "io_tools.h"
#include "parameters.h"
using namespace cv;
using namespace std;
string config_path = "../config/euroc_tracker_config.yaml";
string data_path = "/media/lab/S_disk/data/euroc/MH05/mav0/cam0/data";


int main(){
    //---------------------------------
    readParameters(config_path);

    FeatureTracker trackerData;
    FeatureTracker trackerData_bak;

    trackerData.readIntrinsicParameter(config_path);
    trackerData_bak = trackerData;

    std::vector<std::string> files;
    if (getdir(data_path, files) >= 0) {
        printf("found %d image files in folder %s!\n", (int) files.size(), data_path.c_str());
    } else if (getFile(data_path, files) >= 0) {
        printf("found %d image files in file %s!\n", (int) files.size(), data_path.c_str());
    } else {
        printf("could not load file list! wrong path / file?\n");
        return -1;
    }

    int image_count = files.size();

    if(SHOW_DEBUG_IMG) namedWindow("Images",CV_WINDOW_FREERATIO);

    int key;
    int savedImgCnt = 0;
    int frameCnt = 0;

    ofstream file("./mindist.txt",ios::out);

    for (int k = 0; k < 10; ++k)
    {
        SCALE_FACTOR = 0.5;
        for (int l = 0; l < 10; ++l)
        {
            trackerData = trackerData_bak;

            Mat frame;
            for (int img_idx = 0; img_idx < files.size(); ++img_idx)
            {
                frame = imread(files[img_idx], CV_LOAD_IMAGE_UNCHANGED);

                double dStampSec = img_idx * (1.0 / 30.0);

                cout << "--------------------------------frame NO." << img_idx << endl;
                trackerData.readImage(frame, dStampSec);
                for (unsigned int i = 0;; i++)
                {
                    bool completed = false;
                    completed |= trackerData.updateID(i);
                    if (!completed)
                        break;
                }

                cv::cvtColor(frame, frame, CV_GRAY2RGB);
                cout << trackerData.cur_pts.size() << " fratures tracked" << endl;
                for (unsigned int j = 0; j < trackerData.cur_pts.size(); j++)
                {

                    double len = std::min(1.0, 1.0 * trackerData.track_cnt[j] / WINDOW_SIZE);

                    cv::circle(frame, trackerData.cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);

                    //draw speed line

                    Vector2d tmp_cur_un_pts(trackerData.cur_un_pts[j].x, trackerData.cur_un_pts[j].y);
                    Vector2d tmp_pts_velocity(trackerData.pts_velocity[j].x, trackerData.pts_velocity[j].y);
                    Vector3d tmp_prev_un_pts;
                    tmp_prev_un_pts.head(2) = tmp_cur_un_pts - 0.10 * tmp_pts_velocity;
                    tmp_prev_un_pts.z() = 1;
                    Vector2d tmp_prev_uv;
                    trackerData.m_camera->spaceToPlane(tmp_prev_un_pts, tmp_prev_uv);
                    cv::line(frame,
                             trackerData.cur_pts[j],
                             cv::Point2f(tmp_prev_uv.x(), tmp_prev_uv.y()),
                             cv::Scalar(255, 0, 0),
                             1,
                             8,
                             0);

                    char name[10];
                    sprintf(name, "%d", trackerData.ids[j]);
                    cv::putText(frame,
                                name,
                                trackerData.cur_pts[j],
                                cv::FONT_HERSHEY_SIMPLEX,
                                0.5,
                                cv::Scalar(0, 0, 0));
                }

                if(SHOW_DEBUG_IMG)
                {
                    cv::imshow("Images", frame);
                    cv::waitKey(0);
                }

            }

            cout << "===============================summary============================" << endl;
            double track_rate = trackerData.track_rates_cnt / files.size();
            cout << "mean track rate:           " << track_rate << endl;

            int sum = accumulate(trackerData.track_cnts.begin(), trackerData.track_cnts.end(), 0);
            double mean = (double) sum / (double) trackerData.track_cnts.size();
            cout << "mean track frame per point: " << mean << endl;

            file << MIN_DIST << " " << SCALE_FACTOR << " " << track_rate << " " << mean << endl;

            //TODO:update parameters
            SCALE_FACTOR += 0.03;
        }
        MIN_DIST++;
    }

    file.close();
}
