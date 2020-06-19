//
// Created by lab on 20-5-25.
//

#include "io_tools.h"


void
ORB_formate_calib(string outfile, const Mat &cameraMatrix, const Mat &distCoeffs, const Size &img_size)
{

    ofstream fout(outfile,ios::out);  /* ORBSLAM2格式保存标定结果的文件 */
    if(fout.is_open()){
        cout<<"saving result file:"<<outfile<<endl;
    }else{
        cout<<"WRONG PATH:"<<outfile<<endl;
        return;
    }

    fout.precision(15);
    fout<<"image width: "<< img_size.width <<endl;
    fout<<"image height: "<< img_size.height <<endl;

    fout<<"Camera.fx: "<< cameraMatrix.at<double>(0,0) <<endl;
    fout<<"Camera.fy: "<< cameraMatrix.at<double>(1,1) <<endl;
    fout<<"Camera.cx: "<< cameraMatrix.at<double>(0,2) <<endl;
    fout<<"Camera.cy: "<< cameraMatrix.at<double>(1,2) <<endl;

    fout<<endl;

    fout<<"Camera.k1: "<< distCoeffs.at<double>(0,0) <<endl;
    fout<<"Camera.k2: "<< distCoeffs.at<double>(0,1) <<endl;
    fout<<"Camera.k3: "<< distCoeffs.at<double>(0,2) <<endl;
    fout<<"Camera.r1: "<< distCoeffs.at<double>(0,3) <<endl;
    fout<<"Camera.r2: "<< distCoeffs.at<double>(0,4) <<endl;
//    fout<<""

}





