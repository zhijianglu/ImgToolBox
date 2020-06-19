#include "feature_tracker.h"

int FeatureTracker::n_id = 0;

bool inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)

        if (status[i])
            v[j++] = v[i];
    v.resize(j);

}

void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

void reduceVector(vector<double> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}


void FeatureTracker::redcordFeatureCnt(vector<int> &v, vector<uchar> status)
{

    int j = 0;
    for (int i = 0; i < int(v.size()); i++)  //遍历feature_cnt
        if (!status[i])                      //如果这个特征点即将被删除
            track_cnts.push_back(v[i]);      //记录追踪的次数

}


FeatureTracker::FeatureTracker()
{
    track_rates_cnt = 0;
    feature_direction = cv::Point2f(0,0);
}


void FeatureTracker::setMask()
{
    if (FISHEYE)
        mask = fisheye_mask.clone();
    else
        mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));



    // prefer to keep features that are tracked for long time
    //TODO 重新编码
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;
            //跟踪次数  +  点坐标   +   特征的id号
    for (unsigned int i = 0; i < forw_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(forw_pts[i], ids[i])));

    sort(cnt_pts_id.begin(),
         cnt_pts_id.end(),
         [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
             return a.first > b.first;
         });

    forw_pts.clear();
    ids.clear();
    track_cnt.clear();


    //求区间
    double maxValue=0;
    double minValue=0;
    double scale=0;
    double factor=0;

    if(pts_speed.size()>0){
        maxValue = *max_element(pts_speed.begin(),pts_speed.end());
        minValue = *min_element(pts_speed.begin(),pts_speed.end());
    }
    int r = 0;
    int idx = 0;
    for (auto &it : cnt_pts_id)
    {
        if (mask.at<uchar>(it.second.first) == 255)
        {
            forw_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);

//            double factor = 1 / log( max(2.8, 1000*  pts_speed[idx] ));

//            cout << "value   " << (pts_speed[idx] - minValue) / (maxValue - minValue+1e-10) << endl;

            if(MASK_TYPE == 0){
                r = MIN_DIST;
            }else{
                r = int(SCALE_FACTOR *exp(0.5 - (pts_speed[idx] - minValue) / (maxValue - minValue+1e-10)) * MIN_DIST);
            }

            cout << "r:-----------" << r << endl;
            cv::circle(mask, it.second.first, r, 0, -1);
        }
        idx++;
    }



    float edge_scale = 46;
    feature_direction *= edge_scale;
//    cout<<"feature_direction:  "<<feature_direction<<endl;



    if (USE_EDGE_MASK)
    {
        if (feature_direction.x < 0)
            mask.colRange(0, int(-feature_direction.x)).setTo(0);
        if (feature_direction.x > 0)
            mask.colRange(COL - int(feature_direction.x), COL).setTo(0);

        if (feature_direction.y < 0)
            mask.rowRange(0, int(-feature_direction.y)).setTo(0);
        if (feature_direction.y > 0)
            mask.rowRange(ROW - int(feature_direction.y), ROW).setTo(0);
    }


}

void FeatureTracker::addPoints()
{
    for (auto &p : n_pts)
    {
        forw_pts.push_back(p);
        ids.push_back(-1);
        track_cnt.push_back(1);
    }
}



void FeatureTracker::readImage(const cv::Mat &_img, double _cur_time)
{
    cv::Mat img;
    TicToc t_r;
    cur_time = _cur_time;  //当前帧时间

    if (EQUALIZE)
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        TicToc t_c;
        clahe->apply(_img, img);
        //ROS_DEBUG("CLAHE costs: %fms", t_c.toc());
    }
    else
        img = _img;

    if (forw_img.empty())
    {
        prev_img = cur_img = forw_img = img;
    }
    else
    {
        forw_img = img;
    }

    forw_pts.clear();



    if (cur_pts.size() > 0)
    {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;

        //提取光流
        cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);

        for (int i = 0; i < int(forw_pts.size()); i++)
            if (status[i] && !inBorder(forw_pts[i]))
                status[i] = 0;
        redcordFeatureCnt(track_cnt, status);//记录所有被删掉的特征点追踪总共被跟踪的次数
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);
        reduceVector(cur_un_pts, status);
        reduceVector(track_cnt, status);
        reduceVector(pts_speed, status);

        //ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
    }

    for (auto &n : track_cnt)
        n++;


    rejectWithF();

    //ROS_DEBUG("set mask begins");
    TicToc t_m;
    setMask();
    //ROS_DEBUG("set mask costs %fms", t_m.toc());

    //ROS_DEBUG("detect feature begins");
    TicToc t_t;
    int n_max_cnt = MAX_CNT - static_cast<int>(forw_pts.size());
    if (n_max_cnt > 0)
    {
        if (mask.empty())
            cout << "mask is empty " << endl;
        if (mask.type() != CV_8UC1)
            cout << "mask type wrong " << endl;
        if (mask.size() != forw_img.size())
            cout << "wrong size " << endl;
        cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 0.01, MIN_DIST, mask);
    }
    else
        n_pts.clear();
    //ROS_DEBUG("detect feature costs: %fms", t_t.toc());

    //ROS_DEBUG("add feature begins");
    track_rates_cnt += (float)forw_pts.size()/MAX_CNT;
 //    cout<<"tracked points:"<<forw_pts.size()<<"  rates:"<<(float)forw_pts.size()/MAX_CNT <<endl;
//    cout<<"n_pts.size()"<<n_pts.size()<<endl;

    TicToc t_a;
    addPoints();

    //ROS_DEBUG("selectFeature costs: %fms", t_a.toc());
    bool show_mask = true;

    if(show_mask){

        cv::Mat mask_img;

        cv::cvtColor(mask, mask_img, CV_GRAY2RGB);

        cv::Point2f center(COL/2,ROW/2);

        cv::circle(mask_img, center, 2, cv::Scalar(0, 0, 255), 20);

        cv::Point2f  tmp_prev_un_pts;

        tmp_prev_un_pts = (center - 2*feature_direction);

        cout << "tmp_prev_un_pts:" << tmp_prev_un_pts << endl;
//        cv::line(mask_img, center, tmp_prev_un_pts, cv::Scalar(255 , 0, 255), 5 , 8, 0);
//        cv::circle(mask_img, tmp_prev_un_pts, 2, cv::Scalar(0, 0, 255), 20);

        cv::arrowedLine(mask_img, center, tmp_prev_un_pts, cv::Scalar(0, 255, 0), 5, 8, 0, 0.2);



        for (auto &it : n_pts)
        {
            cv::circle(mask_img, it, 2, cv::Scalar(255, 0, 255), 2); //draw new features
        }

        if(SHOW_DEBUG_IMG){
            imshow("mask_img", mask_img);
            waitKey(1);
        }

    }

    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    cur_img = forw_img;
    cur_pts = forw_pts;

    undistortedPoints();

    prev_time = cur_time;
}

void FeatureTracker::rejectWithF()
{
    if (forw_pts.size() >= 8)
    {
        //ROS_DEBUG("FM ransac begins");
        TicToc t_f;
        vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_forw_pts(forw_pts.size());
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        int size_a = cur_pts.size();

        redcordFeatureCnt(track_cnt, status);//记录所有被删掉的特征点追踪次数
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(cur_un_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        reduceVector(pts_speed, status);

        //ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, forw_pts.size(), 1.0 * forw_pts.size() / size_a);
        //ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
    }
}

bool FeatureTracker::updateID(unsigned int i)
{
    if (i < ids.size())
    {
        if (ids[i] == -1)
            ids[i] = n_id++;
        return true;
    }
    else
        return false;
}

void FeatureTracker::readIntrinsicParameter(const string &calib_file)
{
    cout << "reading paramerter of camera " << calib_file << endl;
    m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}

void FeatureTracker::showUndistortion(const string &name)
{
    cv::Mat undistortedImg(ROW + 600, COL + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < COL; i++)
        for (int j = 0; j < ROW; j++)
        {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            m_camera->liftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
            //printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
        }
    for (int i = 0; i < int(undistortedp.size()); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + COL / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + ROW / 2;
        pp.at<float>(2, 0) = 1.0;
        //cout << trackerData[0].K << endl;
        //printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
        //printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < ROW + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < COL + 600)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
        else
        {
            //ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
        }
    }
    cv::imshow(name, undistortedImg);
    cv::waitKey(0);
}

void FeatureTracker::undistortedPoints()
{
    cur_un_pts.clear();
    pts_speed.clear();
    cur_un_pts_map.clear();
    //cv::undistortPoints(cur_pts, un_pts, K, cv::Mat());
    feature_direction = cv::Point2f(0, 0);

    int valid_cnt = 0;

    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {
        Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
        Eigen::Vector3d b;
        m_camera->liftProjective(a, b);
        cur_un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
        cur_un_pts_map.insert(make_pair(ids[i], cv::Point2f(b.x() / b.z(), b.y() / b.z())));
        //printf("cur pts id %d %f %f", ids[i], cur_un_pts[i].x, cur_un_pts[i].y);
    }
    // caculate points velocity
    if (!prev_un_pts_map.empty())
    {
        double dt = cur_time - prev_time;
        pts_velocity.clear();
        for (unsigned int i = 0; i < cur_un_pts.size(); i++)
        {
            if (ids[i] != -1)
            {
                std::map<int, cv::Point2f>::iterator it;
                it = prev_un_pts_map.find(ids[i]);
                if (it != prev_un_pts_map.end())
                {
                    double dx = (cur_un_pts[i].x - it->second.x);
                    double dy = (cur_un_pts[i].y - it->second.y);

                    double v_x = dx / dt;
                    double v_y = dy / dt;
                    feature_direction += cv::Point2f(v_x, v_y);
                    valid_cnt++;
                    pts_velocity.push_back(cv::Point2f(v_x, v_y));
                    pts_speed.push_back(sqrt(dx * dx + dy * dy));
                }
                else{
                    pts_velocity.push_back(cv::Point2f(0, 0));
                    pts_speed.push_back(0);
                }
            }
            else
            {
                pts_velocity.push_back(cv::Point2f(0, 0));
                pts_speed.push_back(0);
            }
        }
    }
    else
    {
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            pts_velocity.push_back(cv::Point2f(0, 0));
            pts_speed.push_back(0);
        }
    }
    feature_direction /= valid_cnt;
    prev_un_pts_map = cur_un_pts_map;
}

