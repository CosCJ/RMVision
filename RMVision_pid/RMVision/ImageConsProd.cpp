
/*******************************************************************************************************************
Copyright 2017 Dajiang Innovations Technology Co., Ltd (DJI)

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
documentation files(the "Software"), to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense, and / or sell copies of the Software, and
to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of
the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
IN THE SOFTWARE.
*******************************************************************************************************************/

#include "ImageConsProd.hpp"
#include "Predictor.hpp"
#include "RuneResFilter.hpp"
#include "AngleSolver.hpp"
#include "RMVideoCapture.hpp"
#include "serial.h"
#include "RemoteController.hpp"
#include "kalman.hpp"
#include"pid.hpp"
#include<iostream>
#include<fstream>
#include "ArmorDetector.hpp"
#include "Runedetector.hpp"

using namespace std;

//#define USE_VIDEO
#define USE_FILE
#define USE_RESULT 0

#define VIDEO_WIDTH  1280
#define VIDEO_HEIGHT 720


#define BUFFER_SIZE 1
volatile unsigned int prdIdx;
volatile unsigned int csmIdx;

double dist = 500;//初始化的距离，认为5m左右用长焦打击
bool videoformat = true;

struct ImageData {
    Mat img;
    unsigned int frame;
};

ImageData data[BUFFER_SIZE];


void cvtRect(const cv::RotatedRect & rect, const Point2f & center1, cv::RotatedRect & cvted_rect, const Point2f & center2, float scale){
    cv::Size s(rect.size.width * scale, rect.size.height * scale);
    cv::Point2f c = (rect.center - center1) * scale + center2;
    cvted_rect = cv::RotatedRect(c, s, rect.angle);
}


void ImageConsProd::ImageProducer(){

    // set input source and image size
#ifdef USE_VIDEO
    settings->save_result = 0;
    string video_name = "successed.avi";
    //string video_name = "/home/ubuntu/projects/RMVision/RMVision/20160823165244.mp4";
    VideoCapture cap(video_name); // open the default camera
    if (!cap.isOpened())  // check if we succeeded
        return;
#else

    RMVideoCapture cap1("/dev/video0", 3);
    cap1.setVideoFormat(VIDEO_WIDTH, VIDEO_HEIGHT, 1);
    cap1.setExposureTime(0, 10);//settings->exposure_time);
    cap1.startStream();
    cap1.info();
#endif

    while(1){

        while(prdIdx - csmIdx >= BUFFER_SIZE);
#ifdef USE_VIDEO
        cap >> data[prdIdx % BUFFER_SIZE].img;
#endif

        cap1 >> data[prdIdx % BUFFER_SIZE].img;//8mm
        videoformat = true;

#ifndef USE_VIDEO
        data[prdIdx % BUFFER_SIZE].frame = cap1.getFrameCount();
#else
        data[prdIdx % BUFFER_SIZE].frame++;
#endif
        ++prdIdx;
    }
}

void ImageConsProd::ImageConsumer(){
    Settings & setting = *settings;

    // load calibration parameter
    FileStorage fs(setting.intrinsic_file_480, FileStorage::READ);
    if (!fs.isOpened())	{
        cout << "Could not open the configuration file: \"" << setting.intrinsic_file_480 << "\"" << endl;
        return ;
    }
    Mat cam_matrix_480, distortion_coeff_480;
    fs["Camera_Matrix"] >> cam_matrix_480;
    fs["Distortion_Coefficients"] >> distortion_coeff_480;


    FileStorage fs1(setting.intrinsic_file_720, FileStorage::READ);
    if (!fs1.isOpened())	{
        cout << "Could not open the configuration file: \"" << setting.intrinsic_file_720 << "\"" << endl;
        return ;
    }
    Mat cam_matrix_720, distortion_coeff_720;
    fs1["Camera_Matrix"] >> cam_matrix_720;
    fs1["Distortion_Coefficients"] >> distortion_coeff_720;

    AngleSolver solver_480(cam_matrix_480, distortion_coeff_480, 21.6, 5.4, settings->scale_z_480, settings->min_detect_distance, settings->max_detect_distance);
    AngleSolver solver_720(cam_matrix_720, distortion_coeff_720, 21.6, 5.4, settings->scale_z, settings->min_detect_distance, settings->max_detect_distance);


    Point2f image_center_480 = Point2f(cam_matrix_480.at<double>(0,2), cam_matrix_480.at<double>(1,2));
    Point2f image_center_720 = Point2f(cam_matrix_720.at<double>(0,2), cam_matrix_720.at<double>(1,2));


    // parameter of PTZ and barrel
    const double overlap_dist = 1000000.0;
    const double barrel_ptz_offset_y = 3.3;
    const double ptz_camera_y = 2.8;
    const double ptz_camera_z = -3;
    //const double ptz_camera_z = -10.5;
    double theta = -atan((ptz_camera_y + barrel_ptz_offset_y)/overlap_dist);
    double r_data[] = {1,0,0,0,cos(theta),-sin(theta), 0, sin(theta), cos(theta)};

    //8mm的摄像头右边
    double t_data_480[] = {1, ptz_camera_y, ptz_camera_z}; // ptz org position in camera coodinate system

    //4mm的摄像头左边
    double t_data_720[] = {0.1, ptz_camera_y, ptz_camera_z}; // ptz org position in camera coodinate system
    Mat t_camera_ptz_480(3,1, CV_64FC1, t_data_480);
    Mat t_camera_ptz_720(3,1, CV_64FC1, t_data_720);
    Mat r_camera_ptz(3,3, CV_64FC1, r_data);
    solver_480.setRelationPoseCameraPTZ(r_camera_ptz, t_camera_ptz_480, barrel_ptz_offset_y);
    solver_720.setRelationPoseCameraPTZ(r_camera_ptz, t_camera_ptz_720, barrel_ptz_offset_y);

    AngleSolverFactory angle_slover;
    angle_slover.setTargetSize(21.6, 5.4, AngleSolverFactory::TARGET_ARMOR);
    angle_slover.setTargetSize(12.4, 5.4, AngleSolverFactory::TARGET_SAMLL_ATMOR);


    RuneDetector rune_detector;

    Predictor predictor;
    // load armor detector template
    ArmorDetector armor_detector(setting.armor);

    armor_detector.setPnPSlover(&solver_480, &solver_720);
    FilterZ filter_z(0.1);
    ArmorFilter armor_filter(7);


    // vars for debug use
    //LedController led("/sys/class/gpio/gpio158/value");
   // led.ledON();
    Mat src_csm;
    int t1 = 0, t2 = 0;
#ifdef USE_VIDEO
    setting.save_result = 0;
#endif
    VideoWriter vw, vw_src;
    if (USE_RESULT > 0){
        vw.open("webcam.avi", CV_FOURCC('M', 'J', 'P', 'G'), 30, cv::Size(VIDEO_WIDTH, VIDEO_HEIGHT), true);
        vw_src.open("webcam_src.avi", CV_FOURCC('M', 'J', 'P', 'G'), 30, cv::Size(VIDEO_WIDTH, VIDEO_HEIGHT), true);
        if (vw.isOpened() == false || vw_src.isOpened() == false){
            cout << "can't open *.avi file" << endl;
            return;
        }
    }



    Mat src;
    int frame_num = 0;
    int miss_detection_cnt = 0;
    int find_detection_cnt=0;


    bool flash_flag = false;

    //kalman滤波器
    KalmanFilterAngle kf(0, 0);//Differ from cv::KalmanFilter

    Point2f currentPoint(0,0);
    Mat kalmanPoint;
  //  Point2f anti_kalmanPoint(0,0);


#ifdef USE_FILE
    //文件操作

    int countfile = 0;
    ofstream fout("yaw.txt");
    ofstream foutk("kal.txt");
    if(!fout)
    {
        cout<<"could not load file"<<endl;
        return ;
    }
#endif



    double send_data[4] = {0};
    double x_angle=0.0,y_angle=0.0;



    PID_position pidx(6,0.5,30);
    PID_position pidy(1.4,0.0,0);

    double finih=0;
    double star=0;


    while(1){

        // waiting for image data ready
        while(prdIdx - csmIdx == 0);

        finih = getTickCount();

        cout<<(finih - star)*1000/getTickFrequency()<<endl;

        star = getTickCount();

        data[csmIdx % BUFFER_SIZE].img.copyTo(src);
        frame_num = data[csmIdx % BUFFER_SIZE].frame;
        ++csmIdx;



        if(USE_RESULT > 0 || setting.show_image ){
            t1 = cv::getTickCount();
            src.copyTo(src_csm);
        }

        RotatedRect rect,last_rect;
        std::vector<RotatedRect> rects;
       // double angle_x = 0.0, angle_y = 0.0;

        if (setting.mode == ARMOR_MODE){  // armor detection mode

            double start1 = getTickCount();
            if (videoformat == true){
                armor_detector.setPara(setting.armor);
                angle_slover.setSolver(&solver_480);
            }

             armor_detector.setPitchAngle(other_param->angle_pitch);

             rect = armor_detector.getTargetAera(src,rects);

             bool is_small = armor_filter.getResult(armor_detector.isSamllArmor());
             //std::cout<<"is_small:    "<<is_small<<std::endl;
             AngleSolverFactory::TargetType type = is_small ? AngleSolverFactory::TARGET_SAMLL_ATMOR : AngleSolverFactory::TARGET_ARMOR;

             if (angle_slover.getAngle(rect, type, x_angle, y_angle,
                                                           setting.bullet_speed, other_param->angle_pitch) == true){



                miss_detection_cnt = 0;
                find_detection_cnt++;


                float tmp = -1 * y_angle;
                float tmpx  = x_angle;
                x_angle = x_angle ;//+ angle_yaw;
                y_angle = -1*y_angle;// + angle_pitch;

                kalmanPoint = kf.run(x_angle,y_angle);

                cout<<kalmanPoint<<endl;
                foutk<<countfile<<" "<<(kalmanPoint.at<float>(0) - angle_yaw)*100<<" "<<x_angle*100<<"\n";
                countfile++;


                float pidx_increment,pidy_increment;
                if(find_detection_cnt > 4)
                {
                    pidx_increment = pidx.pid_control(x_angle/5);
                    pidy_increment = pidy.pid_control(y_angle);


                    currentPoint = Point2f(pidx_increment,pidy_increment);
                    kalmanPoint = kf.run(pidx_increment,pidy_increment);



                    send_data[0] = pidx_increment*100;//pidx_increment*100;//
                    send_data[1] = pidy_increment*100;////
                    fout<<countfile<<" "<<rect.center.x <<" "<<rect.center.y<<'\n';
                  //foutk<<countfile<<" "<<x_angle*100<<" "<<pidx_increment*100<<"\n";
                    foutk<<countfile<<" "<<pidx.e*100<<"\n";
                    countfile++;
                }
                else
                {
                    pidx.clear();
                    pidy.clear();
                    send_data[0] = x_angle * 100;
                    send_data[1] = y_angle * 100;
                }



              //  send_data[0] = x_angle * 100;
              send_data[1] = y_angle * 100;




                if(pidx_increment > 17 && abs(x_angle - pidx_increment)>15)
                {
                    pidx.clear();
                }
                if(pidy_increment > 17 && abs(y_angle - pidy_increment)>15)
                {
                    pidy.clear();
                }


            //    作为没开自瞄,但是一直识别
                if(abs(pidx.e - pidx.e_pre)<1 && abs(pidx.integral) > 300)
                {
                    send_data[0] = x_angle*100;
                    send_data[1] = y_angle*100;
                }


                //限制的太小会突然跟不上,
                if(send_data[0] > 1500)
                    send_data[0] = 1500;
                else if(send_data[0] < -1500)
                    send_data[0] = -1500;

                if(send_data[1] > 1000)
                    send_data[1] = 1000;
                else if(send_data[1] < -1000)
                    send_data[1] = -1000;


                sendXYZ(fd2car, send_data);
                double end1 = getTickCount();

                cout<<"检测的时间:"<<(end1 - start1)*1000/getTickFrequency()<<endl;

            }
            else {

                find_detection_cnt=0;
                ++miss_detection_cnt;
                send_data[0] = 0;
                send_data[1] = 0;
                send_data[2] = 0;
                sendXYZ(fd2car, send_data);
                if(miss_detection_cnt >2 )
                {
                    pidx.clear();
                    pidy.clear();
                }
                if (miss_detection_cnt > 10){
                    filter_z.clear();
                    pidx.clear();
                    pidy.clear();
                }
            }
        }// end armor detection mode
        else if(setting.mode == RUNE_MODE)
        {
#ifdef USE_VIDEO
            angle_slover.setSolver(&solver_480);//这是8mm的720p
            resize(src,src,Size(640,480));
            if(src.rows != 480)
            {
                //后面把大风车的分辨率改了吧，用480p的
                cout<<"大幅使用的相机不是720p"<<endl;
                exit(0);
            }
#endif



            armor_detector.setPara(setting.armor);
            angle_slover.setSolver(&solver_480);
           // double starttime = getTickCount();

            int bono;

            cv::RotatedRect rect = rune_detector.getTarget(src,bono);


            double angle_x,angle_y;
            if (angle_slover.getAngle(rect, AngleSolverFactory::TARGET_ARMOR, angle_x, angle_y,
                                      setting.bullet_speed, other_param->angle_pitch) == true){
                    send_data[0] = (angle_x ) * 100;
                    send_data[1] = (angle_y ) * 100;
                    send_data[2] = angle_slover.getSolver().position_in_camera.at<double>(2,0);
                }
        }
        else {

            miss_detection_cnt = 0;
            flash_flag = false;
            predictor.clear();
            filter_z.clear();
            armor_detector.reset();
        }



#ifndef USE_VIDEO
        if(setting.show_image > 0 || USE_RESULT > 0){
            // show center and result
            cv::Point2f & image_center = src_csm.rows == 720 ? image_center_720 : image_center_480;
            circle(src_csm, image_center, 3, CV_RGB(0, 255, 0), 2);

            Point2f vertices[4];
            rect.points(vertices);
            for (int i = 0; i < 4; i++){
                line(src_csm, vertices[i], vertices[(i + 1) % 4], CV_RGB(0, 255, 0), 3);
            }

            char str[30];
            sprintf(str, "%.1f, %.1f, %.1f, %.lf", send_data[0], send_data[1], send_data[2],send_data[3]);
            putText(src_csm, str, Point(10, 40), CV_FONT_HERSHEY_COMPLEX_SMALL, 2, CV_RGB(128, 255, 0), 1);
            t2 = cv::getTickCount();
         //   cout << "Consumer-Time: " << (t2 - t1) * 1000.0 / cv::getTickFrequency() << "ms   frame No.:" << frame_num << endl;
        }

        if (setting.show_image > 0){
            Mat src_show = src_csm;
            if (src_csm.rows == 720)
                resize(src_csm, src_show, Size(640,360));


           imshow("result", src_show);


#ifdef USE_VIDEO
            waitKey(0);
#else
            char key = waitKey(1);
            // debug usehuang
            if (key == 'a')
                setting.mode = RUNE_MODE;
            else if (key == 's')
                setting.mode = ARMOR_MODE;
            else if(key == 'e')
            {
                fout.close();
                foutk.close();

                vw.release();
                vw_src.release();
                exit(0) ;
            }
#endif
        }
#endif
        if (USE_RESULT > 0 && src.rows == 720){
         //   std::cout<<"rows :"<<src.rows<<"     cols"<<src.cols<<endl;
         //   cout<<"src_csm.rows"<<src_csm.rows<<"    col"<<src_csm.cols<<endl;
            vw << src;
           vw_src << src_csm;
        }
#ifdef USE_FILE

#endif
    }
}

