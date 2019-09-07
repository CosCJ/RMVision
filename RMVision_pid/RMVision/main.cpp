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
#include "RemoteController.hpp"
#include "Settings.hpp"
#include "serial.h"

#include <thread>
#include <unistd.h>
#include "RMVideoCapture.hpp"

#include <iostream>

using namespace cv;
using namespace std;


void Video()
{
    RMVideoCapture cap("/dev/video1", 3);
    cap.setVideoFormat(1280, 720, 1);
    int exp_t = 16;
    cap.setExposureTime(0, exp_t);//settings->exposure_time);
    cap.startStream();
    cap.info();


    VideoWriter outputVideo;
     //outputVideo.open("哨兵.avi", -1, 30.0, S, true);
     cv::Size sWH = cv::Size((int)1280,
         720);
     outputVideo.open("哨兵.avi", CV_FOURCC('M','J','P','G'), 25.0, sWH);
     if (!outputVideo.isOpened()) {
             cout << "fail to open!" << endl;
             return ;
         }



    while(1){
        Mat src;
        cap >> src;
        imshow("src", src);
        char key = waitKey(20);

        outputVideo << src;


        if (key == 'w'){
            exp_t += 1;
            cap.setExposureTime(0, exp_t);//settings->exposure_time);
            cout << "current exp t:\t" << exp_t << endl;
        }

        else if(key == 's'){
            exp_t -= 1;
            cap.setExposureTime(0, exp_t);
            cout << "current exp t:\t" << exp_t << endl;
        }
    }
}


void watchVideo()
{

    VideoCapture cap1("webcam.avi");
    VideoCapture cap2("webcam_src.avi");
    Mat frame1,frame2;
    if(!cap1.isOpened())
    {
        cout<<"could7 not load "<<endl;
        return ;
    }

    while(1)
    {
       // char c = wiatkey(10);


       cap1>>frame1;
       cap2>>frame2;

       imshow("frame1",frame1);
       imshow("frame2",frame2);
       waitKey(1);

    }
}

int main(int argc, char * argv[]){
  //  adjustExposure();
    //Video();

    //watchVideo();
    char * config_file_name = "/home/coscj/projects/RMVision505/RMVision/calibration-param/param_config.xml";
    if (argc > 1)
        config_file_name = argv[1];
    Settings setting(config_file_name);
    OtherParam other_param;
    // communicate with car
    int fd2car = openPort("/dev/ttyUSB1");
    configurePort(fd2car);

    // start threads
    ImageConsProd image_cons_prod(&setting, &other_param, fd2car);
    std::thread t1(&ImageConsProd::ImageProducer, image_cons_prod); // pass by reference
    std::thread t2(&ImageConsProd::ImageConsumer, image_cons_prod);

    // debug use
    RemoteController controller(&setting, &other_param, fd2car);
    std::thread t3(&RemoteController::paraReceiver, controller);

    t1.join();
    t2.join();
    t3.join();
    close(fd2car);
}

