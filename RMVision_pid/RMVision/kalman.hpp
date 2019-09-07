#ifndef KALMAN_HPP
#define KALMAN_HPP

#include<opencv2/opencv.hpp>
class KalmanFilterAngle
{
public:
    KalmanFilterAngle(int x, int y):
        KF_(4, 2)
        /*
        KalmanFilter( int dynamParams, int measureParams, int controlParams = 0, int type = CV_32F )
        "dynamParams = 4": 4*1 vector of state (x, y, delta x, delta y)
        "measureParams = 2": 2*1 vector of measurement (x, y)
        */
    {
            measurement_ = Mat::zeros(2, 1, CV_32F);// (x, y)
            KF_.transitionMatrix = (Mat_<float>(4, 4) << 1, 0, 1, 0,//**Latter 1: Larger, faster regression
                                                         0, 1, 0, 1,//**Latter 1: Larger, faster regression
                                                         0, 0, 1, 0,
                                                         0, 0, 0, 1);
            setIdentity(KF_.measurementMatrix, Scalar::all(1));
            setIdentity(KF_.processNoiseCov, Scalar::all(120000));//Q 50000   //**10: Larger, slower regression
            setIdentity(KF_.measurementNoiseCov, Scalar::all(1e4));//R   1e7     //1: Larger, quicker regression
            setIdentity(KF_.errorCovPost, Scalar::all(1));

            std::cout<<KF_.measurementNoiseCov<<std::endl;
            
            //纯pnp解算的角度进入kalman,的参数Q=1000,R=1e7

            //输入值为pnp角度,输出为kalman的参数Q = 1e5 R = 1e7

            KF_.statePost = (Mat_<float>(4, 1) << x, y, 0, 0);//Ensure beginner is default value
    }


    cv::Mat run(float x, float y)
    {
        cv::Mat prediction = KF_.predict();
      //Point2f predict_pt = Point2f(prediction.at<float>(0),prediction.at<float>(1));

        measurement_.at<float>(0, 0) = x;
        measurement_.at<float>(1, 0) = y;

        KF_.correct(measurement_);

        return prediction;
    }
private:
    Mat measurement_;
    cv::KalmanFilter KF_;//Differ from Kalman_example::KalmanFilter

};





#endif // KALMAN_HPP

