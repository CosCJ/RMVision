

///装甲板检测

#include "ArmorDetector.hpp"
#include "opencv2/imgproc/imgproc.hpp"
//#include <kalman.h>
#include <iostream>
#include <queue>
#include <stdlib.h>


#ifndef SHOW_DEBUG_IMG
//define SHOW_DEBUG_IMG
#endif

#ifndef COUT_LOG
//#define COUT_LOG
#endif

#ifndef USE_NEON
#define USE_NEON
#endif

using namespace cv;
using namespace std;

//double start0 = getTickCount();
//        double finish0 = getTickCount();
//        cout<<"shijian: "<<(finish0 - start0)/getTickFrequency()*1000<<endl;

double ArmorDetector::distance_xy(cv::Point p1,cv::Point p2)
{
    return ((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y));
}



static inline bool RotateRectSort(RotatedRect a,RotatedRect b)
{
    return a.center.x < b.center.x ;
}


//对原图像进行通道分离，得到绿色通道图像，目标颜色通道图像，和二值化图像，复制给ArmorDetector的成员变量
void ArmorDetector::setImage(const cv::Mat & src){
//    imshow("src0",src);//**********
    _size = src.size();
    const cv::Point & last_result = _res_last.center;
    if(last_result.x == 0 || last_result.y == 0){
        _src = src;
        _dect_rect = Rect(0, 0, src.cols, src.rows);
    }

    int total_pixel = _src.cols * _src.rows;
    const uchar * ptr_src = _src.data;
    const uchar * ptr_src_end = _src.data + total_pixel * 3;

    _g.create(_src.size(), CV_8UC1);
    _ec.create(_src.size(), CV_8UC1);
    _max_color = cv::Mat(_src.size(), CV_8UC1, cv::Scalar(0));

    Mat src_gary;

    if(!_is_lost)
    {
            uchar *ptr_g = _g.data, *ptr_ec = _ec.data, *ptr_max_color = _max_color.data;
            if (_para.enemy_color == RED){
                for (; ptr_src != ptr_src_end; ++ptr_src, ++ptr_g, ++ptr_max_color, ++ptr_ec)	{
                    uchar b = *ptr_src;
                    uchar g = *(++ptr_src);
                    uchar r = *(++ptr_src);
                    *ptr_g = g;
                    *ptr_ec = r;
                    if (r > _para.min_light_gray)
                        *ptr_max_color = 255;
                }
            }
            else {
                for (; ptr_src != ptr_src_end; ++ptr_src, ++ptr_g, ++ptr_max_color, ++ptr_ec)	{
                    uchar b = *ptr_src;
                    uchar g = *(++ptr_src);
                    uchar r = *(++ptr_src);
                    *ptr_g = g;
                    *ptr_ec = b;
                    if (b > _para.min_light_gray)
                        *ptr_max_color = 255;
                }
            }
    }
    else
    {
        //大津法的好处在于得到完整的灯条
        cvtColor(_src,src_gary,CV_BGR2GRAY);     
        src_gary *= 1.4;//提高对比度
        threshold(src_gary,_max_color,0,255,THRESH_OTSU);
    }



    Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
    //  这一步的作用是用于对像素点不清晰的做一级过滤，减少在灯条处，装甲框不断变化。
    dilate(_max_color,_max_color,element);
    erode(_max_color,_max_color,element);


#ifdef SHOW_DEBUG_IMG
    _g = src_gary.clone();
    //cv::imshow("ec",_ec);
   // cv::imshow("g", _g);
    cv::imshow("_max_color", _max_color);
    waitKey(1);
#endif
}





void ArmorDetector::findContourInEnemyColor(
        cv::Mat & left, cv::Mat & right,
        vector<vector<Point2i> > &contours_left,
        vector<vector<Point2i> > &contours_right)
{
    // find contour in sub image of blue and red
    vector<vector<Point2i> > contours_br;
    vector<Vec4i> hierarchy;
    findContours(_max_color, contours_br, hierarchy, CV_RETR_EXTERNAL , CV_CHAIN_APPROX_SIMPLE);
    vector<vector<Point2i> >::const_iterator it = contours_br.begin();


    left = Mat::zeros(_max_color.size(), CV_8UC1);
    right = Mat::zeros(_max_color.size(), CV_8UC1);

#ifdef SHOW_DEBUG_IMG
    Mat _max_color_rgb;
    cvtColor(_max_color, _max_color_rgb, CV_GRAY2BGR);

    Mat rectrgb = _src.clone();

#endif
    while (it != contours_br.end()){
        Rect rect = cv::boundingRect(*it);
        //rectangle(dst, rect, Scalar(255, 255, 255), 1, 8, 0);
        if ((rect.height < _para.min_light_height) ||
            (rect.height > 15 && rect.width > 0.6 * rect.height + 0.5) ||
            (rect.height <= 15 && rect.width > 0.9 * rect.height)){
            ++it;
            continue;
        }


        //去除敌人颜色
        Rect b_rect1 = rect;
        if(broadenRect(b_rect1, 5, 5, _src.size()) == false) continue;
        Scalar m1 = mean(_src(b_rect1));
        if (_para.enemy_color == RED && (m1[0] > m1[2] || m1[1] > m1[2])){
            ++it;
            continue;
        }
        else if(_para.enemy_color == BLUE && (m1[2] > m1[0] || m1[1] > m1[0])){
            ++it;
            continue;
        }


#ifdef  SHOW_DEBUG_IMG
        //检查当前的目标灯条
        rectangle(rectrgb,b_rect1,Scalar(200,200,200),2,8);
        imshow("查看当前灯条",rectrgb);
#endif

        //去除小面积的杂质
        float lightContourArea = contourArea(*it);//
        if ((*it).size() <= 5 || lightContourArea < 20)
        {
            ++it;
            continue;
        }

        Mat roi = _max_color(rect);
        roi.copyTo(left(rect));
        roi.copyTo(right(rect));
        ++it;

#ifdef SHOW_DEBUG_IMG
        //圈出所有符合当前要求的灯条

        Scalar color(rand() & 255, rand() & 255, rand() & 255);
        rectangle(_max_color_rgb, rect, color, 2);
        char slope_str[15];
        sprintf(slope_str, "%d,%d", rect.width, rect.height);
        putText(_max_color_rgb, slope_str, Point(rect.x, rect.y), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.5, color, 1);
        imshow("_max_color_rgb",_max_color_rgb);

#endif
    }


    findContours(left, contours_left, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
    findContours(right, contours_right, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

#ifdef SHOW_DEBUG_IMG
    //当前找到的灯条,绘制出轮廓
    Mat show_l=Mat::zeros(_max_color.size(), CV_8UC1);
    Mat show_r =Mat::zeros(_max_color.size(), CV_8UC1);
    char slope_left[15];

   // cout<<"zhege  ";
    for(int i=0; i<contours_left.size(); i++)
    {
        if(contours_left[i].size()>5)
        {

            double angle = fitEllipse(contours_left[i]).angle;
            if(angle>140)
                angle = 180-angle;
          //  cout<<angle<<"  ";

            sprintf(slope_left, "%.2f", angle);
            putText(show_l, slope_left, contours_left[i][0], CV_FONT_HERSHEY_COMPLEX_SMALL, 0.5, Scalar(255), 1);
        }
        drawContours(show_l,contours_left,i,Scalar(255),1,8);
    }
   // cout<<endl<<endl;
    for(int i=0; i<contours_right.size(); i++)
    {
        drawContours(show_r,contours_right,i,Scalar(255),1,8);
    }
    imshow("contours_left",show_l);
    imshow("contours_right",show_r);
#endif

}

//根据左右灯柱的旋转矩形计算并返回装甲的旋转矩形
cv::RotatedRect ArmorDetector::boundingRRect(const cv::RotatedRect & left, const cv::RotatedRect & right){
    const Point & pl = left.center, & pr = right.center;
    Point2f center = (pl + pr) / 2.0;
    cv::Size2f wh_l = left.size;
    cv::Size2f wh_r = right.size;
    float width = POINT_DIST(pl, pr) + (wh_l.width + wh_r.width) / 2.0;
    float height = std::max(wh_l.height, wh_r.height);
    //float height = (wh_l.height + wh_r.height) / 2.0;
    float angle = std::atan2(right.center.y - left.center.y, right.center.x - left.center.x);
    return RotatedRect(center, Size2f(width, height), angle * 180 / CV_PI);




}

//调整拟合函数拟合成的旋转矩形，保证height>width，且其角度为锐角
RotatedRect ArmorDetector::adjustRRect(const RotatedRect & rect){
    const Size2f & s = rect.size;
    if (s.width < s.height)
        return rect;
    return RotatedRect(rect.center, Size2f(s.height, s.width), rect.angle + 90.0);
}

//在输入的可能的左右灯柱的轮廓中找到装甲的位置，并建立旋转矩形，添加到rects向量中
void ArmorDetector::findTargetInContours(const vector<vector<Point> > & contours_left,
                                         const vector<vector<Point> > & contours_right,
                                         vector<RotatedRect> & rects, std::vector<double> & score) {



    vector<RotatedRect> final_contour_rect_left, final_contour_rect_right;
    vector<double> score_left, score_right;
    vector<Point> points;


#ifdef  SHOW_DEBUG_IMG
    Mat contours_show_left, contours_show_right,contours_show_center;
    _src.copyTo(contours_show_left);
    _src.copyTo(contours_show_right);
    _src.copyTo(contours_show_center);


    Mat tempMat,tempMat1;
    _src.copyTo(tempMat);
    _src.copyTo(tempMat1);



#endif

    
    int index_left[1000],index_right[1000];
    int count=0;
    for (size_t i = 0; i < contours_left.size(); ++i){

        RotatedRect rrect = minAreaRect(contours_left[i]);
        rrect = adjustRRect(rrect);
        double angle = rrect.angle;


        //拟合椭圆至少需要6个点
        if (contours_left[i].size() < 6)
            continue;
            
        //保存符合左灯条的Rect和下标,中心点
        index_left[count++] = i;
        final_contour_rect_left.push_back(rrect);
        points.push_back(rrect.center);//保存灯条,尽量放入灯条,不要进入杂质
        score_left.push_back(angle);
      
#ifdef  SHOW_DEBUG_IMG
        //cout << "(angle)\t(" << angle << ")\n";
        Scalar color(rand() & 255, rand() & 255, rand() & 255);
        char slope_str[15];
        sprintf(slope_str, "%lf", fitEllipse(contours_left[i]).angle);
        putText(contours_show_left, slope_str, contours_left[i][0], CV_FONT_HERSHEY_COMPLEX_SMALL, 0.5, color, 1);
        drawContours(contours_show_left, contours_left, i, color, CV_FILLED, 8);
        imshow("debug_left",contours_show_left);
#endif


    }




    count=0;
    for (size_t i = 0; i < contours_right.size(); ++i){
        // fit the lamp contour as a eclipse
        RotatedRect rrect = minAreaRect(contours_right[i]);
        rrect = adjustRRect(rrect);
        double angle = rrect.angle;

        //拟合椭圆至少需要6个点
        if (contours_left[i].size() < 6)
            continue;
            
        index_right[count++] = i;
        final_contour_rect_right.push_back(rrect);
        score_right.push_back(angle);

#ifdef  SHOW_DEBUG_IMG
        //cout << "(angle)\t(" << angle << ")\n";
        Scalar color(rand() & 255, rand() & 255, rand() & 255);
        char slope_str[15];
        sprintf(slope_str, "%.1f", fitEllipse(contours_right[i]).angle);
        putText(contours_show_right, slope_str, contours_right[i][0], CV_FONT_HERSHEY_COMPLEX_SMALL, 0.5, color, 1);
        drawContours(contours_show_right, contours_right, i, color, CV_FILLED, 8);
        imshow("debug_right",contours_show_right);

#endif
    }


    Rect rect1,rect2,rect3;
    // using all the left edge and right edge to make up rectangles
    for (size_t i = 0; i < final_contour_rect_left.size(); ++i) {
        const RotatedRect & rect_i = final_contour_rect_left[i];
        const Point & center_i = rect_i.center;
        float xi = center_i.x;
        float yi = center_i.y;


        //rectangle(tempMat1,rect_i.boundingRect(),Scalar(255,0,0),2,8);

        double angle_first = fitEllipse(contours_left[index_left[i]]).angle;

        for (size_t j = 0; j < final_contour_rect_right.size(); j++) {
            const RotatedRect & rect_j = final_contour_rect_right[j];
            const Point & center_j = rect_j.center;
            float xj = center_j.x;
            float delta_h = xj - xi;

           if(center_i.x >= center_j.x)
                continue;

            rect1 = rect_i.boundingRect();
            rect2 = rect_j.boundingRect();
            rect3 = rect1 | rect2;


            double angle_second = fitEllipse(contours_right[index_right[j]]).angle;
            double w = abs(angle_first - angle_second);

#ifdef SHOW_DEBUG_IMG
            Scalar color(rand() & 255, rand() & 255, rand() & 255);
            char slope_str[15];
            sprintf(slope_str, "%.1f", w);
            putText(contours_show_right, slope_str, rect3.tl(), CV_FONT_HERSHEY_COMPLEX_SMALL, 0.5, color, 1);
#endif

            if(( w > 5  && w < 45)  || (180-w > 5 && w>145))  //6
            {
             //   cout<<"灯条的角度大于6了"<<endl;
                continue;
            }

           float LenDiff_ratio = abs(rect_i.size.height - rect_j.size.height) /
                        max(rect_i.size.height, rect_j.size.height);

            if (LenDiff_ratio > 0.3)
            {
            //   cout<<"第一个矩形LenDiff_ratio判断不合格"<<endl;
               // cout<<"LenDiff_ratio     "<<LenDiff_ratio<<endl;
                continue;
            }

            float dis = sqrt((rect_i.center.x - rect_j.center.x)*(rect_i.center.x - rect_j.center.x)) + ((rect_i.center.y - rect_j.center.y)*(rect_i.center.y - rect_j.center.y));
            float meanLen = (rect_i.size.height + rect_j.size.height) / 2;
            float yDiff = abs(rect_i.center.y - rect_j.center.y);
            float yDiff_ratio = yDiff / meanLen;
            float xDiff = abs(rect_i.center.x - rect_j.center.x);
            float xDiff_ratio = xDiff / meanLen;
            float ratio = dis / meanLen;

            if (yDiff_ratio > 2 ||
                xDiff_ratio < 0.5 ||
                ratio > 20||
                ratio < 0.35
               )

            {
                cout<<"第二个矩形判断不合格"<<endl;
                continue;
            }



            if (delta_h > _para.min_light_delta_h &&
                    delta_h < _para.max_light_delta_h ) {



#ifdef   SHOW_DEBUG_IMG
                  Scalar color(rand() & 255, rand() & 255, rand() & 255);
                char slope_str[15];
                sprintf(slope_str, "%f", w);
           //     putText(contours_show_right, slope_str, rect3.tl(), CV_FONT_HERSHEY_COMPLEX_SMALL, 2.5, color, 1);
#endif


                RotatedRect rect = boundingRRect(rect_i, rect_j);
                rects.push_back(rect);

            }
        }
}

#ifdef SHOW_DEBUG_IMG
     Scalar color(rand() & 255, rand() & 255, rand() & 255);
    char slope_str[15];
//    sprintf(slope_str, "%d", rects.size());
  //  putText(contours_show_center,slope_str,Point(100,100),FONT_HERSHEY_SIMPLEX,3,Scalar(255,23,0),8,8);
    imshow("contours_show_center",contours_show_center);
    imshow("4.contours_l", contours_show_left);
    imshow("4.contours_r", contours_show_right);
    //waitKey(60);
#endif


}

//从输入的一帧图像中检测装甲的位置，返回装甲所在位置的一个旋转矩形
cv::RotatedRect ArmorDetector::getTargetAera(const cv::Mat & src,std::vector<cv::RotatedRect> & rects){

    setImage(src);
    cv::Mat contrast_left, contrast_right;
    vector<vector<Point2i> > contours_left;
    vector<vector<Point2i> > contours_right;
    findContourInEnemyColor(contrast_left, contrast_right, contours_left, contours_right);
//以上的步骤是用于，找出灯条像素点


//    imshow("light left",contrast_left);
 //   imshow("light right",contrast_right);
  //  vector<RotatedRect> rects;
    vector<double> score;
    findTargetInContours(contours_left, contours_right, rects, score);
    RotatedRect final_rect;


    //去除侧面误识别
    double maxscore=0x3f3f3f3f;
    int ii=0;


    //距离上一帧最近的点

    for(int i=0; i<rects.size(); i++)
    {
        double dist = distance_xy(rects[i].center,_res_last.center);
        if(dist < maxscore)
        {
            maxscore = dist;
            ii = i;
        }
    }



    if(rects.size()>0)
    {

      //  return rects;
        double wh_ratio = rects[ii].size.width/rects[ii].size.height;
        if (wh_ratio < 3.6)
            _is_small_armor = true;
        else
            _is_small_armor = false;
        //_is_small_armor = is_small;
        final_rect = rects[ii];//
    }
     //   final_rect = chooseTarget(rects, score);
    else
    {
        final_rect = RotatedRect();
        _is_lost = true;
    }
    if(final_rect.size.width != 0){
        final_rect.center.x += _dect_rect.x;
        final_rect.center.y += _dect_rect.y;
        _res_last = final_rect;
        _lost_cnt = 0;
    }
    else{
        ++_lost_cnt;

        if (_lost_cnt < 3)
            _res_last.size =Size2f(_res_last.size.width * 2, _res_last.size.height * 1.5);
        else if(_lost_cnt == 6)
            _res_last.size =Size2f(_res_last.size.width * 1.5, _res_last.size.height * 1.5);
        else if(_lost_cnt == 12)
            _res_last.size =Size2f(_res_last.size.width * 1.5, _res_last.size.height * 1.5);
        else if(_lost_cnt == 18)
            _res_last.size =Size2f(_res_last.size.width * 1.5, _res_last.size.height * 1.5);
        else if (_lost_cnt > 60 )
            _res_last = RotatedRect();
    }
    return final_rect;
}
