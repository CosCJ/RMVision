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

#include "Runedetector.hpp"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <iostream>


#ifndef SHOW_IMAGE
#define SHOW_IMAGE
#endif

using namespace cv;
using namespace std;



bool DoesRectangleContainPoint(RotatedRect rectangle, Point2f point) {

    //Get the corner points.
    Point2f corners[4];
    rectangle.points(corners);

    //Convert the point array to a vector.
    //https://stackoverflow.com/a/8777619/1997617
    Point2f* lastItemPointer = (corners + sizeof corners / sizeof corners[0]);
    vector<Point2f> contour(corners, lastItemPointer);

    //Check if the point is within the rectangle.
    double indicator = pointPolygonTest(contour, point, false);
    bool rectangleContainsPoint = (indicator >= 0);
    return rectangleContainsPoint;
}

cv::RotatedRect RuneDetector::getTarget(const cv::Mat & image,int bono){

    cv::Mat midImage;

    std::vector<cv::Mat> imgChannels;
    split(image,imgChannels);
    if(0)
    {
        midImage = imgChannels.at(0) - imgChannels.at(2);
    }
    else
    {
        midImage = imgChannels.at(2) - imgChannels.at(0);
    }
    threshold(midImage,midImage,100,255,CV_THRESH_OTSU);//CV_THRESH_OTSU,CV_THRESH_BINARY


 //   cout<<midImage.cols<<"      "<<midImage.rows<<endl;

    //为了使得findcontours更加完善
    line(midImage,Point(0,480-1),Point(640-1,480-1),Scalar(255),1,8);
    line(midImage,Point(0,0),Point(640-1,0),Scalar(255),1,8);
    line(midImage,Point(640-1,0),Point(640-1,480-1),Scalar(255),1,8);
    line(midImage,Point(0,8),Point(0,480-1),Scalar(255),1,8);

    cv::Mat kernel = cv::getStructuringElement(MORPH_RECT,Size(5,5));
  //  dilate(midImage,midImage,kernel);
  //erode(midImage,midImage,kernel);
#ifdef SHOW_IMAGE
    imshow("midImage", midImage);
    waitKey(6);
#endif
    vector<vector<Point2i>> contours;
    vector<Vec4i> hierarchy;

    findContours(midImage, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
#ifdef SHOW_IMAGE
    Mat show;
    Mat show_mid = Mat(midImage.size(),CV_8UC1,Scalar(0));
    Mat show_mid1 = Mat(midImage.size(),CV_8UC1,Scalar(255));
    image.copyTo(show);

\

    //第一次
   // std::vector<Point> points;//保存筛选后物体的中心点，便于后续的包含排查。
    std::vector<RotatedRect> rrects;//保存大风车的一整片子的rect


    for(int i = 0; i < contours.size(); ++i){
//floodFill(show_mid,contours[i][0],Scalar(0));

         //这里会保存一个图像的最大矩形，因为这个面积固定，所以直接可以排除
         //cout<<minAreaRect(contours[i]).boundingRect().area()<<"      ";
         drawContours(show_mid,contours,i,Scalar(255),-1,8,hierarchy);
         if(contours[i].size() > 5 && fitEllipse(contours[i]).boundingRect().area() != 307680)
            rrects.push_back(fitEllipse(contours[i]));
//       cv::RotatedRect rect_tmp = minAreaRect(contours[i]);
//       Point2f fourPoint2f[4];
//       rect_tmp.points(fourPoint2f);
//       for (int i = 0; i < 4; i++)//画矩形
//           line(show_mid, fourPoint2f[i], fourPoint2f[(i + 1) % 4], Scalar(255));
    }
    cout<<endl;
    imshow("show_mid_mid",show_mid);
    midImage.copyTo(show_mid1,show_mid);
    show_mid1 = ~show_mid1;



//    //对每一片叶子分割出来的三个东西，进行处理，筛选出来一个真正需要的装甲
//    Mat kernel1 = getStructuringElement(MORPH_RECT,Size(3,3));
//    morphologyEx(show_mid1, show_mid1, MORPH_OPEN, kernel1);


     imshow("show_mid1",show_mid1);
     cv::Mat show_mid1_cp = Mat(midImage.size(),CV_8UC1,Scalar(0));

     std::vector<RotatedRect> rrects_mini;//保存大风车的一整片子的rect
     std::vector<int> rrects_ct(rrects.size());//保存大风车的一整片子的rect

     findContours(show_mid1, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);//对融合后的叶子进行轮廓查找
     for(int i = 0; i < contours.size(); ++i){

         if(contours[i].size() <= 5)
             continue;

         //这个参数需要调试，得到
       //  cout<<minAreaRect(contours[i]).boundingRect().area()<<"      ";
         if(contours[i].size()>5 && fitEllipse(contours[i]).boundingRect().area() < 150)
             continue;

          drawContours(show_mid1_cp,contours,i,Scalar(255),1,8,hierarchy);
          rrects_mini.push_back(fitEllipse(contours[i]));
        //
//          char slope_str[15];
//          sprintf(slope_str, "%d", minAreaRect(contours[i]).boundingRect().area());
//   //       sprintf(slope_str, "%.2f", fourPoint2f1[i].x);
//      //    circle(show_mid1_cp,contours[i][0],1,Scalar(255,0,255),1,8);
//          putText(show_mid1_cp, slope_str, contours[i][0], CV_FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255), 1);



  }




     int target_i = - 1;//记录大风车的那片叶子是目标叶子
     int target_j = - 1;//中心点的坐标
     Point2f corners[4];
     Point2f* lastItemPointer;
     for(int i=0; i<rrects.size(); i++)
     {
         int ct = 0;
         int kk = -1;//记录叶子里面的目标
        // bool flag_kk = false;//表示这个目标是
         for(int j=0; j<rrects_mini.size(); j++)
         {
             rrects[i].points(corners);
             lastItemPointer = (corners + sizeof corners / sizeof corners[0]);
             vector<Point2f> contour(corners, lastItemPointer);

             //Check if the point is within the rectangle.
             double indicator = pointPolygonTest(contour, rrects_mini[j].center, false);
             bool rectangleContainsPoint = (indicator >= 0);
             ct += rectangleContainsPoint;
             if(ct == 1 && rectangleContainsPoint == true)
                 kk = j;
             //cout<<rectangleContainsPoint<<endl;
         }
         //cout<<rrects.size()<<"     ";
         rrects_ct[i] = ct;
         if(ct == 1 && kk != -1)
             target_i = kk;
         if(ct == 0)
             target_j = i;
         char slope_str[15];
         sprintf(slope_str, "%d", ct);
         putText(show_mid1_cp, slope_str, rrects[i].center, CV_FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255), 1);
     }
     //cout<<endl;
     imshow("show_mid1_cp",show_mid1_cp);

    
     cv::Mat show_mid1_cp_cp = Mat(midImage.size(),CV_8UC1,Scalar(0));
     
     if(target_j != -1)//画中心点
     {
         cv::RotatedRect rect_tmp1 = rrects[target_j];
         Point2f fourPoint2f2[4];
         rect_tmp1.points(fourPoint2f2);
         for (int i = 0; i < 4; i++)//画矩形
         {
             line(show_mid1_cp_cp, fourPoint2f2[i], fourPoint2f2[(i + 1) % 4], Scalar(255));
         }

     }

     if(target_i != -1)//画出目标
     {
         cv::RotatedRect rect_tmp1 = rrects_mini[target_i];
         Point2f fourPoint2f3[4];
         rect_tmp1.points(fourPoint2f3);
         char slope_str[15];

         for (int i = 0; i < 4; i++)//画矩形
         {
             sprintf(slope_str, "%d", i + 1);
             putText(show_mid1_cp_cp, slope_str, fourPoint2f3[i], CV_FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255), 1);
             line(show_mid1_cp_cp, fourPoint2f3[i], fourPoint2f3[(i + 1) % 4], Scalar(255));
         }

         imshow("show_mid1_cp_cp",show_mid1_cp_cp);
     }

     
     if(target_i != -1 && target_j != -1)
     {
         cv::RotatedRect rect_armor =  rrects_mini[target_i];//每一片叶子木目标的大小
         cv::RotatedRect rect_center = rrects[target_j];//中心方框

         if(rect_armor.center.y > rect_center.center.y)
         {
             bono = true;
             return rect_armor;
         }
         else
         {
             bono = false;
             return rect_armor;
         }
     }
     else
     {
         return cv::RotatedRect();
     }
     
     


     Mat target_mat = show_mid1.clone();
     morphologyEx(show_mid1,show_mid1,MORPH_DILATE,getStructuringElement(MORPH_RECT,Size(15,15)));//让大风车的叶子融合在一起。
 
     imshow("show_mid2",show_mid1);
 //    findContours(show_mid1, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);//对融合后的叶子进行轮廓查找
 
     cv::RotatedRect final_rect;
     
//    for(int i=0; i<contours.size(); i++)//不能使用面积的方案，会遇到
//    {
//        cout<<minAreaRect(contours[i]).boundingRect().area()<<"     ";//只要吧这个参数调好就ok了
//        if(minAreaRect(contours[i]).boundingRect().area() > 1400)
//            drawContours(target_mat,contours,i,Scalar(0),-1,8,hierarchy);
//        else
//        {

//            final_rect = minAreaRect(contours[i]);
//           // cout<<minAreaRect(contours[i]).boundingRect().area()<<endl;
//        }
//    }
//    cout<<endl;
//    morphologyEx(target_mat,target_mat,MORPH_DILATE,getStructuringElement(MORPH_RECT,Size(15,15)));

//    findContours(target_mat, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);//对融合后的叶子进行轮廓查找
//    Mat target_mat_show = Mat(midImage.size(),CV_8UC3,Scalar(0,3,3));


//    for(int i=0; i<contours.size(); i++)
//    {
//        drawContours(target_mat_show,contours,i,Scalar(255,255,0),1,8,hierarchy);
//        final_rect = fitEllipse(contours[i]);
//    }


//    Point2f fourPoint2f1[4];
//    final_rect.points(fourPoint2f1);



//    line(target_mat_show,Point(320,0),Point(320,480),Scalar(255,255,255),1,8);
//    for(int i=0; i<4; i++)
//    {


//        //cout<<fourPoint2f1[i]<<"   ";
//        char slope_str[15];
//        sprintf(slope_str, "%d", i+1);
// //       sprintf(slope_str, "%.2f", fourPoint2f1[i].x);
//        circle(target_mat_show,fourPoint2f1[i],1,Scalar(255,0,255),1,8);
//        putText(target_mat_show, slope_str, fourPoint2f1[i], CV_FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255,255,255), 1);

//    }

//    cout<<endl;
//    imshow("target_mat_show",target_mat_show);
//    imshow("target_mat",target_mat);


//    std::vector<int> ct_rect(contours.size());//记录大风车的一片叶子里面有多少个rect
//    //std::vector<>
//    findContours(show_mid1, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

//    Mat test1 = Mat::zeros(image.size(),CV_8UC1);

//    for(int i=0; i<contours.size(); i++)
//    {
//            cv::RotatedRect rect_tmp = minAreaRect(contours[i]);

//            //满足面积的大小，才能进入计数
//            if(rect_tmp.boundingRect().area() <  100)
//                continue;//防止杂质面积未清除干净，影响判断结果

//            Point2f fourPoint2f[4];
//            rect_tmp.points(fourPoint2f);
//            for (int i = 0; i < 4; i++)//画矩形
//                line(test1, fourPoint2f[i], fourPoint2f[(i + 1) % 4], Scalar(255));


//            float rect_width = 0, rect_height = 0, tmp = 0;
//            rect_width = (float)sqrt((pow((fourPoint2f[0].x - fourPoint2f[1].x), 2) + pow((fourPoint2f[0].y - fourPoint2f[1].y), 2)));
//            rect_height = (float)sqrt((pow((fourPoint2f[0].x - fourPoint2f[3].x), 2) + pow((fourPoint2f[0].y - fourPoint2f[3].y), 2)));

//            if (rect_width<rect_height)
//            {
//                tmp = rect_width;
//                rect_width = rect_height;
//                rect_height = tmp;
//            }
//            //printf("最小外接矩形的长为：%f，宽为：%f。\n\n", aaa, bbb);

//            float tio = rect_height / rect_width;
//            float area = rect_height * rect_width;

//            char slope_str[15];
//            sprintf(slope_str, "%.2f", area);
//            putText(image, slope_str, contours[i][0], CV_FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(255,255,255), 1);

//            Point2f center = rect_tmp.center;//外接矩形中心点坐标
//            Mat rot_mat = getRotationMatrix2D(center, rect_tmp.angle, 1.0);//求旋转矩阵

//            float area = rect_tmp.size.height * rect_tmp.size.width;
//            float tio = rect_tmp.size.height /rect_tmp.size.width;


//                char slope_str[15];
//                sprintf(slope_str, "%.1f", tio);
//                putText(image, slope_str, contours[i][0], CV_FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(255,255,255), 1);

//                rectangle(show_mid1,rect_tmp.boundingRect(),Scalar(255),1,8);
//            if(tio < 23
//                    && area < 30)
//            {

//            }

//    }
//    imshow("test1",test1);
    imshow("image",image);


//    cv::RotatedRect rect_tmp = minAreaRect(contours[i]);
//    adjustRRect(rect_tmp);

//    float area = rect_tmp.size.height*rect_tmp.size.width;
//    float tio = rect_tmp.size.height /rect_tmp.size.width;
//    Scalar color(rand() & 255, rand() & 255, rand() & 255);
//    char slope_str[15];
//    sprintf(slope_str, "%.1f", tio);
//    putText(show, slope_str, contours[i][0], CV_FONT_HERSHEY_COMPLEX_SMALL, 1, color, 1);

//        if(tio < 23
//                && area < 30)

    //imshow("contours", show);
  //  imshow("show_mid",show_mid);
   // imshow("show_mid1",show_mid1);


#endif

    if(final_rect.size.height == 0)
        return cv::RotatedRect();
    else
        return final_rect;
//    sudoku_rects.clear();
//    if (checkSudoku(contours, sudoku_rects)){
//        if (use_perspective == true){
//            pair<int, int> idx = chooseTargetPerspective(src, sudoku_rects);
//            return idx;
//        }
//        else{
//            pair<int, int> idx = chooseTarget(src, sudoku_rects);
//            return idx;
//        }
//    }
//    return make_pair(-1,-1);
}




//int RuneDetector::findTargetCanny(cv::Mat * cells){
//    int min_count_idx = -1;
//    int w_3 = cells[0].cols / 2.8;
//    int w_23 = cells[0].cols * 2 /3.0;
//    double mid_ratio = 0.0;

//    for (size_t i = 0; i < 9; i++)	{
//        int mid_area_count = 0;
//        int black_count = 0;
//        Mat edge;
//        Canny(cells[i], edge, 20, 50);
//        uchar * ptr = (uchar *)edge.data;

//        for (size_t j = 0; j < cells[i].rows; ++j){
//            for (size_t k = 0; k < cells[i].cols; ++k, ++ptr)	{
//                int v = *ptr;
//                if (v == 255)
//                    ++black_count;

//                if(k >= w_3 && k <= w_23)
//                    ++mid_area_count;
//            }
//        }

//        //cout << black_count << "  ";
//        double cur_ratio = (double)mid_area_count/black_count;
//        //cout << cur_ratio << "  ";
//        if(mid_ratio <  cur_ratio){
//            mid_ratio = cur_ratio;
//            min_count_idx = i;
//        }

//        //rectangle(edge, Rect(w_3,0,w_23-w_3,cells[i].rows), CV_RGB(255,255,255), 1);
//        //imshow(string("bin")+char(i+'0'), edge);

//    }
//    //cout << "\n";
//    return min_count_idx;
//}

//int RuneDetector::findTargetEdge(cv::Mat * cells){
//    int grad_threshold = 10;
//    int min_count = 65535;
//    int min_count_idx = -1;
//    for (size_t i = 0; i < 9; i++)	{
//        Mat gradX, gradY;
//        cv::Sobel(cells[i], gradX, CV_16S, 1, 0);
//        cv::Sobel(cells[i], gradY, CV_16S, 0, 1);

//        int large_grad_count = 0;
//        short * ptr_x = (short *)gradX.data;
//        short * ptr_y = (short *)gradY.data;

//        for (size_t j = 0; j < gradX.rows; ++j){
//            for (size_t k = 0; k < gradX.cols; ++k, ++ptr_x, ++ptr_y)	{
//                int x = abs(*ptr_x);
//                int y = abs(*ptr_y);
//                if (x > grad_threshold || y > grad_threshold)
//                    ++large_grad_count;
//            }
//        }

//        if(min_count > large_grad_count){
//            min_count = large_grad_count;
//            min_count_idx = i;
//        }
//        //imshow(string("bin")+char(i+'0'), cells[i]);
//    }
//    return min_count_idx;
//}

//pair<int, int> RuneDetector::chooseTargetPerspective(const Mat & image, const vector<RotatedRect> & sudoku_rects){
//    // get 9(cell) X 4(corner) corner, and 9 cell's center
//    vector<Point2fWithIdx> centers;
//    vector<Point2f> corner;
//    for (size_t i = 0; i < sudoku_rects.size(); i++)	{
//        const RotatedRect & rect = sudoku_rects[i];
//        Point2f vecs[4];
//        rect.points(vecs);
//        for (size_t j = 0; j < 4; j++) {
//            corner.push_back(vecs[j]);
//        }
//        centers.push_back(Point2fWithIdx(rect.center, i));
//    }

//    // arange sudoku cell to following order
//    // 0  1  2
//    // 3  4  5
//    // 6  7  8
//    sort(centers.begin(), centers.end(), [](const Point2fWithIdx & p1, const Point2fWithIdx & p2) { return p1.p.y < p2.p.y; });
//    sort(centers.begin() + 0, centers.begin() + 3, [](const Point2fWithIdx & p1, const Point2fWithIdx & p2) { return p1.p.x < p2.p.x; });
//    sort(centers.begin() + 3, centers.begin() + 6, [](const Point2fWithIdx & p1, const Point2fWithIdx & p2) { return p1.p.x < p2.p.x; });
//    sort(centers.begin() + 6, centers.begin() + 9, [](const Point2fWithIdx & p1, const Point2fWithIdx & p2) { return p1.p.x < p2.p.x; });

//    // get position of [0,2,6,8] corner
//    int corner_idx[] = { 0, 2, 6, 8 };
//    vector<Point2f> corner_0268;
//    for (size_t i = 0; i < 4; i++) {
//        size_t k = centers[corner_idx[i]].idx * 4;
//        for (size_t j = 0; j < 4; j++){
//            corner_0268.push_back(corner[k + j]);
//        }
//    }

//    // find approx corner of sudoku
//    RotatedRect rect = minAreaRect(corner_0268);
//    Point2f vertices[4];
//    rect.points(vertices);
//    Point2f lu, ld, ru, rd;
//    sort(vertices, vertices + 4, [](const Point2f & p1, const Point2f & p2) { return p1.x < p2.x; });
//    if (vertices[0].y < vertices[1].y){
//        lu = vertices[0];
//        ld = vertices[1];
//    }
//    else{
//        lu = vertices[1];
//        ld = vertices[0];
//    }
//    if (vertices[2].y < vertices[3].y)	{
//        ru = vertices[2];
//        rd = vertices[3];
//    }
//    else {
//        ru = vertices[3];
//        rd = vertices[2];
//    }

//    // find actual corner of sudoku
//    Point2f _lu, _ld, _ru, _rd;
//    float mlu = 10000.0, mld = 10000.0, mru = 10000.0, mrd = 10000.0;
//    for (size_t i = 0; i < corner_0268.size(); i++) {
//        const Point2f & p = corner_0268[i];
//        float v1 = (p - lu).dot((p - lu));
//        float v2 = (p - ld).dot((p - ld));
//        float v3 = (p - ru).dot((p - ru));
//        float v4 = (p - rd).dot((p - rd));
//        if (v1 < mlu) {
//            mlu = v1;
//            _lu = p;
//        }
//        if (v2 < mld) {
//            mld = v2;
//            _ld = p;
//        }
//        if (v3 < mru) {
//            mru = v3;
//            _ru = p;
//        }
//        if (v4 < mrd) {
//            mrd = v4;
//            _rd = p;
//        }
//    }

//    // applies a perspective transformation to an image
//    float _width = max((_lu - _ru).dot(_lu - _ru), (_ld - _rd).dot(_ld - _rd));
//    float _height = max((_lu - _ld).dot(_lu - _ld), (_rd - _ru).dot(_rd - _ru));
//    _width = sqrtf(_width);
//    _height = sqrtf(_height);

//    vector<Point2f> src_p;
//    src_p.push_back(_lu);
//    src_p.push_back(_ld);
//    src_p.push_back(_ru);
//    src_p.push_back(_rd);

//    vector<Point2f> dst_p;
//    dst_p.push_back(Point2f(0.0, 0.0));
//    dst_p.push_back(Point2f(0.0, _height));
//    dst_p.push_back(Point2f(_width, 0.0));
//    dst_p.push_back(Point2f(_width, _height));

//    Mat perspective_mat = getPerspectiveTransform(src_p, dst_p);
//    Mat image_persp;
//    warpPerspective(image, image_persp, perspective_mat, Size(_width, _height));

//    // calculate the average width and hieght of each cell
//    const double * pdata = (double *)perspective_mat.data;
//    float height_avg = 0.0, width_avg = 0.0;
//    for (size_t i = 0; i < sudoku_rects.size(); ++i) {
//        vector<Point2f> vec_p;
//        for (size_t j = 0; j < 4; j++) {
//            const Point2f & p = corner[i * 4 + j];
//            float x = pdata[0] * p.x + pdata[1] * p.y + pdata[2];
//            float y = pdata[3] * p.x + pdata[4] * p.y + pdata[5];
//            float s = pdata[6] * p.x + pdata[7] * p.y + pdata[8];
//            vec_p.push_back(Point2f(x / s, y / s));
//        }
//        Rect2f r = boundingRect(vec_p);
//        height_avg += r.height;
//        width_avg += r.width;
//    }
//    height_avg /= 9.0;
//    width_avg /= 9.0;

//    if(height_avg > _height / 3)
//        height_avg = 0.25 * _height;
//    if(width_avg > _width / 3)
//        width_avg = 0.25 * _width;

//    // get image of every cell, then compute ORB feature and match feature;
//    int cell_width = 0.48 * width_avg + 0.5;
//    int cell_height = 0.50 * height_avg + 0.5;
//    int half_w_gap = (width_avg - cell_width) / 2, half_h_gap = (height_avg - cell_height) / 2;
//    int offset_x = 0.05 * cell_width + 0.5;
//    int offset_y = 0.05 * cell_height + 0.5;
//    int width_start[] = { half_w_gap, (_width - cell_width) / 2, _width - cell_width - half_w_gap };
//    int height_start[] = { half_h_gap, (_height - cell_height) / 2, _height - cell_height - half_h_gap };

//    Mat cell[9];
//    for (size_t i = 0; i < 3; i++){
//        for (size_t j = 0; j < 3; j++){
//            size_t idx = i * 3 + j;
//            Rect cell_roi(width_start[j]+offset_x, height_start[i]+offset_y, cell_width, cell_height);
//            image_persp(cell_roi).copyTo(cell[idx]);
//        }
//    }

//    int idx = -1;
//    if (type == RUNE_ORB)
//        idx = findTargetORB(cell);
//    else if (type == RUNE_GRAD)
//        idx = findTargetEdge(cell);
//    else if (type = RUNE_CANNY)
//        idx = findTargetCanny(cell);

//    //int idxx = findTargetCanny(cell);
//    //cout << "Canny: " << idxx << "\tEDGE: " << idx << endl;
//    //return idx < 0 ? make_pair(-1, -1) : make_pair((int)centers[idx].idx, idx);
//}

//pair<int, int> RuneDetector::chooseTarget(const Mat & image, const vector<RotatedRect> & sudoku_rects){
//    // get 9(cell) X 4(corner) corner, and 9 cell's center
//    vector<Point2fWithIdx> centers;
//    vector<Point2f> corner;
//    for (size_t i = 0; i < sudoku_rects.size(); i++)	{
//        const RotatedRect & rect = sudoku_rects[i];
//        Point2f vecs[4];
//        rect.points(vecs);
//        for (size_t j = 0; j < 4; j++) {
//            corner.push_back(vecs[j]);
//        }
//        centers.push_back(Point2fWithIdx(rect.center, i));
//    }

//    // arange sudoku cell to following order
//    // 0  1  2
//    // 3  4  5
//    // 6  7  8
//    sort(centers.begin(), centers.end(), [](const Point2fWithIdx & p1, const Point2fWithIdx & p2) { return p1.p.y < p2.p.y; });
//    sort(centers.begin() + 0, centers.begin() + 3, [](const Point2fWithIdx & p1, const Point2fWithIdx & p2) { return p1.p.x < p2.p.x; });
//    sort(centers.begin() + 3, centers.begin() + 6, [](const Point2fWithIdx & p1, const Point2fWithIdx & p2) { return p1.p.x < p2.p.x; });
//    sort(centers.begin() + 6, centers.begin() + 9, [](const Point2fWithIdx & p1, const Point2fWithIdx & p2) { return p1.p.x < p2.p.x; });

//    Mat cell[9];

//    for (size_t i = 0; i < 3; i++){
//        for (size_t j = 0; j < 3; j++){
//            size_t idx = i * 3 + j;
//            Rect cell_roi = sudoku_rects[centers[idx].idx].boundingRect();
//            int margin_x = 0.25* cell_roi.width;
//            int margin_y = 0.25* cell_roi.height;
//            Rect scale_roi = Rect(cell_roi.x + margin_x, cell_roi.y + margin_y, cell_roi.width - 2*margin_x, cell_roi.height - 2*margin_y);
//            image(scale_roi).copyTo(cell[idx]);
//        }
//    }

///*    int idx = -1;
//    if (type == RUNE_ORB)
//        idx = findTargetORB(cell);
//    else if (type == RUNE_GRAD)
//        idx = findTargetEdge(cell);
//    else if (type = RUNE_CANNY)
//        idx = findTargetCanny(cell);

//    //int idxx =*/ //findTargetCanny(cell);
//    //cout << "Canny: " << idxx << "\tEDGE: " << idx << endl;
//  //  return idx < 0 ? make_pair(-1, -1) : make_pair((int)centers[idx].idx, idx);
//}

RotatedRect RuneDetector::adjustRRect(const RotatedRect & rect){
    const Size2f & s = rect.size;
    if (s.width > s.height)
        return rect;
    return RotatedRect(rect.center, Size2f(s.height, s.width), rect.angle);
}

