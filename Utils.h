#ifndef UTILS_H
#define UTILS_H

#define WIDTH 1280
#define HEIGHT 960

#define BRIGHTNESS_THRESHHOLD  150//120  //default:95

#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"

//一个激光点的metadata
struct Point2dPack
{
    int PtBelonging;  //该点属于第idx条激光
    int PtImgIdx;  //第idx幅图像.   0:left  1:right
#ifdef CAMERA_HORIZONTAL
    int PtRowIdx;  //第idx行
#else
    int PtColIdx;  //第idx列
#endif
    cv::Point2d Pt2d;  //该点的2d坐标
};

//一对激光点的metadata
struct P2dPackMatch
{
    Point2dPack p2dPack[2];
    int belonging;  //该点属于哪一条激光
};


//待匹配点的结构体
struct UnMatchedPoints
{
    std::vector<Point2dPack> Slice;  //特定的某一行或者列
#ifdef CAMERA_HORIZONTAL
    int rowIdx;  //第idx行
#else
    int colIdx;  //第idx列
#endif
    int imgIdx;  //第idx幅图像
    //int laserIdx;  //第idx条激光线
};

//已匹配点的结构体
struct BeMatchedPoints
{
    #ifdef CAMERA_HORIZONTAL
        int BMRowIdx;  //第idx行
    #else
        int BMColIdx;  //第idx列
    #endif

    std::vector<P2dPackMatch> P2dMatchedSlice;
};


#endif // UTILS_H
