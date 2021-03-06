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
    int PtRowIdx;  //第idx行

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
    int rowIdx;  //第idx行

    int imgIdx;  //第idx幅图像
    //int laserIdx;  //第idx条激光线
};

//已匹配点的结构体
struct BeMatchedPoints
{
    int BMRowIdx;  //第idx行

    std::vector<P2dPackMatch> P2dMatchedSlice;
};

//匹配得到的视差与激光线归属, W*H个点
struct DisparityAndBelonging
{
    cv::Mat Disparity;  //视差
    int *DispBelonging;  //激光线归属,[HEIGHT][WIDTH]与Mat匹配
};

//通过2D特征点得到的空间坐标, W*H个点
struct XYZAndBelonging
{
    cv::Mat XYZ;  //空间坐标
    int *XYZBelonging;  //激光线归属,[HEIGHT][WIDTH]与Mat匹配
};

//与上面结构体不同，FilteredP3d描述一个点
struct FilteredP3d
{
    cv::Point3d filterd_p3d;
    int fP3dBelonging;

};

#endif // UTILS_H
