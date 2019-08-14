#include "ReferenceLine.h"

//图0的分类
int referenceIn0(cv::Point2d &inputP2dPack)
{
    int line_num = -1;

    if((inputP2dPack.y >0) && (inputP2dPack.y < 240))
    {
        line_num = 0;
    }
    else if((inputP2dPack.y >240) && (inputP2dPack.y < 480))
    {
        line_num = 1;
    }
    else if((inputP2dPack.y >480) && (inputP2dPack.y < 720))
    {
        line_num = 2;
    }
    else
    {
        line_num = 3;
    }
#if 0
    if((inputP2dPack.y >0) && (inputP2dPack.y < 320))
    {
        line_num = 0;
    }
    else if((inputP2dPack.y >320) && (inputP2dPack.y < 640))
    {
        line_num = 1;
    }
    else if((inputP2dPack.y >640) && (inputP2dPack.y < 960))
    {
        line_num = 2;
    }
    else
    {
        line_num = 3;
    }
#endif

    return line_num;
}

//图1的分类
int referenceIn1(cv::Point2d &inputP2dPack)
{
    int line_num = -1;

    if((inputP2dPack.y >0) && (inputP2dPack.y < 240))
    {
        line_num = 0;
    }
    else if((inputP2dPack.y >240) && (inputP2dPack.y < 480))
    {
        line_num = 1;
    }
    else if((inputP2dPack.y >480) && (inputP2dPack.y < 720))
    {
        line_num = 2;
    }
    else
    {
        line_num = 3;
    }

#if 0 //CAMERA_HORIZONTAL
    if((inputP2dPack.y >0) && (inputP2dPack.y < 320))
    {
        line_num = 0;
    }
    else if((inputP2dPack.y >320) && (inputP2dPack.y < 640))
    {
        line_num = 1;
    }
    else if((inputP2dPack.y >640) && (inputP2dPack.y < 960))
    {
        line_num = 2;
    }
    else
    {
        line_num = 3;
    }
#endif

    return line_num;
}


int excuteReference0(std::vector<cv::Point2d> &img_pts, std::vector<Point2dPack> &img_ref)
{
    Point2dPack temppack;

    for (int i = 0; i < img_pts.size(); i++)
    {
        //img_pts[i].belonging = 0;
        temppack.Pt2d =  img_pts[i];
        //属于第0个相机的图像
        temppack.PtImgIdx = 0;
        //属于第n条激光线
        temppack.PtBelonging = referenceIn0(img_pts[i]);

        img_ref.push_back(temppack);
    }

    return 0;
}

int excuteReference1(std::vector<cv::Point2d> &img_pts, std::vector<Point2dPack> &img_ref)
{
    Point2dPack temppack;

    for (int i = 0; i < img_pts.size(); i++)
    {
        //img_pts[i].belonging = 0;
        temppack.Pt2d =  img_pts[i];
        //属于第1个相机的图像
        temppack.PtImgIdx = 1;
        //属于第n条激光线
        temppack.PtBelonging = referenceIn1(img_pts[i]);

        img_ref.push_back(temppack);
    }

    return 0;
}
