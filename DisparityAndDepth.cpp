#include <iostream>

#include "DisparityAndDepth.h"
#include "RetrievePointCloud.h"

/***************根据匹配点，得到视差与深度************/

//void get_point(cv::Point2f &left1, cv::Point2f& right1)
//{
//    double fx = 454.025;//像素
//
//
//    // 基线
//    double baseline = 0.191;//单位米
//            if ((left1.x - right1.x) == 0.0)
//    {
//        return  0.0;
//
//    }
//
//    cv ::Point3d p;
//    double Z = fx * baseline / (left1.x - right1.x);
//    double  X = (left1.x - cx)*Z / fx;
//    double  Y = (left1.y - cy)*Z / fy;
////单位米
//
//}

int CalcDepthFromBeMatchedPoints(BeMatchedPoints *inputMatchedPts)
{
#ifdef CAMERA_HORIZONTAL
    #if 0
    for (int ii = 0; ii < HEIGHT; ii++)
    {
        if((0 == inputMatchedPts[ii].P2dMatchedSlice.size()))
        {
            //IMG0, IMG1任意一张图该行/列没有激光点
            continue;
        }

        printf("OutpointY: (%lf, %lf)\n", inputMatchedPts[ii].P2dMatchedSlice[0].p2dPack[0].Pt2d.x, inputMatchedPts[ii].P2dMatchedSlice[0].p2dPack[0].Pt2d.y);
    }
    #endif

    double X, Y, Z;
    double baseline = 400;//单位: mm
    double fx = 2232.550777775964;
    double fy = 2231.856456030659;
    double cx = 2232.550777775964;
    double cy = 2232.550777775964;

    //视差
    cv::Mat Disparity(HEIGHT, WIDTH, CV_64FC1, cv::Scalar(0));  //CV_8U

    cv::Point3d *pp3d = new cv::Point3d[WIDTH * HEIGHT];
    #ifdef PCL_PROCESS
    RetrievePointCloud *mPC = new RetrievePointCloud();
    #endif

    for (int ii = 0; ii < HEIGHT; ii++)
    {
        //该行/列没有匹配点
        if(0 == inputMatchedPts[ii].P2dMatchedSlice.size())
        {
            continue;
        }

        //该行/列有匹配点
        for(int jj = 0; jj < inputMatchedPts[ii].P2dMatchedSlice.size(); jj++ )
        {
            double left_x = inputMatchedPts[ii].P2dMatchedSlice[jj].p2dPack[0].Pt2d.x;
            double right_x = inputMatchedPts[ii].P2dMatchedSlice[jj].p2dPack[1].Pt2d.x;

            double left_y = ii;

            //计算视差
            //double Z = fx * baseline / (left1.x - right1.x);
            //double  X = (left1.x - cx)*Z / fx;
            //double  Y = (left1.y - cy)*Z / fy;
            Z = fx * baseline / (left_x - right_x);  //暂时让为正
            //Disparity.at<double>(round(left_x), ii) =  Z;
            //modified by flq, 显示错误
            Disparity.at<double>(ii, round(left_x)) =  Z;

            printf("左右两个点的坐标：(%lf, %d), (%lf, %d)\n", left_x, ii, right_x, ii);
            //printf("fx * baseline=%lf, left_x - right_x=%lf\n", fx * baseline, left_x - right_x);
            //printf("Z at point(%lf,%d)=%lf\n", left_x, ii, Z);


            X = (left_x - cx) * Z / fx;
            Y = (left_y - cy) * Z / fy;

            cv::Point3d p3d = cv::Point3d(X,Y,Z);
            std::cout << "空间坐标:" << p3d << std::endl << std::endl;

            pp3d[ii*WIDTH + (int)(inputMatchedPts[ii].P2dMatchedSlice[jj].p2dPack[0].Pt2d.x)].x = X;
            pp3d[ii*WIDTH + (int)(inputMatchedPts[ii].P2dMatchedSlice[jj].p2dPack[0].Pt2d.x)].y = Y;
            pp3d[ii*WIDTH + (int)(inputMatchedPts[ii].P2dMatchedSlice[jj].p2dPack[0].Pt2d.x)].z = Z;

        }
    }

    imshow("Disparity", Disparity);
    #ifdef PCL_PROCESS
    mPC->RetriveInit(pp3d);
    #endif

    return 0;

#else
    #if 0
    for (int ii = 0; ii < WIDTH; ii++)
    {
        if((0 == inputMatchedPts[ii].P2dMatchedSlice.size()))
        {
            //IMG0, IMG1任意一张图该行/列没有激光点
            continue;
        }

        printf("OutpointY: (%lf, %lf)\n", inputMatchedPts[ii].P2dMatchedSlice[0].p2dPack[0].Pt2d.x, inputMatchedPts[ii].P2dMatchedSlice[0].p2dPack[0].Pt2d.y);
    }
    #endif

    double X, Y, Z;
    double baseline = 400;//单位: mm
    double fx = 2232.550777775964;
    double fy = 2231.856456030659;
    double cx = 2232.550777775964;
    double cy = 2232.550777775964;

    //视差
    cv::Mat Disparity(HEIGHT, WIDTH, CV_64FC1, cv::Scalar(0));  //CV_8U

    cv::Point3d *pp3d = new cv::Point3d[WIDTH * HEIGHT];
    #ifdef PCL_PROCESS
    RetrievePointCloud *mPC = new RetrievePointCloud();
    #endif

    for (int ii = 0; ii < WIDTH; ii++)
    {
        //该行/列没有匹配点
        if(0 == inputMatchedPts[ii].P2dMatchedSlice.size())
        {
            continue;
        }

        //该行/列有匹配点
        for(int jj = 0; jj < inputMatchedPts[ii].P2dMatchedSlice.size(); jj++ )
        {
            double top_y = inputMatchedPts[ii].P2dMatchedSlice[jj].p2dPack[0].Pt2d.y;
            double bottom_y = inputMatchedPts[ii].P2dMatchedSlice[jj].p2dPack[1].Pt2d.y;

            double top_x = ii;

            //计算视差
            //double Z = fx * baseline / (left1.x - right1.x);
            //double  X = (left1.x - cx)*Z / fx;
            //double  Y = (left1.y - cy)*Z / fy;
            Z = fx * baseline / (top_y - bottom_y);  //暂时让为正
            //modified by flq, 显示错误
            //Disparity.at<double>(ii, round(top_y)) =  Z;
            Disparity.at<double>(round(top_y), ii) =  Z;

            printf("上下两个点的坐标：(%lf,%d), (%lf,%d)\n", ii, top_y, ii, bottom_y);
            printf("fx * baseline=%lf, left_x - right_x=%lf\n", fx * baseline, bottom_y - top_y);
            printf("Z at point(%lf,%d)=%lf\n\n", ii, top_y, Z);


            X = (top_y - cx) * Z / fx;
            Y = (top_x - cy) * Z / fy;

            cv::Point3d p3d = cv::Point3d(X,Y,Z);
            std::cout << "空间坐标" << p3d << std::endl << std::endl;

            pp3d[ii*HEIGHT + (int)(inputMatchedPts[ii].P2dMatchedSlice[jj].p2dPack[0].Pt2d.x)].x = X;
            pp3d[ii*HEIGHT + (int)(inputMatchedPts[ii].P2dMatchedSlice[jj].p2dPack[0].Pt2d.x)].y = Y;
            pp3d[ii*HEIGHT + (int)(inputMatchedPts[ii].P2dMatchedSlice[jj].p2dPack[0].Pt2d.x)].z = Z;

        }
    }

    imshow("Disparity", Disparity);
    #ifdef PCL_PROCESS
    mPC->RetriveInit(pp3d);
    #endif

    return 0;

#endif
}
