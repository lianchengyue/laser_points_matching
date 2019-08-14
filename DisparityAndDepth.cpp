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
    int total_num = 0;
    int count = 0;

    //视差
    cv::Mat Disparity(HEIGHT, WIDTH, CV_64FC1, cv::Scalar(0));  //CV_8U

    cv::Point3d *pp3d = new cv::Point3d[WIDTH * HEIGHT];
    cv::Point3d *pp3d2 = new cv::Point3d[3167];
    //#ifdef PCL_PROCESS
    //RetrievePointCloud *mPC = new RetrievePointCloud();
    //#endif

    for (int ii = 0; ii < WIDTH; ii++)
    {
        //该行/列没有匹配点
        if(0 == inputMatchedPts[ii].P2dMatchedSlice.size())
        {
            continue;
        }

        total_num += inputMatchedPts[ii].P2dMatchedSlice.size();

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

            count++;

        }
    }

    printf("\n视差图中总计有%d对匹配点\n\n", total_num);

    imshow("Disparity", Disparity);
    //#ifdef PCL_PROCESS
    //mPC->RetriveInit(pp3d2);
    //#endif

    return 0;

#endif
}




int CalcDisparityFromBeMatchedPoints(BeMatchedPoints *inputMatchedPts)
{
#ifdef CAMERA_HORIZONTAL
    //视差
    DisparityAndBelonging disp_belonging;
    disp_belonging.Disparity = cv::Mat::zeros(HEIGHT, WIDTH, CV_64FC1);
//    cv::Mat Disparity(HEIGHT, WIDTH, CV_64FC1, cv::Scalar(0));  //CV_8U

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

            double MinusValue = right_x - left_x;

            //计算视差
//            Disparity.at<double>(ii, round(left_x)) =  MinusValue;
            disp_belonging.Disparity.at<double>(ii, round(left_x)) =  MinusValue;
            disp_belonging.DispBelonging[ii][(int)round(left_x)] = inputMatchedPts[ii].P2dMatchedSlice[jj].belonging;

            printf("disp Value at point(%lf,%d)=%lf\n\n", left_x, ii, MinusValue);

        }
    }

    imshow("Disparity", disp_belonging.Disparity);

    return 0;

#else
    //视差, 参数3为dtype
    DisparityAndBelonging disp_belonging;
    disp_belonging.Disparity = cv::Mat::zeros(HEIGHT, WIDTH, CV_64FC1);
    //cv::Mat Disparity(HEIGHT, WIDTH, CV_64FC1, cv::Scalar(0));  //CV_8U   CV_32S  CV_32SC1 CV_64FC1

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

            int MinusValue = top_y - bottom_y;

            //计算视差
            //modified by flq, 显示错误
            //Disparity.at<double>(round(top_y), ii) =  MinusValue;
            disp_belonging.Disparity.at<double>(round(top_y), ii) =  MinusValue;
            disp_belonging.DispBelonging[(int)round(top_y)][ii] = inputMatchedPts[ii].P2dMatchedSlice[jj].belonging;

            printf("disp Value at point(%lf,%d)=%lf\n\n", ii, top_y, MinusValue);

        }
    }

    imshow("Disparity", disp_belonging.Disparity);

    ///Important, 计算3D坐标
    Calc3DFromDisparity(disp_belonging);

    return 0;

#endif
}
/*
//https://blog.csdn.net/Gordon_Wei/article/details/86319058
 * reprojectImageTo3D的说明

    cx​、cycy​为左相机主点在图像中的坐标，
    ff为焦距，
    Tx​为两台相机投影中心间的平移（负值），
    c′xcx′​是右相机主点在图像中的坐标
----------
    运算如下：
    [X,Y,Z,W]的T = Q∗[x,y,disparity(x,y),1]的T
    |
    v
    _3dImage(x,y)=(X/W,Y/W,Z/W)
    |
    v
    Z/W = Tx*f /-[d-(cx-c`x)]
----------
    其中:
        [1   0   0       −cx        ]
    Q = [0   1   0       -cy        ]
        [0   0   0       f          ]
        [0   0   -1/Tx   (cx-c`x)/Tx]

      [x]   [x-cx          ]   [X]
    Q*[y] = [y-cy          ] = [Y]
      [d]   [f             ]   [Z]
      [1]   [(-d+cx-c`x)/Tx]   [W]
*/
//如果获得视差图像是CV_16S类型的，这样的视差图的每个像素值由一个16bit表示，其中低位的4位存储的是视差值得小数部分，所以真实视差值应该是该值除以16。在进行映射后应该乘以16，以获得毫米级真实位置。
int Calc3DFromDisparity(DisparityAndBelonging &disp_belonging/*cv::Mat &disp*/)
{
    cv::Mat MatQ = (cv::Mat_<double>(4, 4) <<
                     1, 0, 0, -642.8666915893555,
                     0, 1, 0, -394.385528087616,
                     0, 0, 0, -24379.04789257518,
                     0, 0, 0.001449256085268874, -0);

    cv::Mat xyz;    //三维坐标
    //XYZAndBelonging xyz_belonging;

    //手动重写该函数,抛弃所有的(0,0)点
#if 0
    //Auto
    reprojectImageTo3D(disp, xyz, MatQ, /*true, */false); //在实际求距离时，ReprojectTo3D出来的X / W, Y / W, Z / W都要乘以16(也就是W除以16)，才能得到正确的三维坐标信息。
#else
    //Manual
//     customizeReprojectImageTo3D(disp_belonging.Disparity, xyz, MatQ, true, CV_64FC3);  //param5: CV_32FC3; CV_16SC3, CV_32SC3, CV_32FC3
     customizeReprojectImageTo3D2(/*disp_belonging, /*xyz_belonging,*/ MatQ, true, CV_64FC3);  //param5: CV_32FC3; CV_16SC3, CV_32SC3, CV_32FC3
#endif
    //std::cout << "xyz:" << xyz << std::endl;

    //saveXYZ("XX.pcd", xyz);

//    //cv::Point3d *outputP3d = new cv::Point3d[3167];
//    cv::Point3d outputP3d[3167];
//    filterXYZ(xyz, outputP3d);
//    //free(outputP3d);
//
//    #ifdef PCL_PROCESS
//    RetrievePointCloud *mPC = new RetrievePointCloud();
//    mPC->RetriveInit(outputP3d);
//    #endif

}

void saveXYZ(const char* filename, const cv::Mat& mat)
{
    const double max_z = 1.0e4;
    FILE* fp = fopen(filename, "wb");
    for(int y = 0; y < mat.rows; y++)
    {
        for(int x = 0; x < mat.cols; x++)
        {
            cv::Vec3f point = mat.at<cv::Vec3f>(y, x);
            //if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z)
            //{
            //    continue;
            //}
            fprintf(fp, "%f %f %f\n", point[0]/1000, point[1]/1000, point[2]/10000);
        }
    }
    fclose(fp);
}

void customizeReprojectImageTo3D(cv::InputArray _disparity,
                             cv::OutputArray __3dImage, cv::InputArray _Qmat,
                             bool handleMissingValues, int dtype )
{
    //s:source         d:destination:
    //手动
    cv::Mat disparity = _disparity.getMat();
    cv::Mat Q = _Qmat.getMat();

    int stype = disparity.type();  //CV_64FC1
    printf("stype=%d,CV_64FC1=%d\n", stype, CV_64FC1);  //all=6
    printf("dtype=%d,CV_MAKETYPE(dtype, 1)=%d\n", dtype, CV_MAKETYPE(dtype, 3));  //all=6


    //int dtype = CV_32FC3;  //CV_16SC3, CV_32SC3, CV_32FC3
    //判断条件
    //CV_Assert( stype == CV_8UC1 || stype == CV_16SC1 ||
    //           stype == CV_32SC1 || stype == CV_32FC1 );
    //CV_Assert( Q.size() == cv::Size(4,4) );
    //
    //if( dtype < 0 )
    //    dtype = CV_32FC3;
    //else
    //{
    //    dtype = CV_MAKETYPE(CV_MAT_DEPTH(dtype), 3);
    //    CV_Assert( dtype == CV_16SC3 || dtype == CV_32SC3 || dtype == CV_32FC3 );
    //}
    //判断条件end

    //定义见/opencv-3.2.0/modules/core/include/opencv2/core/hal/interface.h
    //__3dImage.create(disparity.size(), CV_MAKETYPE(dtype, 3));
    __3dImage.create(disparity.size(), /*or CV_MAKETYPE(CV_64F,3)*/CV_64FC3);
    cv::Mat _3dImage = __3dImage.getMat();

    const double bigZ = 10000.;
    double q[4][4];
    cv::Mat _Q(4, 4, CV_64F, q);
    Q.convertTo(_Q, CV_64F);

    int x, cols = disparity.cols;
    //source: sbuf;       destination: dbuf
    std::vector<double> _sbuf(cols+1), _dbuf(cols*3+1);
    double* sbuf = &_sbuf[0], *dbuf = &_dbuf[0];

    double minDisparity = FLT_MAX;

    // NOTE: here we quietly assume that at least one pixel in the disparity map is not defined.
    // and we set the corresponding Z's to some fixed big value.
    if( handleMissingValues )
    {
        cv::minMaxIdx( disparity, &minDisparity, 0, 0, 0 );
    }

    //row = Height = 960:
    ///输入的视差图为单通道, 故stype为C1
    for( int y = 0; y < disparity.rows; y++ )
    {
        double *sptr = sbuf, *dptr = dbuf;
        double qx = q[0][1]*y + q[0][3], qy = q[1][1]*y + q[1][3];
        double qz = q[2][1]*y + q[2][3], qw = q[3][1]*y + q[3][3];

        if( stype == CV_8UC1 )
        {
            const uchar* sptr0 = disparity.ptr<uchar>(y);
            for( x = 0; x < cols; x++ )
                sptr[x] = (float)sptr0[x];
        }
        else if( stype == CV_16SC1 )
        {
            const short* sptr0 = disparity.ptr<short>(y);
            for( x = 0; x < cols; x++ )
                sptr[x] = (float)sptr0[x];
        }
        else if( stype == CV_32SC1 )
        {
            const int* sptr0 = disparity.ptr<int>(y);
            for( x = 0; x < cols; x++ )
                sptr[x] = (float)sptr0[x];
        }
        else if( stype == CV_64FC1 )
        {
            const double* sptr0 = disparity.ptr<double>(y);
            for( x = 0; x < cols; x++ )
                sptr[x] = (double)sptr0[x];
        }
        else
        {
            sptr = (double*)disparity.ptr<double>(y);
        }


        if( dtype == CV_32FC3 )
            dptr = _3dImage.ptr<double>(y);

        ///以下为des赋值
        //cols = 1280
        for( x = 0; x < cols; x++, qx += q[0][0], qy += q[1][0], qz += q[2][0], qw += q[3][0] )
        {
            double d = sptr[x];
            double iW = 1./(qw + q[3][2]*d);
            double X = (qx + q[0][2]*d)*iW;
            double Y = (qy + q[1][2]*d)*iW;
            double Z = (qz + q[2][2]*d)*iW;
            if( fabs(d-minDisparity) <= FLT_EPSILON )
                Z = bigZ;

            //added for 激光点以外的点设为0, not for -inf
            if( 0 == d)
            {
                dptr[x*3] =   0;
                dptr[x*3+1] = 0;
                dptr[x*3+2] = 0;
            }
            //added end

            dptr[x*3] =   (double)X;
            dptr[x*3+1] = (double)Y;
            dptr[x*3+2] = (double)Z;
        }

        if( dtype == CV_16SC3 )
        {
            short* dptr0 = _3dImage.ptr<short>(y);
            for( x = 0; x < cols*3; x++ )
            {
                int ival = cvRound(dptr[x]);
                dptr0[x] = cv::saturate_cast<short>(ival);
            }
        }
        else if( dtype == CV_32SC3 )
        {
            int* dptr0 = _3dImage.ptr<int>(y);
            for( x = 0; x < cols*3; x++ )
            {
                int ival = cvRound(dptr[x]);
                dptr0[x] = ival;
            }
        }
        else if( dtype == CV_64FC3)
        {
            //y: 行号, 0-959
            double* dptr0 = _3dImage.ptr<double>(y);
            //WIDTH = cols =1280
            for( x = 0; x < cols*3; x++ )
            {
                int ival = cvRound(dptr[x]);
                dptr0[x] = ival;

                if(-2147483648 == dptr0[x])
                {
                    dptr0[x] = 0;
                }

                //-inf == -2147483648.000000
                //printf("dptr0[%d]= %d\n", x, dptr0[x]);
            }
        }
    }
}

void customizeReprojectImageTo3D2(/*DisparityAndBelonging disp_belonging,*/
                             /*XYZAndBelonging xyz_belonging,*/ cv::InputArray _Qmat,
                             bool handleMissingValues, int dtype )
{
    //s:source         d:destination:
    //手动
//    cv::Mat disparity = disp_belonging.Disparity;
//    cv::Mat Q = _Qmat.getMat();
//
//    int stype = disparity.type();  //CV_64FC1
//    printf("stype=%d,CV_64FC1=%d\n", stype, CV_64FC1);  //all=6
//    printf("dtype=%d,CV_MAKETYPE(dtype, 1)=%d\n", dtype, CV_MAKETYPE(dtype, 3));  //all=6
//
//
//    //int dtype = CV_32FC3;  //CV_16SC3, CV_32SC3, CV_32FC3
//    //判断条件
//    //CV_Assert( stype == CV_8UC1 || stype == CV_16SC1 ||
//    //           stype == CV_32SC1 || stype == CV_32FC1 );
//    //CV_Assert( Q.size() == cv::Size(4,4) );
//    //
//    //if( dtype < 0 )
//    //    dtype = CV_32FC3;
//    //else
//    //{
//    //    dtype = CV_MAKETYPE(CV_MAT_DEPTH(dtype), 3);
//    //    CV_Assert( dtype == CV_16SC3 || dtype == CV_32SC3 || dtype == CV_32FC3 );
//    //}
//    //判断条件end
//
//    //定义见/opencv-3.2.0/modules/core/include/opencv2/core/hal/interface.h
//    xyz_belonging.XYZ.create(disparity.size(), /*or CV_MAKETYPE(CV_64F,3)*/CV_64FC3);
//    cv::Mat _3dImage = xyz_belonging.XYZ;
//
//    const double bigZ = 10000.;
//    double q[4][4];
//    cv::Mat _Q(4, 4, CV_64F, q);
//    Q.convertTo(_Q, CV_64F);
//
//    int x, cols = disparity.cols;
//    //source: sbuf;       destination: dbuf
//    std::vector<double> _sbuf(cols+1), _dbuf(cols*3+1);
//    double* sbuf = &_sbuf[0], *dbuf = &_dbuf[0];
//
//    double minDisparity = FLT_MAX;
//
//    // NOTE: here we quietly assume that at least one pixel in the disparity map is not defined.
//    // and we set the corresponding Z's to some fixed big value.
//    if( handleMissingValues )
//    {
//        cv::minMaxIdx( disparity, &minDisparity, 0, 0, 0 );
//    }
//
//    //row = Height = 960:
//    ///输入的视差图为单通道, 故stype为C1
//    for( int y = 0; y < disparity.rows; y++ )
//    {
//        double *sptr = sbuf, *dptr = dbuf;
//        double qx = q[0][1]*y + q[0][3], qy = q[1][1]*y + q[1][3];
//        double qz = q[2][1]*y + q[2][3], qw = q[3][1]*y + q[3][3];
//
//        if( stype == CV_8UC1 )
//        {
//            const uchar* sptr0 = disparity.ptr<uchar>(y);
//            for( x = 0; x < cols; x++ )
//                sptr[x] = (float)sptr0[x];
//        }
//        else if( stype == CV_16SC1 )
//        {
//            const short* sptr0 = disparity.ptr<short>(y);
//            for( x = 0; x < cols; x++ )
//                sptr[x] = (float)sptr0[x];
//        }
//        else if( stype == CV_32SC1 )
//        {
//            const int* sptr0 = disparity.ptr<int>(y);
//            for( x = 0; x < cols; x++ )
//                sptr[x] = (float)sptr0[x];
//        }
//        else if( stype == CV_64FC1 )
//        {
//            const double* sptr0 = disparity.ptr<double>(y);
//            for( x = 0; x < cols; x++ )
//                sptr[x] = (double)sptr0[x];
//        }
//        else
//        {
//            sptr = (double*)disparity.ptr<double>(y);
//        }
//
//
//        if( dtype == CV_32FC3 )
//            dptr = _3dImage.ptr<double>(y);
//
//        ///以下为des赋值
//        //cols = 1280
//        for( x = 0; x < cols; x++, qx += q[0][0], qy += q[1][0], qz += q[2][0], qw += q[3][0] )
//        {
//            double d = sptr[x];
//            double iW = 1./(qw + q[3][2]*d);
//            double X = (qx + q[0][2]*d)*iW;
//            double Y = (qy + q[1][2]*d)*iW;
//            double Z = (qz + q[2][2]*d)*iW;
//            if( fabs(d-minDisparity) <= FLT_EPSILON )
//                Z = bigZ;
//
//            //added for 激光点以外的点设为0, not for -inf
//            if( 0 == d)
//            {
//                dptr[x*3] =   0;
//                dptr[x*3+1] = 0;
//                dptr[x*3+2] = 0;
//            }
//            //added end
//
//            dptr[x*3] =   (double)X;
//            dptr[x*3+1] = (double)Y;
//            dptr[x*3+2] = (double)Z;
//        }
//
//        if( dtype == CV_16SC3 )
//        {
//            short* dptr0 = _3dImage.ptr<short>(y);
//            for( x = 0; x < cols*3; x++ )
//            {
//                int ival = cvRound(dptr[x]);
//                dptr0[x] = cv::saturate_cast<short>(ival);
//            }
//        }
//        else if( dtype == CV_32SC3 )
//        {
//            int* dptr0 = _3dImage.ptr<int>(y);
//            for( x = 0; x < cols*3; x++ )
//            {
//                int ival = cvRound(dptr[x]);
//                dptr0[x] = ival;
//            }
//        }
//        else if( dtype == CV_64FC3)
//        {
//            //y: 行号, 0-959
//            double* dptr0 = _3dImage.ptr<double>(y);
//            //WIDTH = cols =1280
//            for( x = 0; x < cols*3; x++ )
//            {
//                int ival = cvRound(dptr[x]);
//                dptr0[x] = ival;
//
//                if(-2147483648 == dptr0[x])
//                {
//                    dptr0[x] = 0;
//                }
//
//                //-inf == -2147483648.000000
//                //printf("dptr0[%d]= %d\n", x, dptr0[x]);
//            }
//        }
//    }
}

void filterXYZ(cv::Mat& inputXYZ, cv::Point3d *outputP3d)
{
    int count_idx = 0;
    //row: 960
    for(int y = 0; y < inputXYZ.rows; y++)
    {
        //cols = 1280
        for(int x = 0; x < inputXYZ.cols; x++)
        {
            //inputXYZ.ptr<double>(y), 指向inputXYZ第y行第一个元素的指针
            if((0 != inputXYZ.ptr<double>(y)[3*x]) &&
                (0 != inputXYZ.ptr<double>(y)[3*x+1]) &&
                (0 != inputXYZ.ptr<double>(y)[3*x+1]))
            {
                outputP3d[count_idx].x = inputXYZ.ptr<double>(y)[3*x];
                outputP3d[count_idx].y = inputXYZ.ptr<double>(y)[3*x+1];
                outputP3d[count_idx].z = inputXYZ.ptr<double>(y)[3*x+2];

                count_idx++;
            }
        }
    }

}

