#include "Zhangsuen.h"
#include "Steger.h"
#include "EpipolarMatching.h" //基于极线约束的激光条纹匹配算
#include "ReferenceLine.h"  //点与线绑定
#include "Utils.h"

#include <iostream>

#include "StereoMatchLeftAndRight.h"

//#define TEMP_SET_OK

//Camera Matrix
//[fx  s   x0]
//[0   fy  y0]
//[0   0   1 ]
//入口函数
int StereoMatch(cv::Mat *rgbImageL, cv::Mat *rgbImageR)
{
    //if 0，用来解决内存泄漏问题
#if 0
    cv::Mat *remapedImg0 = new cv::Mat(cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3)); //CV_8U
    cv::Mat *remapedImg1 = new cv::Mat(cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3)); //CV_8U

    //双目立体校正，使符合对极约束
    DoStereoRectify(rgbImageL, rgbImageR, remapedImg0, remapedImg1);

    //开始处理对齐后的图像
    PostProcess(remapedImg0, remapedImg1);

    free(remapedImg0);
    free(remapedImg1);
#else
    cv::Mat remapedImg0 = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);
    cv::Mat remapedImg1 = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);

    //双目立体校正，使符合对极约束
    DoStereoRectify(rgbImageL, rgbImageR, &remapedImg0, &remapedImg1);

    //开始处理对齐后的图像
    PostProcess(&remapedImg0, &remapedImg1);
#endif
    return 0;
}

//双目立体校正，使符合对极约束
int DoStereoRectify(cv::Mat *rgbImageL, cv::Mat *rgbImageR, cv::Mat *afterRemapImage0, cv::Mat *afterRemapImage1)
{
    cv::Size imageSize = cv::Size(WIDTH, HEIGHT);

    //图片
    cv::Mat grayImage0;
    cv::Mat grayImage1;
    cv::Mat rectifyImage0, rectifyImage1;

    //图像校正之后的裁剪
    cv::Rect validROIL;//图像校正之后，会对图像进行裁剪，这里的validROI就是指裁剪之后的区域
    cv::Rect validROIR;

    cv::Mat mapLx, mapLy, mapRx, mapRy;     //映射表
    cv::Mat Rl, Rr, Pl, Pr, Q;              //校正旋转矩阵R，投影矩阵P 重投影矩阵Q

    //标定得到的内外参
    cv::Mat cameraMatrixL = (cv::Mat_<double>(3, 3)
                         <<
                         2232.550777775964, 0, 669.3747410537889,
                         0, 2231.856456030659, 473.3409386246501,
                         0, 0, 1);
    cv::Mat distCoeffL = (cv::Mat_<double>(5, 1)
                         <<
                         -0.09435279920653783, 0.2562075510521074, 0.0001898762951137649, 0.0005216752373549194, -0.7544675722826982);

    cv::Mat cameraMatrixR = (cv::Mat_<double>(3, 3)
                         <<
                         2223.192609925644, 0, 625.0792794020214,
                         0, 2222.866016106315, 482.1136582945432,
                         0, 0, 1);
    cv::Mat distCoeffR = (cv::Mat_<double>(5, 1)
                         <<
                         -0.08759498549994428, 0.09840725698052433, 0.0005933796478505113, 0.0006947082161594437, 0.4275222788377662);

    cv::Mat T = (cv::Mat_<double>(3, 1)
                         <<
                         6.5399539369845607e+00,
                         -6.7033329436584927e+02,
                         1.6347222727009409e+02);//T平移向量

    cv::Mat R = (cv::Mat_<double>(3, 3)
                         <<
                         9.9888958964013941e-01, 1.2919080759919396e-02,
                         4.5306567524725061e-02, -3.4550520051047279e-02,
                         8.5465602738265134e-01, 5.1804375917745304e-01,
                         -3.2028881853039620e-02, -5.1903388349010837e-01,
                         8.5415336943456510e-01);//R 旋转矩阵

    //立体校正
    //calc: Rl, Rr, Pl, Pr, Q
    //alpha-拉伸参数。如果设置为负或忽略，将不进行拉伸。
    //如果设置为0，那么校正后图像只有有效的部分会被显示（没有黑色的部分），如果设置为1，那么就会显示整个图像。设置为0~1之间的某个值，其效果也居于两者之间。
    stereoRectify(cameraMatrixL, distCoeffL, cameraMatrixR, distCoeffR, imageSize, R, T, Rl, Rr, Pl, Pr, Q, cv::CALIB_ZERO_DISPARITY,
        1/***-1***/, imageSize, &validROIL, &validROIR);

    std::cout << "cameraMatrixL"  << cameraMatrixL << std::endl;
    std::cout << "distCoeffL"  << distCoeffL << std::endl << std::endl;

    std::cout << "cameraMatrixR"  << cameraMatrixR << std::endl;
    std::cout << "distCoeffR"  << distCoeffR << std::endl << std::endl;

    std::cout << "R"  << R << std::endl;
    std::cout << "T"  << T << std::endl;
    std::cout << "Rl"  << Rl << std::endl;
    std::cout << "Rr"  << Rr << std::endl;
    std::cout << "Pl"  << Pl << std::endl;
    std::cout << "Pr"  << Pr << std::endl;
    std::cout << "Q"  << Q << std::endl;

    bool isVerticalStereo =false;

    //Hartley算法的主要思想：
    //1：通过对右图进行H2变换，把右图图像上的外极点映射到水平方向上的无穷远点，使右图图像上的外极线相互平行，并且平行于图像的水平扫描线;
    //2：寻找左图图像上的变换矩阵H1，使得变换后图像上对应点之间的视差最小化。
    //https://blog.csdn.net/hit1524468/article/details/79782685
    {
        cv::Mat H1, H2;
        ///计算单应性矩阵, out:H1，H2
        //stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F, imageSize, H1, H2, 3);
        H1 = (cv::Mat_<double>(3, 3) << -2.189435000927211e-05, 0.005198046968966729, -5.974651645774721,
                -0.0054667715764329, 0.0002526204691698786, 0.1663784814071454,
                7.470224965310847e-08, 7.58412151497686e-07, -0.005811322477235398);
        H2 = (cv::Mat_<double>(3, 3) << 0.009932963123129343, -0.9282435968593886, 1079.199830093704,
                0.9993673216478258, 0.06447456826965205, -190.5428786240415,
                -1.19881155519485e-06, 0.0001120299286483292, 0.9469928736441267);

        Rl = cameraMatrixL.inv() * H1 * cameraMatrixL;
        Rr = cameraMatrixR.inv() * H2 * cameraMatrixR;
        Pl = cameraMatrixL;
        Pr = cameraMatrixR;
    }

    initUndistortRectifyMap(cameraMatrixL, distCoeffL, Rl, Pl, imageSize, CV_32FC1, mapLx, mapLy);//before CV_32FC1, after CV_16SC2
    initUndistortRectifyMap(cameraMatrixR, distCoeffR, Rr, Pr, imageSize, CV_32FC1, mapRx, mapRy);

    //读取图片
    //rgbImageL = imread("./twoline/Left0.bmp", CV_LOAD_IMAGE_COLOR);
    cvtColor(*rgbImageL, grayImage0, CV_BGR2GRAY);
    //rgbImageR = imread("./twoline/Right0.bmp", CV_LOAD_IMAGE_COLOR);
    cvtColor(*rgbImageR, grayImage1, CV_BGR2GRAY);

    imshow("ImageL Before Rectify", grayImage0);
    imshow("ImageR Before Rectify", grayImage1);

    //经过remap之后，左右相机的图像已经共面并且行对准了
    //output: rectifyImage0, rectifyImage1
    remap(grayImage0, rectifyImage0, mapLx, mapLy, cv::INTER_LINEAR);
    remap(grayImage1, rectifyImage1, mapRx, mapRy, cv::INTER_LINEAR);

    //把校正结果显示出来
    cv::Mat rgbrectifyImage0, rgbrectifyImage1;
    cvtColor(rectifyImage0, rgbrectifyImage0, CV_GRAY2BGR);  //伪彩色图
    cvtColor(rectifyImage1, rgbrectifyImage1, CV_GRAY2BGR);

    //单独显示
    //rectangle(rgbrectifyImage0, validROIL, cv::Scalar(0, 0, 255), 3, 8);
    //rectangle(rgbrectifyImage1, validROIR, cv::Scalar(0, 0, 255), 3, 8);
    imshow("ImageL After Rectify", rgbrectifyImage0);
    imshow("ImageR After Rectify", rgbrectifyImage1);
#ifdef IMG_DUMP
    imwrite("remaped0.jpg", rgbrectifyImage0);
    imwrite("remaped1.jpg", rgbrectifyImage1);
#endif

    //显示在同一张图上
    cv::Mat canvas;
    double sf;
    int w, h;

    ////画对极约束图
    //创建画布
    if( !isVerticalStereo )
    {
        sf = 960. / MAX(imageSize.width, imageSize.height);   //600
        w = cvRound(imageSize.width * sf);
        h = cvRound(imageSize.height * sf);
        canvas.create(h, w * 2, CV_8UC3);   //注意通道
    }
    else
    {
        sf = 480./MAX(imageSize.width, imageSize.height);     //300
        w = cvRound(imageSize.width * sf);
        h = cvRound(imageSize.height * sf);
        canvas.create(h * 2, w, CV_8UC3);
    }

    ////Display
    //左/上图像画到画布上
    cv::Mat canvasPart = !isVerticalStereo ?
                        canvas(cv::Rect(w * 0, 0, w, h)) : canvas(cv::Rect(0, h*0, w, h));  //得到画布的一部分
    resize(rgbrectifyImage0, canvasPart, canvasPart.size(), 0, 0, cv::INTER_AREA);     //把图像缩放到跟canvasPart一样大小
    cv::Rect vroiL(cvRound(validROIL.x*sf), cvRound(validROIL.y*sf),                //获得被截取的区域
        cvRound(validROIL.width*sf), cvRound(validROIL.height*sf));
    //rectangle(canvasPart, vroiL, cv::Scalar(0, 0, 255), 3, 8);                      //画上一个矩形
    std::cout << "Painted ImageL" << std::endl;

    //右/下图像画到画布上
    canvasPart = !isVerticalStereo ?
                canvas(cv::Rect(w * 1, 0, w, h)) : canvas(cv::Rect(0, h*1, w, h));            //获得画布的另一部分

    resize(rgbrectifyImage1, canvasPart, canvasPart.size(), 0, 0, cv::INTER_LINEAR);
    cv::Rect vroiR(cvRound(validROIR.x * sf), cvRound(validROIR.y*sf),
        cvRound(validROIR.width * sf), cvRound(validROIR.height * sf));

    std::cout << "Painted ImageR" << std::endl;

    //画上对应的线条
    if(!isVerticalStereo)
    {
        for (int i = 0; i < canvas.rows; i += 16)
            line(canvas, cv::Point(0, i), cv::Point(canvas.cols, i), cv::Scalar(0, 255, 0), 1, 8);
        imshow("rectified", canvas);
    }
    else
    {
        for (int i = 0; i < canvas.rows; i += 16)
            line(canvas, cv::Point(i, 0), cv::Point(i, canvas.rows), cv::Scalar(0, 255, 0), 1, 8);
        imshow("rectified", canvas);
    }

    *afterRemapImage0 = rgbrectifyImage0.clone();
    *afterRemapImage1 = rgbrectifyImage1.clone();

    return 0;
}

//处理对齐后的图像
int PostProcess(cv::Mat *inputImg0, cv::Mat *inputImg1)
{
    ///开始提取激光点
    std::vector<cv::Point2d>  img0_pts;
    std::vector<cv::Point2d>  img1_pts;

    //clear vector here
    //added by flq,  清空临时的vector
    std::vector<cv::Point2d>().swap(img0_pts);
    std::vector<cv::Point2d>().swap(img1_pts);
#if 0
    //Zhang方法
    LaserPointExtract_Zhang(inputImg0, img0_pts, 0); //rgbrectifyImage0

    LaserPointExtract_Zhang(inputImg1, img1_pts, 1); //rgbrectifyImage1
#else
    //Steger方法
    LaserPointExtract_Steger(inputImg0, img0_pts, 0); //rgbrectifyImage0

    LaserPointExtract_Steger(inputImg1, img1_pts, 1); //rgbrectifyImage1
#endif

    //Reference Line
    std::vector<Point2dPack>  img0_ref;
    std::vector<Point2dPack>  img1_ref;
    //clear vector here

    //added by flq,  清空临时的vector
    std::vector<Point2dPack>().swap(img0_ref);
    std::vector<Point2dPack>().swap(img1_ref);

    //确定激光点所在的激光线
    excuteReference0(img0_pts, img0_ref);
    excuteReference1(img1_pts, img1_ref);

    //激光线条匹配
    EpipolarMatch(img0_ref, img1_ref);

    ///temporily moved by flq
    cv::waitKey(0);
    return 0;
}
