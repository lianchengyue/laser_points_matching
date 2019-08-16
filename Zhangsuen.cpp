//https://blog.csdn.net/SLAM_masterFei/article/details/88982874
//灰度重心法公式大全
//https://blog.csdn.net/lyc_daniel/article/details/7869338

#include "Zhangsuen.h"
#include "Utils.h"

//水平方向为亚像素级
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <fstream>
#include <stdlib.h>


using namespace std;
using namespace cv;


//zhang-suen细化算法
void zhang(Mat& input, Mat& output)
{
    Mat copymat;
    input.copyTo(copymat);
    int k = 0;
    //防止溢出
    while (1)
    {
        k++;
        bool stop = false;
        //step1
        for (int i = 1; i < input.cols - 1; i++)
            for (int j = 1; j < input.rows - 1; j++)
            {
                if (input.at<uchar>(j, i)>0)
                {
                    int p1 = int(input.at<uchar>(j, i))>0 ? 1 : 0;
                    int p2 = int(input.at<uchar>(j - 1, i))>0 ? 1 : 0;
                    int p3 = int(input.at<uchar>(j - 1, i + 1))>0 ? 1 : 0;
                    int p4 = int(input.at<uchar>(j, i + 1))>0 ? 1 : 0;
                    int p5 = int(input.at<uchar>(j + 1, i + 1))>0 ? 1 : 0;
                    int p6 = int(input.at<uchar>(j + 1, i))>0 ? 1 : 0;
                    int p7 = int(input.at<uchar>(j + 1, i - 1))>0 ? 1 : 0;
                    int p8 = int(input.at<uchar>(j, i - 1))>0 ? 1 : 0;
                    int p9 = int(input.at<uchar>(j - 1, i - 1))>0 ? 1 : 0;
                    int np1 = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
                    int sp2 = (p2 == 0 && p3 == 1) ? 1 : 0;
                    int sp3 = (p3 == 0 && p4 == 1) ? 1 : 0;
                    int sp4 = (p4 == 0 && p5 == 1) ? 1 : 0;
                    int sp5 = (p5 == 0 && p6 == 1) ? 1 : 0;
                    int sp6 = (p6 == 0 && p7 == 1) ? 1 : 0;
                    int sp7 = (p7 == 0 && p8 == 1) ? 1 : 0;
                    int sp8 = (p8 == 0 && p9 == 1) ? 1 : 0;
                    int sp9 = (p9 == 0 && p2 == 1) ? 1 : 0;
                    int sp1 = sp2 + sp3 + sp4 + sp5 + sp6 + sp7 + sp8 + sp9;

                    if (np1 >= 2 && np1 <= 6 && sp1 == 1 && ((p2*p4*p6) == 0) && ((p4*p6*p8) == 0))
                    {
                        stop = true;
                        copymat.at<uchar>(j, i) = 0;
                    }
                }
            }
#if 0
        //step2
        for (int i = 1; i < input.cols - 1; i++)
        {
            for (int j = 1; j < input.rows - 1; j++)
            {
                if (input.at<uchar>(j, i)>0)
                {
                    int p2 = int(input.at<uchar>(j - 1, i))>0 ? 1 : 0;
                    int p3 = int(input.at<uchar>(j - 1, i + 1)) > 0 ? 1 : 0;
                    int p4 = int(input.at<uchar>(j, i + 1)) > 0 ? 1 : 0;
                    int p5 = int(input.at<uchar>(j + 1, i + 1)) > 0 ? 1 : 0;
                    int p6 = int(input.at<uchar>(j + 1, i)) > 0 ? 1 : 0;
                    int p7 = int(input.at<uchar>(j + 1, i - 1)) > 0 ? 1 : 0;
                    int p8 = int(input.at<uchar>(j, i - 1)) > 0 ? 1 : 0;
                    int p9 = int(input.at<uchar>(j - 1, i - 1)) > 0 ? 1 : 0;
                    int np1 = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
                    int sp2 = (p2 == 0 && p3 == 1) ? 1 : 0;
                    int sp3 = (p3 == 0 && p4 == 1) ? 1 : 0;
                    int sp4 = (p4 == 0 && p5 == 1) ? 1 : 0;
                    int sp5 = (p5 == 0 && p6 == 1) ? 1 : 0;
                    int sp6 = (p6 == 0 && p7 == 1) ? 1 : 0;
                    int sp7 = (p7 == 0 && p8 == 1) ? 1 : 0;
                    int sp8 = (p8 == 0 && p9 == 1) ? 1 : 0;
                    int sp9 = (p9 == 0 && p2 == 1) ? 1 : 0;
                    int sp1 = sp2 + sp3 + sp4 + sp5 + sp6 + sp7 + sp8 + sp9;
                    if (np1 >= 2 && np1 <= 6 && sp1 == 1 && (p2*p4*p8) == 0 && (p2*p6*p8) == 0)
                    {
                        stop = true;
                        copymat.at<uchar>(j, i) = 0;
                    }
                }
            }
        }
#endif
        //将新得到的图片赋给新的图片
        copymat.copyTo(input);
        if (!stop)
        {
            break;
        }
    }
    copymat.copyTo(output);
}

//求该点的灰度重心
//第i，j个点像素值
double ijpixel(double& x, double& y, Mat& m)
{
    int x_0 = int(x);
    int x_1 = int(x + 1);
    int y_0 = int(y);
    int y_1 = int(y + 1);
    int px_0y_0 = int(m.at<uchar>(y_0, x_0));
    int px_0y_1 = int(m.at<uchar>(y_1, x_0));
    int px_1y_0 = int(m.at<uchar>(y_0, x_1));
    int px_1y_1 = int(m.at<uchar>(y_1, x_1));
    double x_y0 = px_0y_0 + (x - double(x_0))*(px_1y_0 - px_0y_0);
    double x_y1 = px_0y_1 + (x - double(x_0))*(px_1y_1 - px_0y_1);
    double x_y = x_y0 + (y - double(y_0))*(x_y1 - x_y0);

    //判断是否y值为小数
    if((x_y - (int)x_y) != 0)
    {
        //printf("XXX\n");
        return -1;
    }
    return x_y;
}

//normal vector
void CalcNormVec(Point2d ptA, Point2d ptB, Point2d ptC, double& pfCosSita, double& pfSinSita)
{
    double fVec1_x, fVec1_y, fVec2_x, fVec2_y;
    if (ptA.x == 0 && ptA.y == 0)
    {
        ptA.x = ptC.x;
        ptA.y = ptC.y;
        //先用B点坐标减A点坐标。
        fVec1_x = -(ptB.x - ptA.x);
        fVec1_y = -(ptB.y - ptA.y);
    }
    else
    {
        //先用B点坐标减A点坐标。
        fVec1_x = ptB.x - ptA.x;
        fVec1_y = ptB.y - ptA.y;
    }

    if (ptC.x == 0 && ptC.y == 0)
    {
        ptC.x = ptA.x;
        ptC.y = ptA.y;
        //再用C点坐标减B点坐标。
        fVec2_x = (ptB.x - ptC.x);
        fVec2_y = (ptB.y - ptC.y);
    }
    else
    {
        //再用C点坐标减B点坐标。
        fVec2_x = ptC.x - ptB.x;
        fVec2_y = ptC.y - ptB.y;
    }

    //单位化。
    double fMod = sqrt(fVec1_x * fVec1_x + fVec1_y * fVec1_y);
    fVec1_x /= fMod;
    fVec1_y /= fMod;
    //计算垂线。
    double fPerpendicularVec1_x = -fVec1_y;
    double fPerpendicularVec1_y = fVec1_x;


    //单位化。
    fMod = sqrt(fVec2_x * fVec2_x + fVec2_y * fVec2_y);
    fVec2_x /= fMod;
    fVec2_y /= fMod;
    //计算垂线。
    double fPerpendicularVec2_x = -fVec2_y;
    double fPerpendicularVec2_y = fVec2_x;
    //求和。
    double fSumX = fPerpendicularVec1_x + fPerpendicularVec2_x;
    double fSumY = fPerpendicularVec1_y + fPerpendicularVec2_y;
    //单位化。
    fMod = sqrt(fSumX * fSumX + fSumY * fSumY);
    double fCosSita = fSumX / fMod;
    double fSinSita = fSumY / fMod;
    pfCosSita = fCosSita;
    pfSinSita = fSinSita;
}

void extractPoint(Mat& inputimg, vector<Point2d>& pt)
{
    pt.push_back(Point2d(0, 0));
    for (int i = 0; i < inputimg.cols; i++)
        for (int j = 0; j < inputimg.rows; j++)
        {
            if (inputimg.at<uchar>(j, i) >= 95)
            {
                Point2d curr = Point2d(i, j);
                //printf("extractPoint, Point(%d, %d):\n", i, j);
                pt.push_back(curr);
            }
        }
    pt.push_back(Point2d(0, 0));
}

int LaserPointExtract(Mat imageInput)
{
    cv::namedWindow("h", CV_WINDOW_NORMAL | CV_WINDOW_FREERATIO | CV_GUI_EXPANDED);//CV_WINDOW_NORMAL
    cv::namedWindow("w", CV_WINDOW_NORMAL | CV_WINDOW_FREERATIO | CV_GUI_EXPANDED);

    //img1:提取完成的激光线
    Mat img1;
    //img2:GaussianBlur后的图像
    Mat img2;

    cvtColor(imageInput, imageInput, CV_RGB2GRAY);
    //对图像进行平滑处理
    GaussianBlur(imageInput, imageInput, Size(3, 3), 0);
    imageInput.copyTo(img2);
    //图像的二值化,  3==THRESH_TRUNC
    threshold(imageInput, imageInput, 95/*95*/, 255, THRESH_BINARY);

    //zhang-suen法，去掉额外的亮点
    zhang(imageInput, img1);
    vector<Point2d> points;

    //找到图像中的亮点
    extractPoint(img1, points);

    vector<double> kcal;
    for (int i = 1; i < points.size() - 1; i++)
    {
        //normal
        double pfCosSita=0, pfSinSita=0;
        CalcNormVec(Point2d(points[i - 1].x, points[i - 1].y), Point2d(points[i].x, points[i].y), Point2d(points[i + 1].x, points[i + 1].y), pfCosSita, pfSinSita);
        //gdd灰度重心法
        double sum=0, sum_sumx=0, sum_sumy=0;
        for (int j = 0; j < 2; j++)
        {
            if (j == 0)
            {
                double cj = points[i].x;
                double ci = points[i].y;
                sum = ijpixel(cj, ci, img2);
                sum_sumx = ijpixel(cj, ci, img2)*cj;
                sum_sumy = ijpixel(cj, ci, img2)*ci;
            }
            else
            {
                double x_cor = points[i].x + j*pfCosSita;
                double y_cor = points[i].y + j*pfSinSita;
                double x_cor1 = points[i].x - j*pfCosSita;
                double y_cor1 = points[i].y - j*pfSinSita;
                sum = sum + ijpixel(x_cor, y_cor, img2) + ijpixel(x_cor1, y_cor1, img2);
                sum_sumx = sum_sumx + ijpixel(x_cor, y_cor, img2)*x_cor + ijpixel(x_cor1, y_cor1, img2)*x_cor1;
                sum_sumy = sum_sumy + ijpixel(x_cor, y_cor, img2)*y_cor + ijpixel(x_cor1, y_cor1, img2)*y_cor1;
            }
        }
        //图像中心线画出来
        circle(img1, Point(sum_sumx / sum, sum_sumy / sum), 1, Scalar(255, 255, 255));
        printf("Point(%f, %f):\n", sum_sumx/sum, sum_sumy/sum);

    }

    imshow("h", imageInput);
    imshow("w", img2);
    imwrite("./Leftline_wheel.bmp", imageInput);

    system("pause");
    return 0;
}

//提取校准后的图像中的激光点
//param1:输入的图像
//param2:输出的激光点
///再下一步，对激光点做逐一匹配
int LaserPointExtract_Zhang(Mat *imageInput, std::vector<cv::Point2d> &output_points, int dev_num)
{
    printf("\n\n\nLaserPointExtract_zhang\n");
    //cv::namedWindow("h", CV_WINDOW_NORMAL | CV_WINDOW_FREERATIO | CV_GUI_EXPANDED);//CV_WINDOW_NORMAL
    //cv::namedWindow("w", CV_WINDOW_NORMAL | CV_WINDOW_FREERATIO | CV_GUI_EXPANDED);

    //img1:提取完成的激光线
    Mat img1;
    //img2:GaussianBlur后的图像
    Mat img2;

    cvtColor(*imageInput, *imageInput, CV_RGB2GRAY);
    //对图像进行平滑处理
    GaussianBlur(*imageInput, *imageInput, Size(3, 3), 0);
    imageInput->copyTo(img2);

    //图像的二值化,  3==THRESH_TRUNC
    threshold(*imageInput, *imageInput, BRIGHTNESS_THRESHHOLD/*95*/, 255, THRESH_BINARY);

    //zhang-suen法，去掉额外的亮点
    zhang(*imageInput, img1);

    std::vector<cv::Point2d> points;

    //找到图像中的亮点
    extractPoint(img1, points);
    printf("图像中获取激光点的个数:%d\n\n", points.size());

    vector<double> kcal;
    for (int i = 1; i < points.size() - 1; i++)
    {
        bool bContinue = false;

        //normal
        double pfCosSita=0, pfSinSita=0;
        CalcNormVec(Point2d(points[i - 1].x, points[i - 1].y), Point2d(points[i].x, points[i].y), Point2d(points[i + 1].x, points[i + 1].y), pfCosSita, pfSinSita);
        //gdd
        double sum=0, sum_sumx=0, sum_sumy=0;
        for (int j = 0; j < 2; j++)
        {
            if (j == 0)
            {
                double cj = points[i].x;
                double ci = points[i].y;
                sum = ijpixel(cj, ci, img2);
                //也x为小数，放弃
                if(-1 == sum)
                {
                    bContinue = true;
                    continue;
                }

                sum_sumx = ijpixel(cj, ci, img2)*cj;
                sum_sumy = ijpixel(cj, ci, img2)*ci;
            }
            else
            {
                double temp1, temp2;
                double x_cor = points[i].x + j*pfCosSita;
                double y_cor = points[i].y + j*pfSinSita;
                double x_cor1 = points[i].x - j*pfCosSita;
                double y_cor1 = points[i].y - j*pfSinSita;

                temp1 = ijpixel(x_cor, y_cor, img2);
                temp2 = ijpixel(x_cor1, y_cor1, img2);
                //也x为小数，放弃
                if(-1 == temp1 || -1 == temp2)
                {
                    bContinue = true;
                    continue;
                }

                sum = sum + ijpixel(x_cor, y_cor, img2) + ijpixel(x_cor1, y_cor1, img2);
                sum_sumx = sum_sumx + ijpixel(x_cor, y_cor, img2)*x_cor + ijpixel(x_cor1, y_cor1, img2)*x_cor1;
                sum_sumy = sum_sumy + ijpixel(x_cor, y_cor, img2)*y_cor + ijpixel(x_cor1, y_cor1, img2)*y_cor1;
            }
        }

        if(bContinue)
        {
            continue;
        }

        //赋值后传出
        output_points.push_back(Point2d(sum_sumx / sum, sum_sumy / sum));
        //图像中心线画出来
        circle(img1, Point(sum_sumx / sum, sum_sumy / sum), 1, Scalar(255, 255, 255));
#ifdef DEBUG
        printf("Point(%f, %f):\n", sum_sumx/sum, sum_sumy/sum);
#endif
    }

#ifdef IMG_DISPLAY
    //显示图像
    cv::Mat disImg = cv::Mat::zeros(960, 1280, CV_8UC3);

    for (int i = 0; i < output_points.size() - 1; i++)
    {
        disImg.at<cv::Vec3b>(round(points[i].x), round(points[i].y)) = cv::Vec3b(128, 255, 0);
    }

    char *strname = new char[255];
    sprintf(strname, "the %d camera color display:", dev_num);
    imshow(strname, disImg);
    free(strname);
#endif

    //waitKey(0);
    return 0;
}
