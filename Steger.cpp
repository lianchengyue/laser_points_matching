#include "Steger.h"
#include "Utils.h"

//求该点的灰度重心
//第i，j个点像素值
double ijpixel1(double& x, double& y, cv::Mat& m)
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
void CalcNormVec1(cv::Point2d ptA, cv::Point2d ptB, cv::Point2d ptC, double& pfCosSita, double& pfSinSita)
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

int gdd(cv::Mat &inputImg, std::vector<cv::Point2d> &points, std::vector<cv::Point2d> &output_points)
{
    std::vector<double> kcal;
    for (int i = 1; i < points.size() - 1; i++)
    {
        //normal
        double pfCosSita=0, pfSinSita=0;
        CalcNormVec1(cv::Point2d(points[i - 1].x, points[i - 1].y), cv::Point2d(points[i].x, points[i].y), cv::Point2d(points[i + 1].x, points[i + 1].y), pfCosSita, pfSinSita);
        //gdd
        double sum=0, sum_sumx=0, sum_sumy=0;
        for (int j = 0; j < 2; j++)
        {
            if (j == 0)
            {
                double cj = points[i].x;
                double ci = points[i].y;
                sum = ijpixel1(cj, ci, inputImg);
                if(-1 == sum)
                {
                    continue;
                }

                sum_sumx = ijpixel1(cj, ci, inputImg)*cj;
                sum_sumy = ijpixel1(cj, ci, inputImg)*ci;
            }
            else
            {
                double x_cor = points[i].x + j*pfCosSita;
                double y_cor = points[i].y + j*pfSinSita;
                double x_cor1 = points[i].x - j*pfCosSita;
                double y_cor1 = points[i].y - j*pfSinSita;
                sum = sum + ijpixel1(x_cor, y_cor, inputImg) + ijpixel1(x_cor1, y_cor1, inputImg);
                sum_sumx = sum_sumx + ijpixel1(x_cor, y_cor, inputImg)*x_cor + ijpixel1(x_cor1, y_cor1, inputImg)*x_cor1;
                sum_sumy = sum_sumy + ijpixel1(x_cor, y_cor, inputImg)*y_cor + ijpixel1(x_cor1, y_cor1, inputImg)*y_cor1;
            }
        }

        //赋值后传出
        output_points.push_back(cv::Point2d(sum_sumx / sum, sum_sumy / sum));
        //图像中心线画出来
        //circle(inputImg, cv::Point(sum_sumx / sum, sum_sumy / sum), 1, cv::Scalar(255, 255, 255));
#ifdef DEBUG
//        printf("Point(%f, %f):\n", sum_sumx/sum, sum_sumy/sum);
#endif
    }

    return 0;
}

int LaserPointExtract_Steger(cv::Mat *imageInput, std::vector<cv::Point2d> &p2d, int dev_num)
{
//    cv::Mat img0 = cv::imread("2.bmp", 1); //0_laser.jpg  //2.bmp
//    cv::Mat img;
//    cvtColor(img0, img0, CV_BGR2GRAY);
//    img = img0.clone();

    ////std::vector<cv::Point2d>  p2d;  ///tqq
    cv::Mat img;
    cvtColor(*imageInput, *imageInput, CV_RGB2GRAY);
    img = imageInput->clone();

    //高斯滤波
    //注意:在求Hessian矩阵之前需要对图像进行高斯滤波
    img.convertTo(img, CV_32FC1);
    GaussianBlur(img, img, cv::Size(0, 0), 6, 6);

    //一阶偏导数
    cv::Mat m1, m2;
    m1 = (cv::Mat_<float>(1, 2) << 1, -1);  //x偏导
    m2 = (cv::Mat_<float>(2, 1) << 1, -1);  //y偏导

    cv::Mat dx, dy;
    filter2D(img, dx, CV_32FC1, m1);
    filter2D(img, dy, CV_32FC1, m2);

    //二阶偏导数
    cv::Mat m3, m4, m5; m3 = (cv::Mat_<float>(1, 3) << 1, -2, 1);  //二阶x偏导
    m4 = (cv::Mat_<float>(3, 1) << 1, -2, 1);  //二阶y偏导
    m5 = (cv::Mat_<float>(2, 2) << 1, -1, -1, 1);  //二阶xy偏导

    cv::Mat dxx, dyy, dxy;
    filter2D(img, dxx, CV_32FC1, m3);
    filter2D(img, dyy, CV_32FC1, m4);
    filter2D(img, dxy, CV_32FC1, m5);

    //hessian矩阵
    double maxD = -1;
    int imgcol = img.cols;
    int imgrow = img.rows;
    std::vector<double> Pt;

    printf("imgcol=%d, imgrow=%d\n", imgcol, imgrow);
    for (int i=0;i<imgcol;i++)//1280
    {
        for (int j=0;j<imgrow;j++) //960
        {
            //200:亮度门限
            if (imageInput->at<uchar>(j,i)>200)
            {
                cv::Mat hessian(2, 2, CV_32FC1);
                hessian.at<float>(0, 0) = dxx.at<float>(j, i);
                hessian.at<float>(0, 1) = dxy.at<float>(j, i);
                hessian.at<float>(1, 0) = dxy.at<float>(j, i);
                hessian.at<float>(1, 1) = dyy.at<float>(j, i);
                //std::cout << "hessian:" << endl << hessian << endl;

                cv::Mat eValue;
                cv::Mat eVectors;

                //src – input matrix that must have CV_32FC1 or CV_64FC1 type, square size and be symmetrical (src T == src).
                //eigenvalues – output vector of eigenvalues of the same type as src; the eigenvalues are stored in the descending order.
                //eigenvectors – output matrix of eigenvectors; it has the same size and type as src; the eigenvectors are stored as subsequent matrix rows, in the same order as the corresponding eigenvalues.
                //特征值（eigenvalue),特征向量（eigenvector）,特征值分解（eigenvalue decomposition）
                //https://blog.csdn.net/zhengwei223/article/details/78913898
                cv::eigen(hessian, eValue, eVectors);

                ///nx:特征向量
                ///ny:特征向量
                ///t:对t求微分
                double nx, ny;
                double fmaxD = 0;
                //fabs功能：求浮点数x的绝对值
                if (fabs(eValue.at<float>(0,0))>= fabs(eValue.at<float>(1,0))) //求特征值最大时对应的特征向量
                {
                    nx = eVectors.at<float>(0, 0);
                    ny = eVectors.at<float>(0, 1);
                    fmaxD = eValue.at<float>(0, 0);
                }
                else
                {
                    nx = eVectors.at<float>(1, 0);
                    ny = eVectors.at<float>(1, 1);
                    fmaxD = eValue.at<float>(1, 0);
                }

                double t = -(nx*dx.at<float>(j, i) + ny*dy.at<float>(j, i)) / (nx*nx*dxx.at<float>(j,i)+2*nx*ny*dxy.at<float>(j,i)+ny*ny*dyy.at<float>(j,i));

                if (fabs(t*nx)<=0.5 && fabs(t*ny)<=0.5)
                {

                    Pt.push_back(i);
                    Pt.push_back(j);
                }

            }
        }
    }

    printf("图像中获取激光点的个数:%d\n\n", Pt.size()/2);
    for (int k = 0; k<Pt.size()/2; k++)
    {
        cv::Point rpt;
        rpt.x = Pt[2 * k + 0];
        rpt.y = Pt[2 * k + 1];
        circle(*imageInput, rpt, 1.0/*1*/, cv::Scalar(127, 127, 127));//255

#ifdef DEBUG
        printf("(%d,%d)\n",rpt.x, rpt.y);
#endif
        //set value
        cv::Point2d point2d;
        point2d.x = rpt.x;
        point2d.y = rpt.y;
        p2d.push_back(point2d);
    }

    //imshow("result", img0);
    //waitKey(0);

    //灰度质心法
    //img: 高斯滤波后的图像
#if 0 //tqq
    std::vector<cv::Point2d>  output_points;
    gdd(/**imageInput*/img, p2d, output_points);
    imshow("img", *imageInput);
#endif

#ifdef IMG_DISPLAY
    //显示图像
    cv::Mat disImg = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);

    for (int i = 0; i < p2d.size() - 1; i++)
    {
        disImg.at<cv::Vec3b>(round(p2d[i].y), round(p2d[i].x)) = cv::Vec3b(0, 255, 255);
    }

    char *strname = new char[255];
    sprintf(strname, "the %d camera color display:", dev_num);
    imshow(strname, disImg);
    free(strname);
#endif

    return 0;
}
