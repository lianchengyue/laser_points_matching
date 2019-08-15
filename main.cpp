#include "StereoMatchLeftAndRight.h"

//#include "opencv2/calib3d/calib3d.hpp"
//#include "opencv2/imgproc.hpp"
//#include "opencv2/imgcodecs.hpp"
//#include "opencv2/highgui.hpp"
//#include "opencv2/core/utility.hpp"

int main()
{
    cv::Mat *rgbImageL, *rgbImageR;
    cv::Mat *offlineImageL, *offlineImageR;

    rgbImageL = new cv::Mat(cv::Mat::zeros(960, 1280, CV_8UC3)); //CV_8U
    rgbImageR = new cv::Mat(cv::Mat::zeros(960, 1280, CV_8UC3)); //CV_8U

#if 0
//测试数据
    *rgbImageL = cv::imread("./testdata/left/4_laser.bmp", CV_LOAD_IMAGE_COLOR);//4_laser, 4
    *rgbImageR = cv::imread("./testdata//right/4_laser.bmp", CV_LOAD_IMAGE_COLOR);//4_laser

//下面均有wheel
#elif 0
    *rgbImageL = cv::imread("./twoline/Left_wheel.bmp", CV_LOAD_IMAGE_COLOR);//Left_wheel
    *rgbImageR = cv::imread("./twoline/Right_wheel.bmp", CV_LOAD_IMAGE_COLOR);//Right_Wheel
#elif 0
    *rgbImageL = cv::imread("./realwheel/Image_left.bmp", CV_LOAD_IMAGE_COLOR);//Left_wheel
    *rgbImageR = cv::imread("./realwheel/Image_right.bmp", CV_LOAD_IMAGE_COLOR);//Right_Wheel
#elif 0
    //竖直方向，实景,标定之前移动过
    *rgbImageL = cv::imread("./ws/up.bmp", CV_LOAD_IMAGE_COLOR);//Left_wheel
    *rgbImageR = cv::imread("./ws/down.bmp", CV_LOAD_IMAGE_COLOR);//Right_Wheel
#elif 0
    //2019.08.05竖直方向，标定板
    *rgbImageL = cv::imread("./calib/top0.bmp", CV_LOAD_IMAGE_COLOR);//Left_wheel
    *rgbImageR = cv::imread("./calib/bottom0.bmp", CV_LOAD_IMAGE_COLOR);//Right_Wheel
#else
    //2019.08.05竖直方向，实景
    *rgbImageL = cv::imread("./calib/top_laser.bmp", CV_LOAD_IMAGE_COLOR);//Left_wheel
    *rgbImageR = cv::imread("./calib/bottom_laser.bmp", CV_LOAD_IMAGE_COLOR);//Right_Wheel
#endif

    StereoMatch(rgbImageL, rgbImageR, offlineImageL, offlineImageR);

    cv::waitKey(0);

    //free
    //rgbImageL ,rgbImageR
    return 0;
}
