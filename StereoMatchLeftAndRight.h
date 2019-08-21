#ifndef STEREOCALIBLEFTANDRIGHT_H
#define STEREOCALIBLEFTANDRIGHT_H

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"

//入口函数
int StereoMatch(cv::Mat *rgbImageL, cv::Mat *rgbImageR);
//双目立体校正，使符合对极约束
int DoStereoRectify(cv::Mat *rgbImageL, cv::Mat *rgbImageR, cv::Mat *LaserInputImage0, cv::Mat *LaserInputImage1);
//处理对齐后的图像
int PostProcess(cv::Mat *inputImg0, cv::Mat *inputImg1);

#endif // STEREOCALIBLEFTANDRIGHT_H
