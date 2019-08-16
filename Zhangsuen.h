#ifndef ZHANGSUEN_H
#define ZHANGSUEN_H


#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"

int LaserPointExtract(cv::Mat imageInput);
int LaserPointExtract_Zhang(cv::Mat *imageInput, std::vector<cv::Point2d> &points, int dev_num);

#endif // ZHANGSUEN_H
