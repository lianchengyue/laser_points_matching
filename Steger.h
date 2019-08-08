#ifndef STEGER_H
#define STEGER_H

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"

int LaserPointExtract_Steger(cv::Mat *imageInput, std::vector<cv::Point2d> &/*output_points*/p2d, int dev_num);

#endif // STEGER_H
