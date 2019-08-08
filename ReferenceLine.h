#ifndef REFERENCELINE_H
#define REFERENCELINE_H

#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"

#include "Utils.h"

/*********************激光点匹配到线的归属********************/
int excuteReference0(std::vector<cv::Point2d> &img_pts, std::vector<Point2dPack> &img_ref);
int excuteReference1(std::vector<cv::Point2d> &img_pts, std::vector<Point2dPack> &img_ref);
#endif // REFERENCELINE_H
