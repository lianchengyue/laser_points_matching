#ifndef STEREOCALIBLEFTANDRIGHT_H
#define STEREOCALIBLEFTANDRIGHT_H

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"

int StereoMatch(cv::Mat *rgbImageL, cv::Mat *rgbImageR);

#endif // STEREOCALIBLEFTANDRIGHT_H
