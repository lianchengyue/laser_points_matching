#ifndef DISPARITYANDDEPTH_H
#define DISPARITYANDDEPTH_H
/***************根据匹配点，得到视差与深度************/

#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include "Utils.h"

int CalcDepthFromBeMatchedPoints(BeMatchedPoints *inputMatchedPts);
int CalcDisparityFromBeMatchedPoints(BeMatchedPoints *inputMatchedPts);

int Calc3DFromDisparity(DisparityAndBelonging &disp_belonging);
void customizeReprojectImageTo3D( cv::InputArray _disparity,
                             cv::OutputArray __3dImage, cv::InputArray _Qmat,
                             bool handleMissingValues, int dtype );
void customizeReprojectImageTo3D2(/*DisparityAndBelonging disp_belonging,*/
                             /*XYZAndBelonging xyz_belonging,*/ cv::InputArray _Qmat,
                             bool handleMissingValues, int dtype );
void saveXYZ(const char* filename, const cv::Mat& mat);
void filterXYZ(cv::Mat& inputXYZ, cv::Point3d *outputP3d);

#endif // DISPARITYANDDEPTH_H
