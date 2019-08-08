#ifndef DISPARITYANDDEPTH_H
#define DISPARITYANDDEPTH_H
/***************根据匹配点，得到视差与深度************/

#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"

#include "Utils.h"

int CalcDepthFromBeMatchedPoints(BeMatchedPoints *inputMatchedPts);

#endif // DISPARITYANDDEPTH_H
