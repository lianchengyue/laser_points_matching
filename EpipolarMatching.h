#ifndef EPIPOLARMATCHING_H
#define EPIPOLARMATCHING_H

#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"

#include "Utils.h"

/***************基于极线约束的激光条纹匹配算法************/
int EpipolarMatch(std::vector<Point2dPack> &leftPoints, std::vector<Point2dPack> &rightPoints);

///1:按行/列整理顺序
//PointsInput:  输入,图0or图1提取到的激光点
//nSliceOutput:  输出,图0or图1的整理点
int EpipolarPointOrder(std::vector<Point2dPack> &PointsInput, UnMatchedPoints  *outputSlice);
///2:每行/列中 小->大 sort
int EpipolarPonitSort(UnMatchedPoints *nSliceInput);
///3:左右图中的激光点匹配
int EpipolarPonitBind(UnMatchedPoints *nSliceLeft, UnMatchedPoints *nSliceRight, BeMatchedPoints *outputSlice, int &total_matched_cnt);


//输入图像的激光点的总数
int AlignedInputLaserPointsCnt(UnMatchedPoints *nSlice0, int camID);
//统计匹配的激光点的总数
int MatchedPointsCnt(BeMatchedPoints *outputSlice);

#endif // EPIPOLARMATCHING_H
