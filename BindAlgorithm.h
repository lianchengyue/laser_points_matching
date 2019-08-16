#ifndef BINDALGORITHM_H
#define BINDALGORITHM_H

#include "Utils.h"

/*********************多线激光的匹配********************/
int MatchAndBindAlgorithm(std::vector<Point2dPack> &Slice0, std::vector<Point2dPack> &Slice1, std::vector<P2dPackMatch> &SliceMatched);

//每条线的激光点每行只出现一次
int SelectMostFitPoint(Point2dPack &inputPt0, Point2dPack &inputPt1, Point2dPack &fitterPt);
//Enhanced, 每条线的激光点每行可出现多次
int SelectFitterPoint(Point2dPack &inputPt0, Point2dPack &inputPt1, Point2dPack &fitterPt, int Slice1_pt_idx, int &fitter_idx);

#endif // BINDALGORITHM_H
