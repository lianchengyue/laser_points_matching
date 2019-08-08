#ifndef BINDALGORITHM_H
#define BINDALGORITHM_H

#include "Utils.h"

/*********************多线激光的匹配********************/
int SelectMostFitPoint(Point2dPack &inputPt0, Point2dPack &inputPt1, Point2dPack &fitterPt);
int MatchAndBindAlgorithm(std::vector<Point2dPack> &Slice0, std::vector<Point2dPack> &Slice1, std::vector<P2dPackMatch> &SliceMatched);


#endif // BINDALGORITHM_H
