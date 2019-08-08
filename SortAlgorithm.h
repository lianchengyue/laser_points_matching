#ifndef SORTALGORITHM_H
#define SORTALGORITHM_H

#include "Utils.h"

//对每一行/列的激光点进行 小->大排序

int sortRowOrColumnLaserPoint(std::vector<Point2dPack> &inputData);  //行

//or
//对每一行/列的激光点进行 小->大排序
int VectorSort(std::vector<Point2dPack> &inputData);
//for test
int VectorSort();

#endif // SORTALGORITHM_H
