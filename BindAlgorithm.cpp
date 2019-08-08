#include "BindAlgorithm.h"

/*********************多线激光的匹配********************/
int MatchAndBindAlgorithm(std::vector<Point2dPack> &Slice0, std::vector<Point2dPack> &Slice1, std::vector<P2dPackMatch> &outputMatchedSlice)
{
    P2dPackMatch tempPackMatch;
    Point2dPack tempfitPoint;

    int kk = 0;
    int size0;
    int size1;

    memset(&tempfitPoint, 0, sizeof(Point2dPack));
    ///如果求距离最远
    //tempfitPoint.x or y = MAX(1280 or 960);  //如果求距离最远  //flq

    size0 = Slice0.size();
    size1 = Slice1.size();

    //遍历IMG0中的一个slice的值
    for (int ii = 0; ii < size0; ii++)
    {
        //遍历IMG1中的一个slice的值
        for (int jj = 0; jj < size1; jj++)
        {
            if(Slice0[ii].PtBelonging == Slice1[jj].PtBelonging)
            {
                //1.进行排序，找到两图的slice中，PtBelonging相同的点
                SelectMostFitPoint(Slice0[ii], Slice1[jj], tempfitPoint);

                //2.绑定操作
                //插入一项匹配点P2dPackMatch
                outputMatchedSlice.push_back(tempPackMatch);
                //为新插入的匹配点赋值
                memcpy(&outputMatchedSlice[kk].p2dPack[0], &Slice0[ii], sizeof(Point2dPack)); //左侧的匹配点
                memcpy(&outputMatchedSlice[kk].p2dPack[1], &Slice1[jj], sizeof(Point2dPack)); //右侧的匹配点
                outputMatchedSlice[kk].belonging = Slice0[ii].PtBelonging;  //该点属于哪条激光线
                kk++;
            }

        }

    }
}

//P0点固定，逐个遍历P1点，满足条件的P1点赋值给fitterPt
int SelectMostFitPoint(Point2dPack &inputPt0, Point2dPack &inputPt1, Point2dPack &fitterPt)
{
#ifdef CAMERA_HORIZONTAL
    if(fabs(inputPt0.Pt2d.x - inputPt1.Pt2d.x) < fabs(inputPt0.Pt2d.x - fitterPt.Pt2d.x))
    {
        memcpy(&fitterPt, &inputPt1, sizeof(Point2dPack));
    }
#else
    if(fabs(inputPt0.Pt2d.y - inputPt1.Pt2d.y) < fabs(inputPt0.Pt2d.y - fitterPt.Pt2d.y))
    {
        memcpy(&fitterPt, &inputPt1, sizeof(Point2dPack));
    }
#endif

    //memcpy(&fitterPt, &inputPt1, sizeof(Point2dPack));
    return 0;
}
