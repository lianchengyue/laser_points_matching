#include "BindAlgorithm.h"

/*********************多线激光的匹配********************/
int MatchAndBindAlgorithm(std::vector<Point2dPack> &Slice0, std::vector<Point2dPack> &Slice1, std::vector<P2dPackMatch> &outputMatchedSlice)
{
    //未优化
#if 0
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
#ifdef DEBUG
    printf("Slice0.size=%d, Slice1.size=%d\n", size0, size1);
#endif

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
#else
    //优化,相比之前效果增强
    //Enhanced, 每条线的激光点每行可出现多次
    P2dPackMatch tempPackMatch;
    Point2dPack tempfitPoint;

    int kk = 0;
    int size0;
    int size1;

    int fitter_idx = 0;

    memset(&tempfitPoint, 0, sizeof(Point2dPack));
    ///如果求距离最远
    //tempfitPoint.x or y = MAX(1280 or 960);  //如果求距离最远  //flq

    size0 = Slice0.size();
    size1 = Slice1.size();
#ifdef DEBUG
    //打印每个slice(每一行)有几个点
    //调试初期需要，后期删除，减少LOG打印量
    //printf("Slice0.size=%d, Slice1.size=%d\n", size0, size1);
#endif

    //遍历IMG0中的一个slice的值
    for (int ii = 0; ii < size0; ii++)
    {
        //遍历IMG1中的一个slice的值
        for (int jj = 0; jj < size1; jj++)
        {
            if(Slice0[ii].PtBelonging == Slice1[jj].PtBelonging)
            {
                //1.进行排序，找到两图的slice中，PtBelonging相同的点
                SelectFitterPoint(Slice0[ii], Slice1[jj], tempfitPoint, jj, fitter_idx);

                //2.绑定操作

                //Slice0上的一个点遍历完成，进行赋值与绑定
                if(jj = size1 - 1)
                {
                    //Tip:初始化,给vector添加新的一项
                    outputMatchedSlice.push_back(tempPackMatch);
                    //为新插入的匹配点赋值
                    memcpy(&outputMatchedSlice[kk].p2dPack[0], &Slice0[ii], sizeof(Point2dPack)); //左侧的匹配点
                    memcpy(&outputMatchedSlice[kk].p2dPack[1], &Slice1[fitter_idx], sizeof(Point2dPack)); //右侧的匹配点
                    outputMatchedSlice[kk].belonging = Slice0[ii].PtBelonging;  //该点属于哪条激光线
                    kk++;
                }
                //未遍历完，继续
                else
                {
                    continue;
                }

            }
        }
    }
#endif

}


//P0点固定，逐个遍历P1点，满足条件的P1点赋值给fitterPt
int SelectMostFitPoint(Point2dPack &inputPt0, Point2dPack &inputPt1, Point2dPack &fitterPt)
{
    if(fabs(inputPt0.Pt2d.x - inputPt1.Pt2d.x) < fabs(inputPt0.Pt2d.x - fitterPt.Pt2d.x))
    {
        memcpy(&fitterPt, &inputPt1, sizeof(Point2dPack));
    }

    return 0;
}

//Slice1_pt_idx: Sice1中的第idx个点
//fitter_idx : 更适合的点的idx
int SelectFitterPoint(Point2dPack &inputPt0, Point2dPack &inputPt1, Point2dPack &fitterPt, int Slice1_pt_idx, int &fitter_idx)
{
    if(fabs(inputPt0.Pt2d.x - inputPt1.Pt2d.x) < fabs(inputPt0.Pt2d.x - fitterPt.Pt2d.x))
    {
        memcpy(&fitterPt, &inputPt1, sizeof(Point2dPack));
        //fitter_idx是更适合的点
        fitter_idx = Slice1_pt_idx;
    }

    return 0;
}
