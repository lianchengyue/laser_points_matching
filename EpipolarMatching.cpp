#include "EpipolarMatching.h"
#include "SortAlgorithm.h"
#include "DisparityAndDepth.h"
#include "DisplayImg.h"
#include "BindAlgorithm.h"

/*************************************视差细化（亚像素级）***********************************/
/***************基于极线约束的激光条纹匹配算法************/

/////flq, 激光纹理筛选+排序函数
//1:按行/列整理顺序
//2:每行/列中 小->大 sort
//3:左右图中的激光点匹配
int EpipolarMatch(std::vector<Point2dPack> &leftPoints, std::vector<Point2dPack> &rightPoints)
{
#ifdef CAMERA_HORIZONTAL
    UnMatchedPoints nSlice0[HEIGHT];
    memset(nSlice0, 0, HEIGHT * sizeof(UnMatchedPoints));

    UnMatchedPoints nSlice1[HEIGHT];
    memset(nSlice1, 0, HEIGHT * sizeof(UnMatchedPoints));

    //匹配后的集合
    BeMatchedPoints MatchedSlice[HEIGHT];
    memset(MatchedSlice, 0, HEIGHT * sizeof(UnMatchedPoints));
#else
    UnMatchedPoints nSlice0[WIDTH];
    memset(nSlice0, 0, WIDTH * sizeof(UnMatchedPoints));

    UnMatchedPoints nSlice1[WIDTH];
    memset(nSlice1, 0, WIDTH * sizeof(UnMatchedPoints));

    //匹配后的集合
    BeMatchedPoints MatchedSlice[WIDTH];
    memset(MatchedSlice, 0, WIDTH * sizeof(UnMatchedPoints));
#endif

    //1:按行/列整理顺序
    EpipolarPointOrder(leftPoints, nSlice0);  //图0
    EpipolarPointOrder(rightPoints, nSlice1);  //图1
    //2:每行/列中 小->大 sort
    EpipolarPonitSort(nSlice0);
    EpipolarPonitSort(nSlice1);
    //3:左右/上下图中的激光点匹配
    EpipolarPonitBind(nSlice0, nSlice1, MatchedSlice);

    //3.5:显示排序并分类好的图像，匹配前
    DisplayUnmatchedImg0(nSlice0);
    DisplayUnmatchedImg1(nSlice1);


#if 0//def OUTPUT_POINTS_DEBUG
    for (int ii = 0; ii < HEIGHT; ii++)
    {
        if((0 == MatchedSlice[ii].P2dMatchedSlice.size()))
        {
            //IMG0, IMG1任意一张图该行/列没有激光点
            continue;
        }

        printf("OutpointX: (%lf, %lf)\n", MatchedSlice[ii].P2dMatchedSlice[0].p2dPack[0].Pt2d.x, MatchedSlice[ii].P2dMatchedSlice[0].p2dPack[0].Pt2d.y);
    }
#endif

    //4:计算得到深度z
    CalcDepthFromBeMatchedPoints(MatchedSlice);


    return 0;
}

///1:按行/列整理顺序
int EpipolarPointOrder(std::vector<Point2dPack> &PointsInput, UnMatchedPoints *nSliceOutput)
{
#ifdef CAMERA_HORIZONTAL
    //提取Points,按从左到右
    for (int i = 0; i < PointsInput.size(); i++)
    {
        //判断是否是整数
        if(0 == (PointsInput[i].Pt2d.y - (int)PointsInput[i].Pt2d.y))  //if vertical, here is x
        {
            ///(int)(PointsInput[i].y):当前激光点的所在行
            ///将统一行/列的激光点放进该行/列
            PointsInput[i].PtRowIdx = PointsInput[i].Pt2d.y; //added by flq,激光线赋值
            nSliceOutput[(int)(PointsInput[i].Pt2d.y)].Slice.push_back(PointsInput[i]);

            nSliceOutput[(int)(PointsInput[i].Pt2d.y)].imgIdx = 0;
            nSliceOutput[(int)(PointsInput[i].Pt2d.y)].rowIdx = PointsInput[i].Pt2d.y;

            //...
        }
    }
#else
    //提取Points,按从左到右
    for (int i = 0; i < PointsInput.size(); i++)
    {
        //判断是否是整数
        if(0 == (PointsInput[i].Pt2d.x - (int)PointsInput[i].Pt2d.x))  //if vertical, here is x
        {
            ///(int)(PointsInput[i].y):当前激光点的所在行
            ///将同一列的激光点放进该列
            PointsInput[i].PtColIdx = PointsInput[i].Pt2d.x; //added by flq,激光线赋值
            nSliceOutput[(int)(PointsInput[i].Pt2d.x)].Slice.push_back(PointsInput[i]);

            nSliceOutput[(int)(PointsInput[i].Pt2d.x)].imgIdx = 0;
            nSliceOutput[(int)(PointsInput[i].Pt2d.x)].colIdx = PointsInput[i].Pt2d.x;

            //...
        }
    }
#endif

    return 0;
}

///2:每行/列中 小->大 sort
int EpipolarPonitSort(UnMatchedPoints *nSliceInput)
{
#ifdef CAMERA_HORIZONTAL
    //两个相机横置,竖线:960
    for (int ii = 0; ii < HEIGHT; ii++)
    {
        if(nSliceInput[ii].Slice.size() >1)
        {
            //sortAlgorithm(std::vector<Point2dPack> Slice)
            //SortAlgorithm;
            //对每一行/列的激光点进行 小->大排序
            //sortRowOrColumnLaserPoint(nSliceInput[ii].Slice);  //行
            //VectorSort();
            VectorSort(nSliceInput[ii].Slice);
        }

    }

    ///打印排序后的值
    #ifdef OUTPUT_POINTS_DEBUG
    for (int ii = 0; ii < HEIGHT; ii++)
    {
        if(nSliceInput[ii].Slice.size() > 0)
        {
            for (int jj = 0; jj < nSliceInput[ii].Slice.size(); jj++)
                printf("After Sort: Point(%f, %f):\n", nSliceInput[ii].Slice[jj].Pt2d.x, nSliceInput[ii].Slice[jj].Pt2d.y);
        }
    }
    #endif
#else
    //两个相机竖直放置,横线:1280
    for (int ii = 0; ii < WIDTH; ii++)
    {
        if(nSliceInput[ii].Slice.size() >1)
        {
            //sortAlgorithm(std::vector<Point2dPack> Slice)
            //SortAlgorithm;
            //对每一行/列的激光点进行 小->大排序
            //sortRowOrColumnLaserPoint(nSliceInput[ii].Slice);  //行
            //VectorSort();
            VectorSort(nSliceInput[ii].Slice);
        }

    }

    ///打印排序后的值
    #ifdef OUTPUT_POINTS_DEBUG
    for (int ii = 0; ii < WIDTH; ii++)
    {
        if(nSliceInput[ii].Slice.size() > 0)
        {
            for (int jj = 0; jj < nSliceInput[ii].Slice.size(); jj++)
                printf("After Sort: Point(%f, %f):\n", nSliceInput[ii].Slice[jj].Pt2d.x, nSliceInput[ii].Slice[jj].Pt2d.y);
        }
    }
    #endif
#endif

    return 0;
}

///3:左右图中的激光点匹配, 绑定每一行/列的激光点
int EpipolarPonitBind(UnMatchedPoints *nSlice0, UnMatchedPoints *nSlice1, BeMatchedPoints *outputSlice)
#ifdef MATCH_COMPARE  //旧的简易匹配
{
    BeMatchedPoints tempMatchedSlice;
    Point2dPack tempp2dPack[2];

    P2dPackMatch tempPackMatch;


#ifdef CAMERA_HORIZONTAL
    //两个相机横置,竖线:960
    for (int ii = 0; ii < HEIGHT; ii++)
#else
    //两个相机竖直放置,横线:1280
    for (int ii = 0; ii < WIDTH; ii++)
#endif
    {
        if((0 == nSlice0[ii].Slice.size()) || (0 == nSlice1[ii].Slice.size()))
        {
            //IMG0, IMG1任意一张图该行/列没有激光点
            continue;
        }

        for (int jj = 0; jj < nSlice0[ii].Slice.size()/*后续改为每一行/列匹配上的点的个数*/; jj++)
        {
            /**赋值**/
            if(nSlice0[ii].Slice.size() <= nSlice1[ii].Slice.size())
            {
                //outputSlice[ii].P2dMatchedSlice[0].p2dPack[0];
                //nSlice0[ii].Slice[jj];

                //插入一项匹配点P2dPackMatch
                outputSlice[ii].P2dMatchedSlice.push_back(tempPackMatch);
                //为新插入的匹配点赋值
                memcpy(&outputSlice[ii].P2dMatchedSlice[jj].p2dPack[0], &nSlice0[ii].Slice[jj], sizeof(Point2dPack)); //左侧的匹配点
                memcpy(&outputSlice[ii].P2dMatchedSlice[jj].p2dPack[1], &nSlice1[ii].Slice[jj], sizeof(Point2dPack)); //右侧的匹配点

                #ifdef OUTPUT_POINTS_DEBUG
                printf("Outpoint: (%lf, %lf)\n", outputSlice[ii].P2dMatchedSlice[0].p2dPack[0].Pt2d.x, outputSlice[ii].P2dMatchedSlice[0].p2dPack[0].Pt2d.y);
                #endif
            }
            //else
            //{
            //    tempMatchedPts.p2dPack[0].Pt2d = nSlice0[ii].Slice[jj].Pt2d;
            //}

            //push
            //tempMatchedPts.push_back(tempMatchedPts);
        }
    }

    return 0;
}
#else
{
    P2dPackMatch tempPackMatch;

    #ifdef CAMERA_HORIZONTAL
    //两个相机横置,竖线:960
    for (int ii = 0; ii < HEIGHT; ii++)
    #else
    //两个相机竖直放置,横线:1280
    for (int ii = 0; ii < WIDTH; ii++)
    #endif
    {
        if((0 == nSlice0[ii].Slice.size()) || (0 == nSlice1[ii].Slice.size()))
        {
            //IMG0, IMG1任意一张图该行/列没有激光点
            continue;
        }

        //参数1:输入,IMG0中的slice的所有点
        //参数2:输入,IMG1中的slice的所有点
        //参数3:输出,匹配完后的点
        MatchAndBindAlgorithm(nSlice0[ii].Slice, nSlice1[ii].Slice, outputSlice[ii].P2dMatchedSlice);
    }

    #ifdef POINTS_STATISTICES
    //输入的激光点总数
    AlignedInputLaserPointsCnt(nSlice0, 0);
    AlignedInputLaserPointsCnt(nSlice1, 1);
    //统计匹配的激光点的总数
    MatchedPointsCnt(outputSlice);
    #endif

    return 0;
}

#endif

//输入图像的激光点的总数
int AlignedInputLaserPointsCnt(UnMatchedPoints *nSlice0, int camID)
{
    int total_input_pts_cnt = 0;
    #ifdef CAMERA_HORIZONTAL
    //两个相机横置,竖线:960
    for (int ii = 0; ii < HEIGHT; ii++)
    #else
    //两个相机竖直放置,横线:1280
    for (int ii = 0; ii < WIDTH; ii++)
    #endif
    {
        total_input_pts_cnt += nSlice0[ii].Slice.size();

    }
    printf("\n输入图像%d中,有%d对匹配点\n\n", camID, total_input_pts_cnt);

    return total_input_pts_cnt;
}

//统计匹配的激光点的总数
int MatchedPointsCnt(BeMatchedPoints *outputSlice)
{
    int total_matched_pts_cnt = 0;
    #ifdef CAMERA_HORIZONTAL
    //两个相机横置,竖线:960
    for (int ii = 0; ii < HEIGHT; ii++)
    #else
    //两个相机竖直放置,横线:1280
    for (int ii = 0; ii < WIDTH; ii++)
    #endif
    {
        total_matched_pts_cnt += outputSlice[ii].P2dMatchedSlice.size();

    }
    printf("\n总共有%d对匹配点\n\n", total_matched_pts_cnt);

    return total_matched_pts_cnt;
}
