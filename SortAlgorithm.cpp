#include <algorithm>
#include "SortAlgorithm.h"
#include <iostream>

using namespace std;

//对每一行/列的激光点进行 小->大排序

int sortRowOrColumnLaserPoint(std::vector<Point2dPack> &inputData)  //行
{

    return 0;
}

//https://blog.csdn.net/m372897500/article/details/51478136
//八种排序算法效率比较
void merge(std::vector<Point2dPack> inputData, std::vector<Point2dPack> tmpVector,int lpos,int rpos,int rightend)
{
    int i,leftend,numelements,tmppos;
    leftend=rpos-1;
    tmppos=lpos;
    numelements=rightend-lpos+1;
    while(lpos<=leftend && rpos<=rightend)
    {
        if(inputData[lpos].Pt2d.x <= inputData[rpos].Pt2d.x) {
            tmpVector[tmppos++].Pt2d.x = inputData[lpos++].Pt2d.x;
        }
        else {
            tmpVector[tmppos++].Pt2d.x = inputData[rpos++].Pt2d.x;
        }

        while(lpos<=leftend) {
            tmpVector[tmppos++].Pt2d.x = inputData[lpos++].Pt2d.x;
        }

        while(rpos<=rightend) {
            tmpVector[tmppos++].Pt2d.x = inputData[rpos++].Pt2d.x;
        }

        for(i=0;i<numelements;i++,rightend--)
        {
            inputData[rightend].Pt2d.x = tmpVector[rightend].Pt2d.x;
        }
    }
}

void msort(std::vector<Point2dPack> inputData,std::vector<Point2dPack> tmpVector, int left,int right)
{
    int center;
    if(left<right)
    {
        center=(left+right)/2;
        msort(inputData,tmpVector,left,center);
        msort(inputData,tmpVector,center+1,right);
        merge(inputData,tmpVector,left,center+1,right);
    }
}

//归并排序
void MergeSort(std::vector<Point2dPack> inputData, int n)
{
    std::vector<Point2dPack> tmpVector(n);

    if(n > 1)
    {
        msort(inputData,tmpVector,0,n-1);
    }
    else
    {
        printf("need not to Sort!");
    }

     std::vector<Point2dPack>().swap(tmpVector);
}


//C++自带sort函数对vector容器元素进行排序
//自定义排序函数
bool sortFunc(const cv::Point2d &p1, const cv::Point2d &p2)
{
    return p1.x < p2.x;//升序排列
}

int VectorSort()
{
    std::vector<cv::Point2d> po;


    cv::Point2d p1(2, 4), p2(4, 3), p3(1, 7), p4(0,4);
    po.push_back(p1);
    po.push_back(p2);
    po.push_back(p3);
    po.push_back(p4);
    std::cout << "排序前： ";
    for (auto elem : po)
        std::cout << elem << " ";

    sort(po.begin(), po.end(), sortFunc);
    std::cout << std::endl << "排序后： " ;
    for (auto elem : po)
        cout << elem << " ";

    cout << endl;

    return 0;
}

//排序函数，升序排列
bool UppersortFunc(const Point2dPack &pack1, const Point2dPack &pack2)
{
    return pack1.Pt2d.x < pack2.Pt2d.x;//升序排列
}

int VectorSort(std::vector<Point2dPack> &inputData)
{
//    std::cout << "排序前： ";
//    for (auto elem : inputData)
//        std::cout << elem << " ";

    sort(inputData.begin(), inputData.end(), UppersortFunc);
//    std::cout << std::endl << "排序后： " ;
//    for (auto elem : inputData)
//        cout << elem << " ";
//
//    std::cout << endl;

    return 0;
}
