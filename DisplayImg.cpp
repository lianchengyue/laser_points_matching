#include "DisplayImg.h"

///3.5
int DisplayUnmatchedImg0(UnMatchedPoints *nSliceInput)
{
    cv::Mat disImg = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);

#ifdef CAMERA_HORIZONTAL
    //两个相机横置,竖线:960
    for (int ii = 0; ii < HEIGHT; ii++)
    {
        if(nSliceInput[ii].Slice.size() > 0)
        {
            for (int jj = 0; jj < nSliceInput[ii].Slice.size(); jj++)
            {
                int xx = (int)nSliceInput[ii].Slice[jj].Pt2d.x;
                int yy = ii;  // = //(int)nSliceInput[ii].Slice[jj].Pt2d.y;

                //上色
                if(0 == nSliceInput[ii].Slice[jj].PtBelonging % 4)
                {
                    disImg.at<cv::Vec3b>(yy, xx) = cv::Vec3b(255, 0, 0);
                }
                else if(1 == nSliceInput[ii].Slice[jj].PtBelonging % 4)
                {
                    disImg.at<cv::Vec3b>(yy, xx) = cv::Vec3b(0, 255, 0);
                }
                else if(2 == nSliceInput[ii].Slice[jj].PtBelonging % 4)
                {
                    disImg.at<cv::Vec3b>(yy, xx) = cv::Vec3b(0, 0, 255);
                }
                else if(3 == nSliceInput[ii].Slice[jj].PtBelonging % 4)
                {
                    disImg.at<cv::Vec3b>(yy, xx) = cv::Vec3b(255, 255, 0);
                }
            }
        }
    }
#else
    //两个相机横置,竖线:960
    for (int ii = 0; ii < WIDTH; ii++)
    {
        for (int jj = 0; jj < nSliceInput[ii].Slice.size(); jj++)
        {
            int xx = ii; // = //(int)nSliceInput[ii].Slice[jj].Pt2d.x;
            int yy = (int)nSliceInput[ii].Slice[jj].Pt2d.y;

            //上色
            if(0 == nSliceInput[ii].Slice[jj].PtBelonging % 4)
            {
                disImg.at<cv::Vec3b>(yy, xx) = cv::Vec3b(255, 0, 0);
            }
            else if(1 == nSliceInput[ii].Slice[jj].PtBelonging % 4)
            {
                disImg.at<cv::Vec3b>(yy, xx) = cv::Vec3b(0, 255, 0);
            }
            else if(2 == nSliceInput[ii].Slice[jj].PtBelonging % 4)
            {
                disImg.at<cv::Vec3b>(yy, xx) = cv::Vec3b(0, 0, 255);
            }
            else if(3 == nSliceInput[ii].Slice[jj].PtBelonging % 4)
            {
                disImg.at<cv::Vec3b>(yy, xx) = cv::Vec3b(255, 255, 0);
            }
        }
    }
#endif

    imshow("DisplayUnmatchedImg0", disImg);

    return 0;
}

int DisplayUnmatchedImg1(UnMatchedPoints *nSliceInput)
{
    cv::Mat disImg = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);

#ifdef CAMERA_HORIZONTAL
    //两个相机横置,竖线:960
    for (int ii = 0; ii < HEIGHT; ii++)
    {
        if(nSliceInput[ii].Slice.size() > 0)
        {
            for (int jj = 0; jj < nSliceInput[ii].Slice.size(); jj++)
            {
                int xx = (int)nSliceInput[ii].Slice[jj].Pt2d.x;
                int yy = ii;  // = //(int)nSliceInput[ii].Slice[jj].Pt2d.y;

                //上色
                if(0 == nSliceInput[ii].Slice[jj].PtBelonging % 4)
                {
                    disImg.at<cv::Vec3b>(yy, xx) = cv::Vec3b(255, 0, 0);
                }
                else if(1 == nSliceInput[ii].Slice[jj].PtBelonging % 4)
                {
                    disImg.at<cv::Vec3b>(yy, xx) = cv::Vec3b(0, 255, 0);
                }
                else if(2 == nSliceInput[ii].Slice[jj].PtBelonging % 4)
                {
                    disImg.at<cv::Vec3b>(yy, xx) = cv::Vec3b(0, 0, 255);
                }
                else if(3 == nSliceInput[ii].Slice[jj].PtBelonging % 4)
                {
                    disImg.at<cv::Vec3b>(yy, xx) = cv::Vec3b(255, 255, 0);
                }
            }
        }
    }
#else
    //两个相机横置,竖线:960
    for (int ii = 0; ii < WIDTH; ii++)
    {
        for (int jj = 0; jj < nSliceInput[ii].Slice.size(); jj++)
        {
            int xx = ii; // = //(int)nSliceInput[ii].Slice[jj].Pt2d.x;
            int yy = (int)nSliceInput[ii].Slice[jj].Pt2d.y;

            //上色
            if(0 == nSliceInput[ii].Slice[jj].PtBelonging % 4)
            {
                disImg.at<cv::Vec3b>(yy, xx) = cv::Vec3b(255, 0, 0);
            }
            else if(1 == nSliceInput[ii].Slice[jj].PtBelonging % 4)
            {
                disImg.at<cv::Vec3b>(yy, xx) = cv::Vec3b(0, 255, 0);
            }
            else if(2 == nSliceInput[ii].Slice[jj].PtBelonging % 4)
            {
                disImg.at<cv::Vec3b>(yy, xx) = cv::Vec3b(0, 0, 255);
            }
            else if(3 == nSliceInput[ii].Slice[jj].PtBelonging % 4)
            {
                disImg.at<cv::Vec3b>(yy, xx) = cv::Vec3b(255, 255, 0);
            }
        }
    }
#endif

    imshow("DisplayUnmatchedImg1", disImg);
    return 0;
}
